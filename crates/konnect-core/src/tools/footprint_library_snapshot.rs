//! Export authoritative saved-board footprints with KiCad's native library writer.
use super::{get_path, ToolDef};
use crate::{mcp::protocol::CallToolResult, tool};
use anyhow::{ensure, Context};
use konnect_sexp::writer::{read_consistent, write_atomic_if_unchanged};
use serde_json::json;
use sha2::{Digest, Sha256};
pub(super) fn tool() -> ToolDef {
    tool!("snapshot_saved_footprint_library",
    "Update an existing project footprint library entry from an authoritative saved board using native KiCad Python. Never writes the board. Requires a saved board, native Python executable, a project-local library and exact reference. Shared entry replacement refuses divergent placements; entry_name creates a separate snapshot instead. Does not relink the board. Dry run and exact plan revision required. Native export/readback verifies footprint equivalence before an atomic revision-checked library write.",
    json!({"type":"object","additionalProperties":false,"properties":{
      "board":{"type":"string"},"reference":{"type":"string"},"python_executable":{"type":"string"},
      "destination_library":{"type":"string","description":"Optional existing project-local .pretty directory for a new entry copied from a global or project footprint; requires entry_name and refuses existing destination"},
      "entry_name":{"type":"string","description":"Optional new project-library entry; leaves original shared entry untouched"},"dry_run":{"type":"boolean","default":true},"expected_plan_revision":{"type":"string"}
    },"required":["board","reference","python_executable"]}),
    |args,_ctx|async move {
        let board=get_path(args,"board")?; let old_board=read_consistent(&board)?;
        let reference=args["reference"].as_str().context("reference required")?;
        let parsed=konnect_sexp::parser::parse_sexp(&old_board)?;
        let mut ids=Vec::new();
        for fp in parsed.find_all("footprint") {
            let matched=fp.find_all("property").iter().any(|p|p.get(1).and_then(|v|v.as_str())==Some("Reference") && p.get(2).and_then(|v|v.as_str())==Some(reference));
            if matched { ids.push(fp.get(1).and_then(|v|v.as_str()).context("missing footprint ID")?.to_owned()); }
        }
        ensure!(ids.len()==1,"reference missing or ambiguous");
        let source=super::library::resolve_footprint_path(&ids[0],board.parent()).map_err(anyhow::Error::msg)?;
        let project=board.parent().context("missing board parent")?.canonicalize()?;
        let entry=args["entry_name"].as_str().unwrap_or("");
        validate_entry(entry)?;
        let dest=if let Some(directory)=args["destination_library"].as_str() {
            ensure!(!entry.is_empty(),"destination_library requires entry_name");
            new_destination(&project, std::path::Path::new(directory), entry)?
        } else {
            ensure!(source.canonicalize()?.starts_with(&project),"only existing project-local library entries may be replaced");
            if entry.is_empty(){source}else{source.with_file_name(format!("{entry}.kicad_mod"))}
        };
        let old=if dest.exists(){Some(read_consistent(&dest)?)}else{None};
        let tmp=tempfile::tempdir()?; let dir=tmp.path().join("snapshot.pretty");std::fs::create_dir(&dir)?;
        let output=std::process::Command::new(args["python_executable"].as_str().context("python executable required")?)
            .args(["-c",include_str!("footprint_library_snapshot.py")]).arg(&board).arg(reference).arg(&dir).arg(entry).output()?;
        ensure!(output.status.success(),"native export failed: {}",String::from_utf8_lossy(&output.stderr));
        let report:serde_json::Value=serde_json::from_slice(&output.stdout).context("native export did not return JSON")?;
        ensure!(!entry.is_empty() || report["conflicting_references"].as_array().context("missing coverage")?.is_empty(),"shared library placements differ: {}",report);
        let candidate=dir.join(format!("{}.kicad_mod",report["name"].as_str().context("missing name")?));
        ensure!(candidate.file_name()==dest.file_name(),"entry name changed");
        let new=read_consistent(&candidate)?;konnect_sexp::parser::parse_sexp(&new)?;
        let mut h=Sha256::new();for b in [board.to_string_lossy().as_bytes(),dest.to_string_lossy().as_bytes(),old_board.as_bytes(),old.as_deref().unwrap_or("<absent>").as_bytes(),reference.as_bytes(),entry.as_bytes(),args["python_executable"].as_str().unwrap().as_bytes()]{h.update(b);}
        let revision=format!("{:x}",h.finalize());let apply=!args["dry_run"].as_bool().unwrap_or(true);
        if apply {ensure!(args["expected_plan_revision"].as_str()==Some(&revision),"stale or missing plan revision");ensure!(read_consistent(&board)?==old_board,"saved board changed during export");if let Some(old)=&old {write_atomic_if_unchanged(&dest,old,&new)?;} else {use std::io::Write;let mut f=tempfile::NamedTempFile::new_in(dest.parent().unwrap())?;f.write_all(new.as_bytes())?;f.as_file().sync_all()?;f.persist_noclobber(&dest)?;}ensure!(read_consistent(&dest)?==new,"library readback differs");}
        Ok(CallToolResult::json(&json!({"status":if apply{"complete"}else{"ready"},"dry_run":!apply,"plan_revision":revision,"library_path":dest,"native_verification":report,"board_unchanged":true})))
    })
}

pub(super) fn link_tool() -> ToolDef {
    use super::{with_board_ipc_classified, BoardAccess};
    use konnect_ipc::{builders, gen::kiapi};
    use prost::Message;
    tool!("link_board_footprint_library",
    "Change only the library identifier of an existing live footprint. Does not refresh or replace geometry. Caller must first verify the target library is the intended exact footprint. Target must resolve in the project. Dry run and exact revision required; full footprint readback after one undo entry. No file fallback.",
    json!({"type":"object","additionalProperties":false,"properties":{"board":{"type":"string"},"reference":{"type":"string"},"library_id":{"type":"string"},"dry_run":{"type":"boolean","default":true},"expected_plan_revision":{"type":"string"}},"required":["board","reference","library_id"]}),
    |args,ctx|async move {
      let path=get_path(args,"board")?; let id=args["library_id"].as_str().context("missing library_id")?;
      let (nickname,entry)=id.split_once(':').context("Library:Footprint required")?;
      ensure!(!nickname.is_empty() && !entry.is_empty(),"empty library ID");
      let target=super::library::resolve_footprint_path(id,path.parent()).map_err(anyhow::Error::msg)?;
      let library=read_consistent(&target)?;konnect_sexp::parser::parse_sexp(&library)?;
      let nickname=nickname.to_owned();let entry=entry.to_owned();let args=args.clone();
      let result=with_board_ipc_classified(ctx,&path,move|c|{
        let reference=args["reference"].as_str().context("missing reference")?;
        let old=find_footprint(c,reference)?;let mut new=old.clone();
        new.definition.as_mut().context("missing definition")?.id=Some(kiapi::common::types::LibraryIdentifier{library_nickname:nickname,entry_name:entry});
        let mut h=Sha256::new();h.update(old.encode_to_vec());h.update(new.encode_to_vec());h.update(library.as_bytes());let revision=format!("{:x}",h.finalize());
        let apply=!args["dry_run"].as_bool().unwrap_or(true);
        if apply {
          ensure!(args["expected_plan_revision"].as_str()==Some(&revision),"stale or missing plan revision");
          ensure!(read_consistent(&target)?==library,"library changed");
          c.run_commit("Link footprint library",|c|c.update_items(vec![builders::pack_any(&new,"kiapi.board.types.FootprintInstance")]))?;
          let actual=find_footprint(c,reference)?;
          if actual!=new {c.run_commit("Restore footprint library link",|c|c.update_items(vec![builders::pack_any(&old,"kiapi.board.types.FootprintInstance")]))?;ensure!(find_footprint(c,reference)?==old,"library link readback and restoration failed");anyhow::bail!("readback differed; original restored");}
        }
        Ok(json!({"status":if apply{"complete"}else{"ready"},"plan_revision":revision,"dry_run":!apply,"reference":reference,"library_id":args["library_id"],"geometry_unchanged":true}))
      }).await?;
      match result {Ok(v)=>Ok(CallToolResult::json(&v)),Err(e)=>Ok(CallToolResult::error(format!("live library link refused: {e}")))}
    }).with_board_access(BoardAccess::LiveOnly)
}

fn find_footprint(
    c: &konnect_ipc::client::KiCadIpcClient,
    reference: &str,
) -> anyhow::Result<konnect_ipc::gen::kiapi::board::types::FootprintInstance> {
    use konnect_ipc::{builders, gen::kiapi};
    use prost::Message;
    let mut found = Vec::new();
    for item in c.get_items(kiapi::common::types::KiCadObjectType::KotPcbFootprint)? {
        ensure!(
            builders::any_is(&item, "kiapi.board.types.FootprintInstance"),
            "unexpected item type"
        );
        let fp = kiapi::board::types::FootprintInstance::decode(item.value.as_slice())?;
        if fp
            .reference_field
            .as_ref()
            .and_then(|f| f.text.as_ref())
            .and_then(|t| t.text.as_ref())
            .is_some_and(|t| t.text == reference)
        {
            found.push(fp);
        }
    }
    ensure!(found.len() == 1, "reference missing or ambiguous");
    Ok(found.remove(0))
}

fn new_destination(
    project: &std::path::Path,
    directory: &std::path::Path,
    entry: &str,
) -> anyhow::Result<std::path::PathBuf> {
    validate_entry(entry)?;
    ensure!(!entry.is_empty(), "destination requires entry_name");
    let library = directory.canonicalize()?;
    ensure!(
        library.is_dir()
            && library.extension().is_some_and(|e| e == "pretty")
            && library.starts_with(project.canonicalize()?),
        "destination must be an existing project-local .pretty directory"
    );
    let dest = library.join(format!("{entry}.kicad_mod"));
    ensure!(
        !dest.exists(),
        "cross-library export refuses existing destination"
    );
    Ok(dest)
}

fn validate_entry(entry: &str) -> anyhow::Result<()> {
    ensure!(
        entry.is_empty()
            || (entry
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || "_.-".contains(c))
                && entry != "."
                && entry != ".."),
        "invalid entry name"
    );
    Ok(())
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn snapshot_entries_cannot_escape_library() {
        for value in ["../x", "a/b", "a\\b", ".", "..", "bad:name", "bad\nname"] {
            assert!(validate_entry(value).is_err(), "{value}");
        }
        for value in ["", "C9_C0402_V1", "SOT-23_2.9"] {
            assert!(validate_entry(value).is_ok());
        }
    }
    #[test]
    fn snapshot_tools_default_to_reviewed_plans() {
        let snapshot = tool();
        let link = link_tool();
        assert_eq!(snapshot.name, "snapshot_saved_footprint_library");
        assert_eq!(link.name, "link_board_footprint_library");
    }
    #[test]
    fn snapshot_new_destination_is_project_local_and_non_overwriting() {
        let project = tempfile::tempdir().unwrap();
        let library = project.path().join("parts.pretty");
        std::fs::create_dir(&library).unwrap();
        let target = new_destination(project.path(), &library, "USB_Variant").unwrap();
        std::fs::write(&target, "sentinel").unwrap();
        assert!(new_destination(project.path(), &library, "USB_Variant").is_err());
        assert!(new_destination(project.path(), &library, "../escape").is_err());
        assert!(new_destination(project.path(), &library, "").is_err());
        let outside = tempfile::tempdir().unwrap();
        let dir = outside.path().join("parts.pretty");
        std::fs::create_dir(&dir).unwrap();
        assert!(new_destination(project.path(), &dir, "USB_Variant").is_err());
        assert_eq!(std::fs::read_to_string(target).unwrap(), "sentinel");
    }
}
