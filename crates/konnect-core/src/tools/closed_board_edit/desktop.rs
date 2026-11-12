//! The operating-system side of `run_closed_board_edit`: which window is the
//! PCB editor holding a board, asking that window to close, and starting the
//! standalone PCB editor again.
//!
//! KiCad's IPC API has no way to close an editor (`common.Control.quit` does
//! nothing on a standalone PCB editor and quits every KiCad window when the
//! editor belongs to the project manager), so closing goes through the
//! desktop: the same request a click on the window's close button sends. KiCad
//! then runs its own close path, prompts if anything is unsaved, and removes
//! its lock files. Nothing here terminates a process or touches a lock.
//!
//! Only the Windows implementation has been run against KiCad. Linux
//! (`wmctrl`) and macOS (`osascript`) follow the same shape and are untested.

use std::path::Path;

/// One top-level window of a KiCad GUI process.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct KiCadWindow {
    /// The platform's handle for the window, as text.
    pub id: String,
    pub pid: u32,
    /// `pcbnew` or `kicad`: the executable's name, lowercase, no extension.
    pub process: String,
    /// `None` where the platform cannot read window titles (macOS).
    pub title: Option<String>,
    /// The window this one belongs to, for a dialog.
    pub owner: Option<String>,
}

pub(crate) trait Desktop: Send + Sync {
    /// Every visible top-level window of a running `kicad` or `pcbnew`.
    fn kicad_windows(&self) -> Result<Vec<KiCadWindow>, String>;
    /// Ask `window` to close, as its close button does. Never forceful.
    fn request_close(&self, window: &KiCadWindow) -> Result<(), String>;
    fn process_alive(&self, pid: u32) -> bool;
    /// Start `binary` on `board` without waiting for it, returning its pid.
    fn launch(&self, binary: &Path, board: &Path) -> Result<u32, String>;
}

/// The PCB editor window for one board.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct EditorTarget {
    pub window: KiCadWindow,
    /// The window belongs to a `pcbnew` process of its own, rather than being
    /// one frame of the project manager.
    pub standalone: bool,
}

const GUI_PROCESSES: [&str; 2] = ["pcbnew", "kicad"];
/// KiCad's own name for the frame, as its English UI titles it.
const PCB_EDITOR_TITLE: &str = "PCB Editor";

/// What follows the board's name in a window title, when the title names it.
/// KiCad titles its frames `<name> — <frame>`, with `*` in front while the
/// document has unsaved changes.
fn title_remainder<'a>(title: &'a str, stem: &str) -> Option<&'a str> {
    let rest = title.trim_start_matches('*').strip_prefix(stem)?;
    (rest.is_empty() || rest.starts_with(' ')).then_some(rest)
}

/// Pick the PCB editor window for `board` out of KiCad's windows.
///
/// A window qualifies by title. KiCad translates the frame's name, so when no
/// window carries the English one, the only other accepted answer is a single
/// `pcbnew` process whose one candidate window names the board (or has no
/// readable title at all). Anything less certain is an error that says what
/// was seen: closing the wrong window is not a risk worth a guess.
pub(crate) fn select_pcb_editor(
    windows: &[KiCadWindow],
    board: &Path,
) -> Result<EditorTarget, String> {
    let stem = board
        .file_stem()
        .and_then(|stem| stem.to_str())
        .ok_or_else(|| format!("board path '{}' has no file name", board.display()))?;
    let frames: Vec<&KiCadWindow> = windows
        .iter()
        .filter(|window| window.owner.is_none())
        .filter(|window| GUI_PROCESSES.contains(&window.process.as_str()))
        .collect();
    let target = |window: &KiCadWindow| EditorTarget {
        window: window.clone(),
        standalone: window.process == "pcbnew",
    };
    let describe = |list: &[&KiCadWindow]| {
        list.iter()
            .map(|window| {
                format!(
                    "'{}' ({} pid {})",
                    window.title.as_deref().unwrap_or("<title unavailable>"),
                    window.process,
                    window.pid
                )
            })
            .collect::<Vec<_>>()
            .join(", ")
    };

    let exact: Vec<&KiCadWindow> = frames
        .iter()
        .copied()
        .filter(|window| {
            window
                .title
                .as_deref()
                .and_then(|title| title_remainder(title, stem))
                .is_some_and(|rest| rest.ends_with(PCB_EDITOR_TITLE))
        })
        .collect();
    match exact.as_slice() {
        [only] => return Ok(target(only)),
        [] => {}
        several => {
            return Err(format!(
                "{} PCB editor windows are titled for a board named '{stem}': {}. Konnect \
                 cannot tell which one holds this board",
                several.len(),
                describe(several)
            ))
        }
    }

    let fallback: Vec<&KiCadWindow> = frames
        .iter()
        .copied()
        .filter(|window| window.process == "pcbnew")
        .filter(|window| match window.title.as_deref() {
            Some(title) => title_remainder(title, stem).is_some(),
            None => true,
        })
        .collect();
    match fallback.as_slice() {
        [only] => Ok(target(only)),
        [] if frames.is_empty() => Err("no KiCad window was found on this desktop".to_string()),
        [] => Err(format!(
            "no window titled '{stem} \u{2014} {PCB_EDITOR_TITLE}' was found among KiCad's windows: {}",
            describe(&frames)
        )),
        several => Err(format!(
            "{} standalone PCB editor windows could hold a board named '{stem}': {}",
            several.len(),
            describe(several)
        )),
    }
}

/// The main window of the standalone PCB editor process `pid`, for a caller
/// that knows the process because it started it.
///
/// Matching the executable as well as the process id keeps a recycled id,
/// now some other program's, from being asked to close.
///
/// It is an error when another process carries the same window handle. That
/// happens where the platform cannot tell windows apart and the handle is the
/// application itself (macOS): the close request there goes to the
/// application, and with two editors running from it there is no saying which
/// one would answer. The other one may be the user's.
pub(crate) fn closable_editor_of(
    windows: &[KiCadWindow],
    pid: u32,
) -> Result<&KiCadWindow, String> {
    let window = windows
        .iter()
        .find(|window| window.pid == pid && window.process == "pcbnew" && window.owner.is_none())
        .ok_or_else(|| {
            format!("no standalone PCB editor window of process {pid} was found on this desktop")
        })?;
    match windows
        .iter()
        .find(|other| other.pid != pid && other.id == window.id)
    {
        None => Ok(window),
        Some(other) => Err(format!(
            "another PCB editor (pid {}) runs from the same application, and on this platform \
             a close request cannot be addressed to one of them; Konnect will not risk closing \
             the wrong one",
            other.pid
        )),
    }
}

/// Titles of the dialogs KiCad is showing on top of `editor`, where the
/// platform reports ownership. A save prompt is one of these.
pub(crate) fn blocking_dialogs(windows: &[KiCadWindow], editor: &KiCadWindow) -> Vec<String> {
    windows
        .iter()
        .filter(|window| window.pid == editor.pid && window.owner.as_ref() == Some(&editor.id))
        .map(|window| window.title.clone().unwrap_or_default())
        .collect()
}

/// Whether the editor window, and for a standalone editor its process, is
/// still there. A desktop that cannot be read counts as still open.
pub(crate) fn editor_still_open(desktop: &dyn Desktop, target: &EditorTarget) -> bool {
    match desktop.kicad_windows() {
        Err(_) => true,
        Ok(windows) => {
            windows
                .iter()
                .any(|window| window.pid == target.window.pid && window.id == target.window.id)
                || (target.standalone && desktop.process_alive(target.window.pid))
        }
    }
}

/// Start a GUI program detached from this server's standard streams (which
/// carry the MCP conversation) and reap it when it exits.
fn spawn_detached(binary: &Path, board: &Path) -> Result<u32, String> {
    let mut child = std::process::Command::new(binary)
        .arg(board)
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .spawn()
        .map_err(|error| format!("could not start '{}': {error}", binary.display()))?;
    let pid = child.id();
    std::thread::spawn(move || {
        let _ = child.wait();
    });
    Ok(pid)
}

/// The executable's name as [`KiCadWindow::process`] spells it.
#[cfg_attr(
    not(any(windows, target_os = "linux", target_os = "macos")),
    allow(dead_code)
)]
fn process_stem(path: &str) -> String {
    let name = Path::new(path.trim())
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or(path.trim())
        .to_ascii_lowercase();
    name.strip_suffix(".exe").unwrap_or(&name).to_string()
}

pub(crate) fn system() -> Box<dyn Desktop> {
    Box::new(platform::SystemDesktop)
}

#[cfg(windows)]
mod platform {
    use super::{process_stem, Desktop, KiCadWindow, GUI_PROCESSES};
    use std::collections::HashMap;
    use std::path::Path;
    use windows_sys::Win32::Foundation::{CloseHandle, HWND, LPARAM, WAIT_TIMEOUT};
    use windows_sys::Win32::System::Threading::{
        OpenProcess, QueryFullProcessImageNameW, WaitForSingleObject,
        PROCESS_QUERY_LIMITED_INFORMATION, PROCESS_SYNCHRONIZE,
    };
    use windows_sys::Win32::UI::WindowsAndMessaging::{
        EnumWindows, GetWindow, GetWindowTextW, GetWindowThreadProcessId, IsWindow,
        IsWindowVisible, PostMessageW, GW_OWNER, WM_CLOSE,
    };

    pub(super) struct SystemDesktop;

    unsafe extern "system" fn collect(window: HWND, list: LPARAM) -> i32 {
        // SAFETY: `list` is the `&mut Vec<HWND>` passed to `EnumWindows` below,
        // which calls this only for the duration of that call.
        let list = unsafe { &mut *(list as *mut Vec<HWND>) };
        list.push(window);
        1
    }

    fn handle_text(window: HWND) -> String {
        format!("{}", window as usize)
    }

    fn parse_handle(id: &str) -> Option<HWND> {
        id.parse::<usize>().ok().map(|value| value as HWND)
    }

    fn owning_pid(window: HWND) -> u32 {
        let mut pid = 0u32;
        // SAFETY: `pid` is a valid out pointer; an invalid window leaves it 0.
        unsafe { GetWindowThreadProcessId(window, &mut pid) };
        pid
    }

    fn process_name(pid: u32) -> Option<String> {
        // SAFETY: the handle is checked, used only here, and closed.
        unsafe {
            let process = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, 0, pid);
            if process.is_null() {
                return None;
            }
            let mut buffer = [0u16; 1024];
            let mut length = buffer.len() as u32;
            let ok = QueryFullProcessImageNameW(process, 0, buffer.as_mut_ptr(), &mut length);
            CloseHandle(process);
            (ok != 0).then(|| process_stem(&String::from_utf16_lossy(&buffer[..length as usize])))
        }
    }

    fn title(window: HWND) -> String {
        let mut buffer = [0u16; 512];
        // SAFETY: the buffer and its length describe valid writable memory.
        let length = unsafe { GetWindowTextW(window, buffer.as_mut_ptr(), buffer.len() as i32) };
        String::from_utf16_lossy(&buffer[..length.max(0) as usize])
    }

    impl Desktop for SystemDesktop {
        fn kicad_windows(&self) -> Result<Vec<KiCadWindow>, String> {
            let mut handles: Vec<HWND> = Vec::new();
            // SAFETY: `collect` only dereferences the pointer during this call.
            let ok =
                unsafe { EnumWindows(Some(collect), &mut handles as *mut Vec<HWND> as LPARAM) };
            if ok == 0 {
                return Err(format!(
                    "the desktop's windows could not be listed: {}",
                    std::io::Error::last_os_error()
                ));
            }
            let mut names: HashMap<u32, Option<String>> = HashMap::new();
            let mut windows = Vec::new();
            for handle in handles {
                // SAFETY: plain queries on a handle; a stale one answers 0.
                if unsafe { IsWindowVisible(handle) } == 0 {
                    continue;
                }
                let pid = owning_pid(handle);
                let Some(process) = names
                    .entry(pid)
                    .or_insert_with(|| process_name(pid))
                    .clone()
                    .filter(|name| GUI_PROCESSES.contains(&name.as_str()))
                else {
                    continue;
                };
                // SAFETY: as above.
                let owner = unsafe { GetWindow(handle, GW_OWNER) };
                windows.push(KiCadWindow {
                    id: handle_text(handle),
                    pid,
                    process,
                    title: Some(title(handle)),
                    owner: (!owner.is_null()).then(|| handle_text(owner)),
                });
            }
            Ok(windows)
        }

        fn request_close(&self, window: &KiCadWindow) -> Result<(), String> {
            let handle = parse_handle(&window.id)
                .ok_or_else(|| format!("'{}' is not a window handle", window.id))?;
            // A handle value can be reused once its window is gone: only the
            // window still owned by the same process is the one that was found.
            // SAFETY: plain queries and a posted message on a checked handle.
            unsafe {
                if IsWindow(handle) == 0 || owning_pid(handle) != window.pid {
                    return Err("the PCB editor window is no longer there".to_string());
                }
                if PostMessageW(handle, WM_CLOSE, 0, 0) == 0 {
                    return Err(format!(
                        "the close request could not be posted: {}",
                        std::io::Error::last_os_error()
                    ));
                }
            }
            Ok(())
        }

        fn process_alive(&self, pid: u32) -> bool {
            // SAFETY: the handle is checked, used only here, and closed.
            unsafe {
                let process = OpenProcess(PROCESS_SYNCHRONIZE, 0, pid);
                if process.is_null() {
                    return false;
                }
                let alive = WaitForSingleObject(process, 0) == WAIT_TIMEOUT;
                CloseHandle(process);
                alive
            }
        }

        fn launch(&self, binary: &Path, board: &Path) -> Result<u32, String> {
            super::spawn_detached(binary, board)
        }
    }
}

/// Untested against KiCad. `wmctrl` speaks to an X11 window manager (and to
/// XWayland); a KiCad running as a native Wayland client is not listed, and
/// the tool then refuses rather than guessing.
#[cfg(target_os = "linux")]
mod platform {
    use super::{process_stem, Desktop, KiCadWindow, GUI_PROCESSES};
    use std::path::Path;
    use std::process::Command;

    pub(super) struct SystemDesktop;

    fn wmctrl(args: &[&str]) -> Result<String, String> {
        let output = Command::new("wmctrl")
            .args(args)
            .output()
            .map_err(|error| {
                format!(
                    "wmctrl could not be run ({error}); it is what Konnect uses on Linux to find \
                 and close the PCB editor window"
                )
            })?;
        if !output.status.success() {
            return Err(format!(
                "wmctrl {} failed: {}",
                args.join(" "),
                String::from_utf8_lossy(&output.stderr).trim()
            ));
        }
        Ok(String::from_utf8_lossy(&output.stdout).into_owned())
    }

    /// One `wmctrl -l -p` line: window id, desktop, pid, host, title.
    pub(super) fn parse_line(line: &str) -> Option<(String, u32, String)> {
        let mut rest = line;
        let mut field = || {
            let trimmed = rest.trim_start();
            let end = trimmed.find(char::is_whitespace).unwrap_or(trimmed.len());
            let (head, tail) = trimmed.split_at(end);
            rest = tail;
            (!head.is_empty()).then(|| head.to_string())
        };
        let id = field()?;
        let _desktop = field()?;
        let pid = field()?.parse().ok()?;
        let _host = field()?;
        Some((id, pid, rest.trim().to_string()))
    }

    impl Desktop for SystemDesktop {
        fn kicad_windows(&self) -> Result<Vec<KiCadWindow>, String> {
            Ok(wmctrl(&["-l", "-p"])?
                .lines()
                .filter_map(parse_line)
                .filter_map(|(id, pid, title)| {
                    let process =
                        process_stem(&std::fs::read_to_string(format!("/proc/{pid}/comm")).ok()?);
                    GUI_PROCESSES
                        .contains(&process.as_str())
                        .then_some(KiCadWindow {
                            id,
                            pid,
                            process,
                            title: Some(title),
                            owner: None,
                        })
                })
                .collect())
        }

        fn request_close(&self, window: &KiCadWindow) -> Result<(), String> {
            wmctrl(&["-i", "-c", &window.id]).map(|_| ())
        }

        fn process_alive(&self, pid: u32) -> bool {
            Path::new(&format!("/proc/{pid}")).exists()
        }

        fn launch(&self, binary: &Path, board: &Path) -> Result<u32, String> {
            super::spawn_detached(binary, board)
        }
    }

    #[cfg(test)]
    mod tests {
        #[test]
        fn a_wmctrl_line_yields_the_window_its_pid_and_its_whole_title() {
            assert_eq!(
                super::parse_line("0x03800007  0 4242   host Yoshi \u{2014} PCB Editor"),
                Some((
                    "0x03800007".to_string(),
                    4242,
                    "Yoshi \u{2014} PCB Editor".to_string()
                ))
            );
            assert_eq!(super::parse_line("garbage"), None);
        }
    }
}

/// Untested against KiCad. macOS shows window titles only to processes with
/// the Accessibility permission, so a standalone `pcbnew` is identified by
/// its process alone and asked to quit by Apple event, which runs KiCad's
/// normal close path. An editor inside the project manager cannot be closed
/// on its own this way and is never selected.
#[cfg(target_os = "macos")]
mod platform {
    use super::{process_stem, Desktop, KiCadWindow, GUI_PROCESSES};
    use std::path::Path;
    use std::process::Command;

    pub(super) struct SystemDesktop;

    /// The `.app` bundle an executable inside it belongs to.
    pub(super) fn bundle_of(executable: &str) -> Option<&str> {
        executable
            .rfind(".app/")
            .map(|index| &executable[..index + 4])
    }

    impl Desktop for SystemDesktop {
        fn kicad_windows(&self) -> Result<Vec<KiCadWindow>, String> {
            let output = Command::new("ps")
                .args(["-A", "-o", "pid=,comm="])
                .output()
                .map_err(|error| format!("ps could not be run: {error}"))?;
            Ok(String::from_utf8_lossy(&output.stdout)
                .lines()
                .filter_map(|line| {
                    let (pid, executable) = line.trim().split_once(char::is_whitespace)?;
                    let process = process_stem(executable);
                    GUI_PROCESSES
                        .contains(&process.as_str())
                        .then_some(KiCadWindow {
                            id: executable.trim().to_string(),
                            pid: pid.parse().ok()?,
                            process,
                            title: None,
                            owner: None,
                        })
                })
                .collect())
        }

        fn request_close(&self, window: &KiCadWindow) -> Result<(), String> {
            let bundle = bundle_of(&window.id).ok_or_else(|| {
                format!(
                    "'{}' is not inside an application bundle, so it cannot be asked to quit",
                    window.id
                )
            })?;
            let script = format!(
                "tell application \"{}\" to quit",
                bundle.replace('\\', "\\\\").replace('"', "\\\"")
            );
            let output = Command::new("osascript")
                .args(["-e", &script])
                .output()
                .map_err(|error| format!("osascript could not be run: {error}"))?;
            if output.status.success() {
                Ok(())
            } else {
                Err(format!(
                    "osascript refused the quit request: {}",
                    String::from_utf8_lossy(&output.stderr).trim()
                ))
            }
        }

        fn process_alive(&self, pid: u32) -> bool {
            Command::new("ps")
                .args(["-p", &pid.to_string()])
                .output()
                .is_ok_and(|output| output.status.success())
        }

        fn launch(&self, binary: &Path, board: &Path) -> Result<u32, String> {
            super::spawn_detached(binary, board)
        }
    }

    #[cfg(test)]
    mod tests {
        #[test]
        fn the_bundle_is_the_innermost_app_directory_holding_the_executable() {
            assert_eq!(
                super::bundle_of(
                    "/Applications/KiCad/KiCad.app/Contents/Applications/pcbnew.app/Contents/MacOS/pcbnew"
                ),
                Some("/Applications/KiCad/KiCad.app/Contents/Applications/pcbnew.app")
            );
            assert_eq!(super::bundle_of("/usr/local/bin/pcbnew"), None);
        }
    }
}

#[cfg(not(any(windows, target_os = "linux", target_os = "macos")))]
mod platform {
    use super::{Desktop, KiCadWindow};
    use std::path::Path;

    pub(super) struct SystemDesktop;

    const UNSUPPORTED: &str = "closing a KiCad editor window is not implemented on this platform";

    impl Desktop for SystemDesktop {
        fn kicad_windows(&self) -> Result<Vec<KiCadWindow>, String> {
            Err(UNSUPPORTED.to_string())
        }
        fn request_close(&self, _window: &KiCadWindow) -> Result<(), String> {
            Err(UNSUPPORTED.to_string())
        }
        fn process_alive(&self, _pid: u32) -> bool {
            true
        }
        fn launch(&self, binary: &Path, board: &Path) -> Result<u32, String> {
            super::spawn_detached(binary, board)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn window(id: &str, pid: u32, process: &str, title: Option<&str>) -> KiCadWindow {
        KiCadWindow {
            id: id.to_string(),
            pid,
            process: process.to_string(),
            title: title.map(str::to_string),
            owner: None,
        }
    }

    const BOARD: &str = "/work/Yoshi_Link_RevA.kicad_pcb";

    #[test]
    fn the_standalone_editor_is_found_by_its_title_saved_or_modified() {
        for title in [
            "Yoshi_Link_RevA \u{2014} PCB Editor",
            "*Yoshi_Link_RevA \u{2014} PCB Editor",
            "Yoshi_Link_RevA [Read Only] \u{2014} PCB Editor",
        ] {
            let windows = [window("7", 100, "pcbnew", Some(title))];
            let found = select_pcb_editor(&windows, Path::new(BOARD)).unwrap();
            assert_eq!(found.window.id, "7");
            assert!(found.standalone);
        }
    }

    #[test]
    fn an_editor_inside_the_project_manager_is_found_and_not_called_standalone() {
        let windows = [
            window(
                "1",
                100,
                "kicad",
                Some("Yoshi_Link_RevA \u{2014} KiCad 10.0"),
            ),
            window(
                "2",
                100,
                "kicad",
                Some("Yoshi_Link_RevA \u{2014} PCB Editor"),
            ),
            window(
                "3",
                100,
                "kicad",
                Some("Yoshi_Link_RevA [Yoshi_Link_RevA/] \u{2014} Schematic Editor"),
            ),
        ];
        let found = select_pcb_editor(&windows, Path::new(BOARD)).unwrap();
        assert_eq!(found.window.id, "2");
        assert!(!found.standalone);
    }

    #[test]
    fn a_board_whose_name_only_starts_the_same_is_not_this_board() {
        let windows = [window(
            "1",
            100,
            "pcbnew",
            Some("Yoshi_Link_RevA_v2 \u{2014} PCB Editor"),
        )];
        assert!(select_pcb_editor(&windows, Path::new(BOARD)).is_err());
    }

    #[test]
    fn two_editors_titled_for_the_same_board_name_are_refused_as_ambiguous() {
        let windows = [
            window(
                "1",
                100,
                "pcbnew",
                Some("Yoshi_Link_RevA \u{2014} PCB Editor"),
            ),
            window(
                "2",
                200,
                "pcbnew",
                Some("Yoshi_Link_RevA \u{2014} PCB Editor"),
            ),
        ];
        let error = select_pcb_editor(&windows, Path::new(BOARD)).unwrap_err();
        assert!(error.contains("cannot tell which one"), "{error}");
    }

    #[test]
    fn a_translated_title_is_accepted_only_for_one_standalone_editor() {
        let translated = "Yoshi_Link_RevA \u{2014} Leiterplatteneditor";
        let alone = [window("1", 100, "pcbnew", Some(translated))];
        assert!(
            select_pcb_editor(&alone, Path::new(BOARD))
                .unwrap()
                .standalone
        );

        // Inside the project manager a translated title cannot be told apart
        // from the schematic editor's, so nothing is selected.
        let hosted = [window("1", 100, "kicad", Some(translated))];
        assert!(select_pcb_editor(&hosted, Path::new(BOARD)).is_err());
    }

    #[test]
    fn a_process_without_readable_titles_is_accepted_only_as_one_standalone_editor() {
        let one = [
            window("a", 100, "pcbnew", None),
            window("b", 200, "kicad", None),
        ];
        assert_eq!(
            select_pcb_editor(&one, Path::new(BOARD))
                .unwrap()
                .window
                .pid,
            100
        );
        let two = [
            window("a", 100, "pcbnew", None),
            window("b", 200, "pcbnew", None),
        ];
        assert!(select_pcb_editor(&two, Path::new(BOARD)).is_err());
        let manager_only = [window("b", 200, "kicad", None)];
        assert!(select_pcb_editor(&manager_only, Path::new(BOARD)).is_err());
    }

    #[test]
    fn a_started_editor_is_found_by_its_process_and_nothing_else_is() {
        let mut prompt = window("9", 100, "pcbnew", Some("Save Changes?"));
        prompt.owner = Some("7".to_string());
        let windows = [
            prompt,
            window("7", 100, "pcbnew", Some("board \u{2014} PCB Editor")),
            window("3", 200, "kicad", Some("board \u{2014} PCB Editor")),
            window("8", 400, "pcbnew", Some("other \u{2014} PCB Editor")),
        ];
        assert_eq!(closable_editor_of(&windows, 100).unwrap().id, "7");
        // The project manager is never a standalone editor, and a process id
        // with no editor window behind it finds nothing.
        assert!(closable_editor_of(&windows, 200).is_err());
        assert!(closable_editor_of(&windows, 300).is_err());
    }

    /// Where the handle is the application rather than a window, two editors
    /// from one application cannot be told apart by a close request.
    #[test]
    fn an_editor_sharing_its_handle_with_another_process_is_not_closable() {
        let app =
            "/Applications/KiCad/KiCad.app/Contents/Applications/pcbnew.app/Contents/MacOS/pcbnew";
        let ours = window(app, 100, "pcbnew", None);
        let manager = window(
            "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad",
            300,
            "kicad",
            None,
        );
        let alone = [ours.clone(), manager.clone()];
        assert_eq!(closable_editor_of(&alone, 100).unwrap().pid, 100);

        let beside_the_users = [ours, manager, window(app, 200, "pcbnew", None)];
        let error = closable_editor_of(&beside_the_users, 100).unwrap_err();
        assert!(error.contains("pid 200"), "{error}");
    }

    #[test]
    fn a_dialog_is_never_the_editor_and_is_reported_as_blocking_it() {
        let editor = window(
            "7",
            100,
            "pcbnew",
            Some("*Yoshi_Link_RevA \u{2014} PCB Editor"),
        );
        let mut prompt = window("9", 100, "pcbnew", Some("Save Changes?"));
        prompt.owner = Some("7".to_string());
        let windows = [prompt, editor.clone()];
        assert_eq!(
            select_pcb_editor(&windows, Path::new(BOARD))
                .unwrap()
                .window,
            editor
        );
        assert_eq!(blocking_dialogs(&windows, &editor), ["Save Changes?"]);
    }
}
