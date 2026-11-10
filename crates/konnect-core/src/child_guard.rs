//! Child processes that belong to one tool call and must not outlive this
//! process.
//!
//! `kill_on_drop` ends such a child when its tool call ends, and when the
//! runtime drops the call on an orderly stop. It does nothing when the process
//! itself is ended from outside (`TerminateProcess`, a service manager's hard
//! stop, a crash): no destructor runs, and the child carried on alone. That
//! left a Freerouting `java` behind a killed server.
//!
//! On Windows the children started through [`spawn`] are placed in one Job
//! Object that this process alone holds a handle to, with
//! `JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE`. The operating system closes that
//! handle when the process ends, however it ends, and then ends every process
//! still in the job. The server is not a member itself, so the detached route
//! job runners (`freerouting_job::spawn_detached`) are untouched: they are
//! never assigned, and a process is only in a job it was assigned to or
//! inherited from its parent.
//!
//! Windows also delivers a console's Ctrl+C, Ctrl+Break and close to every
//! process attached to that console, so a stop request typed at the server
//! reached the tool call's child directly and ended it mid-work, while the
//! server was waiting for that very call to finish. [`spawn`] therefore gives
//! the child a hidden console of its own (`CREATE_NO_WINDOW`), as the route
//! job runners already have: the server decides when its children end.
//!
//! On other platforms [`spawn`] only spawns. A terminal's Ctrl+C there goes to
//! the whole foreground process group, children included, and a service
//! manager may signal every process of the service; neither is changed here.
//! The orderly stop is still covered there by `kill_on_drop`, for the children
//! that set it; an outright kill of the server is not. Linux has
//! `PR_SET_PDEATHSIG` and a process group would serve elsewhere, but neither
//! could be exercised where this was written, so neither is claimed.

use tokio::process::{Child, Command};

/// Spawn `command` so that the child ends with this process.
///
/// The caller still sets `kill_on_drop(true)`: that is what ends the child
/// when the tool call itself ends first.
pub(crate) fn spawn(command: &mut Command) -> std::io::Result<Child> {
    #[cfg(windows)]
    {
        const CREATE_NO_WINDOW: u32 = 0x0800_0000;
        command.creation_flags(CREATE_NO_WINDOW);
    }
    let child = command.spawn()?;
    end_with_this_process(&child);
    Ok(child)
}

/// End every child started through [`spawn`] that is still running. For a
/// stop that abandons tool calls: their children go now, not whenever the
/// runtime gets round to dropping them.
pub fn end_children() {
    #[cfg(windows)]
    if let Some(job) = windows::process_job() {
        job.end_all();
    }
}

#[cfg(windows)]
fn end_with_this_process(child: &Child) {
    let (Some(job), Some(handle)) = (windows::process_job(), child.raw_handle()) else {
        return;
    };
    // Assigned after it started, so a child that started a process of its own
    // within these few microseconds would leave that one outside the job.
    // Neither `java` nor `kicad-cli` does.
    if let Err(error) = job.adopt(handle) {
        tracing::warn!(
            "child process {:?} was not tied to this server's lifetime: {error}",
            child.id()
        );
    }
}

#[cfg(not(windows))]
fn end_with_this_process(_child: &Child) {}

#[cfg(windows)]
mod windows {
    use std::os::windows::io::RawHandle;
    use std::sync::OnceLock;
    use windows_sys::Win32::Foundation::{CloseHandle, HANDLE};
    use windows_sys::Win32::System::JobObjects::{
        AssignProcessToJobObject, CreateJobObjectW, JobObjectExtendedLimitInformation,
        SetInformationJobObject, TerminateJobObject, JOBOBJECT_EXTENDED_LIMIT_INFORMATION,
        JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE,
    };

    /// A Job Object whose members end when its last handle closes.
    pub(super) struct KillOnCloseJob(HANDLE);

    // SAFETY: a kernel handle is a process-wide name, not thread-affine, and
    // the job functions used here are documented as callable from any thread.
    unsafe impl Send for KillOnCloseJob {}
    unsafe impl Sync for KillOnCloseJob {}

    impl KillOnCloseJob {
        pub(super) fn new() -> std::io::Result<Self> {
            // SAFETY: null attributes and name are the documented way to ask
            // for an unnamed job with a default, non-inheritable handle. Not
            // inheritable matters: a child holding the handle would keep the
            // job open after this process is gone.
            let handle = unsafe { CreateJobObjectW(std::ptr::null(), std::ptr::null()) };
            if handle.is_null() {
                return Err(std::io::Error::last_os_error());
            }
            let job = Self(handle);
            let mut limits = JOBOBJECT_EXTENDED_LIMIT_INFORMATION::default();
            limits.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
            // SAFETY: `limits` is a live, correctly sized structure of the
            // class named, and `job.0` is the handle just created.
            let set = unsafe {
                SetInformationJobObject(
                    job.0,
                    JobObjectExtendedLimitInformation,
                    std::ptr::from_ref(&limits).cast(),
                    std::mem::size_of::<JOBOBJECT_EXTENDED_LIMIT_INFORMATION>() as u32,
                )
            };
            if set == 0 {
                return Err(std::io::Error::last_os_error());
            }
            Ok(job)
        }

        pub(super) fn adopt(&self, process: RawHandle) -> std::io::Result<()> {
            // SAFETY: both handles are open: the job is owned by `self`, and
            // the caller borrows the process handle from a live child.
            if unsafe { AssignProcessToJobObject(self.0, process.cast()) } == 0 {
                return Err(std::io::Error::last_os_error());
            }
            Ok(())
        }

        pub(super) fn end_all(&self) {
            // SAFETY: `self.0` is an open job handle. A job with no members
            // is not an error.
            unsafe { TerminateJobObject(self.0, 1) };
        }
    }

    impl Drop for KillOnCloseJob {
        fn drop(&mut self) {
            // SAFETY: `self.0` is open and owned by this value alone.
            unsafe { CloseHandle(self.0) };
        }
    }

    /// The job for this process's own children, created on first use and kept
    /// until the process ends. `None` when the system refused one; the
    /// children then behave as they did before.
    pub(super) fn process_job() -> Option<&'static KillOnCloseJob> {
        static JOB: OnceLock<Option<KillOnCloseJob>> = OnceLock::new();
        JOB.get_or_init(|| match KillOnCloseJob::new() {
            Ok(job) => Some(job),
            Err(error) => {
                tracing::warn!(
                    "no job object for child processes ({error}); a killed server may leave them running"
                );
                None
            }
        })
        .as_ref()
    }
}

#[cfg(all(test, windows))]
mod tests {
    use super::windows::KillOnCloseJob;
    use std::os::windows::io::AsRawHandle;
    use std::process::Stdio;
    use std::time::{Duration, Instant};

    /// Something that runs for a minute and needs nothing installed.
    fn long_running() -> std::process::Command {
        let mut command = std::process::Command::new("ping");
        command
            .args(["-n", "60", "127.0.0.1"])
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::null());
        command
    }

    fn process_is_running(pid: u32) -> bool {
        let output = std::process::Command::new("tasklist")
            .args(["/FI", &format!("PID eq {pid}"), "/FO", "CSV", "/NH"])
            .output()
            .expect("run tasklist");
        String::from_utf8_lossy(&output.stdout).contains(&format!("\"{pid}\""))
    }

    fn exits_within(child: &mut std::process::Child, limit: Duration) -> bool {
        let deadline = Instant::now() + limit;
        while Instant::now() < deadline {
            if child.try_wait().expect("poll child").is_some() {
                return true;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        false
    }

    #[test]
    fn a_child_in_the_job_ends_when_the_last_job_handle_closes() {
        let job = KillOnCloseJob::new().expect("create job");
        let mut inside = long_running().spawn().expect("spawn ping");
        let mut outside = long_running().spawn().expect("spawn ping");
        job.adopt(inside.as_raw_handle()).expect("assign to job");

        // What the operating system does to the handle when the process ends.
        drop(job);

        let ended = exits_within(&mut inside, Duration::from_secs(10));
        let untouched = outside.try_wait().expect("poll child").is_none();
        let _ = outside.kill();
        let _ = outside.wait();
        let _ = inside.kill();
        assert!(ended, "the child in the job outlived the job");
        assert!(untouched, "a child outside the job was ended with it");
    }

    #[test]
    fn ending_the_job_ends_its_children_and_leaves_the_job_usable() {
        let job = KillOnCloseJob::new().expect("create job");
        let mut first = long_running().spawn().expect("spawn ping");
        job.adopt(first.as_raw_handle()).expect("assign to job");
        job.end_all();
        let ended = exits_within(&mut first, Duration::from_secs(10));
        let _ = first.kill();
        assert!(ended, "end_all left the child running");

        let mut second = long_running().spawn().expect("spawn ping");
        job.adopt(second.as_raw_handle())
            .expect("assign after end_all");
        drop(job);
        let ended = exits_within(&mut second, Duration::from_secs(10));
        let _ = second.kill();
        assert!(ended, "a child adopted after end_all outlived the job");
    }

    const PARENT_ROLE: &str = "KONNECT_CHILD_GUARD_TEST_PID_FILE";
    const GUARDED: &str = "KONNECT_CHILD_GUARD_TEST_GUARDED";

    /// Run this test binary again as a stand-in server that starts one child,
    /// records its pid and then waits to be killed.
    fn killed_parent_leaves_child_running(guarded: bool) -> bool {
        let dir = tempfile::tempdir().expect("tempdir");
        let pid_file = dir.path().join("child.pid");
        let mut parent = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "child_guard::tests::stand_in_server_for_the_killed_parent_tests",
                "--nocapture",
            ])
            .env(PARENT_ROLE, &pid_file)
            .env(GUARDED, if guarded { "1" } else { "0" })
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .expect("re-run the test binary");
        let deadline = Instant::now() + Duration::from_secs(30);
        let pid: u32 = loop {
            if let Some(pid) = std::fs::read_to_string(&pid_file)
                .ok()
                .and_then(|text| text.trim().parse().ok())
            {
                break pid;
            }
            assert!(Instant::now() < deadline, "the stand-in never started");
            std::thread::sleep(Duration::from_millis(50));
        };
        assert!(
            process_is_running(pid),
            "the child was gone before the kill"
        );

        // TerminateProcess: no destructor, no handler, nothing runs.
        parent.kill().expect("kill the stand-in");
        parent.wait().expect("reap the stand-in");

        let deadline = Instant::now() + Duration::from_secs(10);
        let mut running = true;
        while running && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(100));
            running = process_is_running(pid);
        }
        if running {
            let _ = std::process::Command::new("taskkill")
                .args(["/F", "/PID", &pid.to_string()])
                .output();
        }
        running
    }

    /// Not a test of its own: the body the two tests below run in a second
    /// process. Without the environment variable it does nothing.
    #[test]
    fn stand_in_server_for_the_killed_parent_tests() {
        let Some(pid_file) = std::env::var_os(PARENT_ROLE) else {
            return;
        };
        let guarded = std::env::var(GUARDED).as_deref() == Ok("1");
        let runtime = tokio::runtime::Runtime::new().expect("runtime");
        runtime.block_on(async move {
            let mut command = tokio::process::Command::from(long_running());
            command.kill_on_drop(true);
            let child = if guarded {
                super::spawn(&mut command)
            } else {
                command.spawn()
            }
            .expect("spawn ping");
            let pid = child.id().expect("child pid");
            let staged = std::path::PathBuf::from(&pid_file).with_extension("tmp");
            std::fs::write(&staged, pid.to_string()).expect("write pid");
            std::fs::rename(&staged, &pid_file).expect("publish pid");
            tokio::time::sleep(Duration::from_secs(120)).await;
            drop(child);
        });
    }

    #[test]
    fn a_child_started_through_the_guard_ends_when_its_parent_is_killed() {
        assert!(
            !killed_parent_leaves_child_running(true),
            "the guarded child outlived its killed parent"
        );
    }

    /// The contrast that shows the test above can fail: `kill_on_drop` alone
    /// does not survive a kill, which is the defect the job closes.
    #[test]
    fn a_child_started_without_the_guard_outlives_its_killed_parent() {
        assert!(
            killed_parent_leaves_child_running(false),
            "kill_on_drop alone ended the child of a killed parent"
        );
    }
}
