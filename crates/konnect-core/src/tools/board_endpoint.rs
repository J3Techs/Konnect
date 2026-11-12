//! Which KiCad endpoint holds a requested board.
//!
//! One KiCad process serves one IPC endpoint, and a machine can run several:
//! the first listens on `api.sock`, each later one on `api-<pid>.sock` beside
//! it (see `konnect_ipc::sibling_ipc_addresses`). A standalone PCB editor
//! started beside the project manager is the common case. A server that asks
//! only the address it was configured with is told, truthfully, that the
//! board is not open there, and then treats a board another process holds as
//! closed.
//!
//! So a board-targeted call asks each endpoint in turn and runs against the
//! one that names the board. When none does, the caller is handed the answer
//! that leaves it the least room: an endpoint that did not settle the
//! question outranks one that said "not open", which outranks one with no PCB
//! editor, which outranks one that was never reached. With a single endpoint
//! that is its own answer, unchanged.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use konnect_ipc::client::KiCadIpcClient;
use konnect_ipc::gen::kiapi::common::types::DocumentSpecifier;

use super::BoardEditorAbsence;

/// How long an endpoint may take to list its open documents when it is
/// neither the configured one nor the one that last held this board. Those
/// two keep the client's default bound; a further KiCad that is not
/// answering must not hold every board call up for that long.
const SIBLING_PROBE_TIMEOUT: Duration = Duration::from_secs(5);

/// The endpoints to ask about one board, in the order to ask them.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Candidates {
    pub addresses: Vec<String>,
    /// How many leading addresses are given the client's default bound.
    pub patient: usize,
}

/// The endpoint that last held each board, so the next call asks it first.
///
/// An ordering hint and nothing more: the endpoint is asked again on every
/// call, and what it answers then is what counts.
#[derive(Clone, Default)]
pub(crate) struct BoardEndpoints {
    last: Arc<Mutex<HashMap<PathBuf, String>>>,
}

impl BoardEndpoints {
    pub(crate) fn remember(&self, board: &Path, address: &str) {
        self.last
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .insert(super::board_session::board_key(board), address.to_string());
    }

    /// The endpoint that last held `board`, if one did.
    pub(crate) fn last_holder(&self, board: &Path) -> Option<String> {
        self.last
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .get(&super::board_session::board_key(board))
            .cloned()
    }

    /// `addresses` (the configured endpoint first) reordered for `board`:
    /// the endpoint that last held it, when it is still listed, goes first.
    pub(crate) fn candidates_for(&self, board: &Path, addresses: Vec<String>) -> Candidates {
        let remembered = self
            .last_holder(board)
            .and_then(|address| addresses.iter().position(|listed| *listed == address));
        match remembered {
            Some(index) if index > 0 => {
                let mut addresses = addresses;
                let holder = addresses.remove(index);
                addresses.insert(0, holder);
                Candidates {
                    addresses,
                    patient: 2,
                }
            }
            _ => Candidates {
                addresses,
                patient: 1,
            },
        }
    }
}

/// The endpoint that names the requested board, and the client bound to it.
pub(crate) struct Located {
    pub address: String,
    pub client: KiCadIpcClient,
    pub document: DocumentSpecifier,
}

impl std::fmt::Debug for Located {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("Located")
            .field("address", &konnect_ipc::redact_endpoint(&self.address))
            .finish_non_exhaustive()
    }
}

/// No endpoint named the board: the answer to act on, and who gave it.
#[derive(Debug)]
pub(crate) struct Missed {
    pub address: String,
    pub error: anyhow::Error,
}

/// What an endpoint that did not name the board established, least
/// restrictive first.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Answer {
    /// The request never arrived: nothing listens there.
    Unreachable,
    /// KiCad answered that it has no PCB editor to ask.
    NoBoardEditor,
    /// KiCad listed its boards and this one is not among them.
    NotOpen,
    /// KiCad is there and did not settle whether it holds the board.
    Unsettled,
}

fn answer(error: &anyhow::Error) -> Answer {
    if konnect_ipc::is_transport_unreachable(error) {
        return Answer::Unreachable;
    }
    if BoardEditorAbsence::from_error(error).is_some() {
        return Answer::NoBoardEditor;
    }
    let proves_not_open = error
        .chain()
        .find_map(|cause| cause.downcast_ref::<konnect_ipc::BoardTargetError>())
        .is_some_and(konnect_ipc::BoardTargetError::proves_not_open);
    if proves_not_open {
        Answer::NotOpen
    } else {
        Answer::Unsettled
    }
}

/// Ask each candidate for `requested` and stop at the first that names it.
pub(crate) fn locate_board(candidates: &Candidates, requested: &Path) -> Result<Located, Missed> {
    // No endpoint at all is the blank one: the client reports it unconfigured.
    let blank = [String::new()];
    let addresses = if candidates.addresses.is_empty() {
        &blank[..]
    } else {
        &candidates.addresses[..]
    };
    let mut missed: Option<(Answer, Missed)> = None;
    for (index, address) in addresses.iter().enumerate() {
        let client = KiCadIpcClient::new(address);
        let found = if index < candidates.patient {
            client.find_open_board(requested)
        } else {
            client.find_open_board_within(requested, SIBLING_PROBE_TIMEOUT)
        };
        match found {
            Ok(document) => {
                return Ok(Located {
                    address: address.clone(),
                    client,
                    document,
                })
            }
            Err(error) => {
                let answer = answer(&error);
                // Strictly greater: among equal answers the earlier endpoint
                // speaks, which for one endpoint is the only one.
                if missed.as_ref().is_none_or(|(kept, _)| answer > *kept) {
                    missed = Some((
                        answer,
                        Missed {
                            address: address.clone(),
                            error,
                        },
                    ));
                }
            }
        }
    }
    Err(missed.expect("at least one endpoint was asked").1)
}

/// What one endpoint said when asked which boards it has open.
#[derive(Debug)]
pub(crate) struct EndpointBoards {
    pub address: String,
    /// The endpoint this server was configured with, or found for itself.
    pub configured: bool,
    /// The configured endpoint's Ping. Further endpoints are not pinged:
    /// their answer to the open-documents query says the same.
    pub ping: Option<konnect_ipc::PingOutcome>,
    /// KiCad answered at this endpoint.
    pub available: bool,
    pub open_boards: Vec<PathBuf>,
    pub open_boards_error: Option<String>,
}

/// Ask every endpoint in `addresses` (the configured one first) for its open
/// boards.
///
/// The configured endpoint is always reported, answered or not, exactly as a
/// server with one endpoint reports it. A further endpoint is reported only
/// when something answered there: a socket file its KiCad left behind is not
/// a finding, and is passed over without a word.
pub(crate) fn survey(addresses: &[String]) -> Vec<EndpointBoards> {
    let mut reports = Vec::new();
    for (index, address) in addresses.iter().enumerate() {
        let client = KiCadIpcClient::new(address);
        if index == 0 {
            let ping = client.ping_outcome();
            let available = ping.is_responsive();
            let (open_boards, open_boards_error) = if available {
                match client.get_open_board_paths() {
                    Ok(paths) => (paths, None),
                    Err(error) => (Vec::new(), Some(format!("{error:#}"))),
                }
            } else {
                (Vec::new(), None)
            };
            reports.push(EndpointBoards {
                address: address.clone(),
                configured: true,
                ping: Some(ping),
                available,
                open_boards,
                open_boards_error,
            });
            continue;
        }
        match client.get_open_board_paths_within(SIBLING_PROBE_TIMEOUT) {
            Ok(open_boards) => reports.push(EndpointBoards {
                address: address.clone(),
                configured: false,
                ping: None,
                available: true,
                open_boards,
                open_boards_error: None,
            }),
            Err(error) if konnect_ipc::is_transport_unreachable(&error) => {}
            Err(error) => reports.push(EndpointBoards {
                address: address.clone(),
                configured: false,
                ping: None,
                // A status from KiCad proves it is there; a reply that never
                // came does not.
                available: konnect_ipc::ApiStatusError::from_error(&error).is_some(),
                open_boards: Vec::new(),
                open_boards_error: Some(format!("{error:#}")),
            }),
        }
    }
    reports
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::MockIpcServer;
    use crate::tools::pcb_board::board_mock::spawn_kicad_holding_board;
    use konnect_ipc::gen::kiapi;

    fn board(dir: &Path, name: &str) -> PathBuf {
        let board = dir.join(name);
        std::fs::write(&board, "").unwrap();
        board
    }

    fn candidates(addresses: &[&str]) -> Candidates {
        Candidates {
            addresses: addresses
                .iter()
                .map(|address| address.to_string())
                .collect(),
            patient: 1,
        }
    }

    /// An endpoint that answers every command with one status, as the
    /// project manager does (`AS_UNHANDLED`) with no PCB editor open.
    fn spawn_answering(status: kiapi::common::ApiStatusCode) -> MockIpcServer {
        MockIpcServer::spawn("status-only", move |_| kiapi::common::ApiResponse {
            status: Some(kiapi::common::ApiResponseStatus {
                status: status as i32,
                error_message: String::new(),
            }),
            header: None,
            message: None,
        })
    }

    const NOTHING_LISTENS: &str = "inproc://konnect-core-board-endpoint-nothing-listens";

    #[test]
    fn the_endpoint_that_holds_the_board_is_chosen_over_the_configured_one() {
        let dir = tempfile::tempdir().unwrap();
        let wanted = board(dir.path(), "wanted.kicad_pcb");
        let other = board(dir.path(), "other.kicad_pcb");
        let manager = spawn_answering(kiapi::common::ApiStatusCode::AsUnhandled);
        let elsewhere = spawn_kicad_holding_board(&other, |_| None);
        let editor = spawn_kicad_holding_board(&wanted, |_| None);

        let located = locate_board(
            &candidates(&[
                manager.address(),
                NOTHING_LISTENS,
                elsewhere.address(),
                editor.address(),
            ]),
            &wanted,
        )
        .unwrap_or_else(|missed| panic!("{:#}", missed.error));

        assert_eq!(located.address, editor.address());
        // The client handed back is bound to that endpoint's document.
        located.client.ensure_board_is_active(&wanted).unwrap();
    }

    #[test]
    fn one_endpoint_reports_its_own_answer_whatever_that_is() {
        let dir = tempfile::tempdir().unwrap();
        let wanted = board(dir.path(), "wanted.kicad_pcb");
        let other = board(dir.path(), "other.kicad_pcb");

        let unreachable = locate_board(&candidates(&[NOTHING_LISTENS]), &wanted).unwrap_err();
        assert_eq!(unreachable.address, NOTHING_LISTENS);
        assert!(konnect_ipc::is_transport_unreachable(&unreachable.error));

        let blank = locate_board(&candidates(&[""]), &wanted).unwrap_err();
        assert!(konnect_ipc::is_transport_unreachable(&blank.error));

        let elsewhere = spawn_kicad_holding_board(&other, |_| None);
        let not_open = locate_board(&candidates(&[elsewhere.address()]), &wanted).unwrap_err();
        assert_eq!(answer(&not_open.error), Answer::NotOpen);
    }

    #[test]
    fn a_miss_reports_the_answer_that_permits_least() {
        let dir = tempfile::tempdir().unwrap();
        let wanted = board(dir.path(), "wanted.kicad_pcb");
        let other = board(dir.path(), "other.kicad_pcb");
        let manager = spawn_answering(kiapi::common::ApiStatusCode::AsUnhandled);
        let elsewhere = spawn_kicad_holding_board(&other, |_| None);
        let busy = spawn_answering(kiapi::common::ApiStatusCode::AsBusy);

        // A stale socket never outvotes an endpoint that answered.
        let missed =
            locate_board(&candidates(&[NOTHING_LISTENS, manager.address()]), &wanted).unwrap_err();
        assert_eq!(missed.address, manager.address());
        assert_eq!(answer(&missed.error), Answer::NoBoardEditor);

        // An endpoint that lists other boards says more than one with no editor.
        let missed = locate_board(
            &candidates(&[manager.address(), elsewhere.address(), NOTHING_LISTENS]),
            &wanted,
        )
        .unwrap_err();
        assert_eq!(missed.address, elsewhere.address());
        assert_eq!(answer(&missed.error), Answer::NotOpen);

        // An endpoint that would not say may be the one holding the board, so
        // "not open" from another is not the answer to act on.
        let missed = locate_board(
            &candidates(&[elsewhere.address(), busy.address(), manager.address()]),
            &wanted,
        )
        .unwrap_err();
        assert_eq!(missed.address, busy.address());
        assert_eq!(answer(&missed.error), Answer::Unsettled);
    }

    #[test]
    fn a_survey_reports_the_configured_endpoint_and_every_other_that_answered() {
        let dir = tempfile::tempdir().unwrap();
        let wanted = board(dir.path(), "wanted.kicad_pcb");
        let manager = spawn_answering(kiapi::common::ApiStatusCode::AsUnhandled);
        let editor = spawn_kicad_holding_board(&wanted, |_| None);
        let addresses = [
            NOTHING_LISTENS.to_string(),
            manager.address().to_string(),
            "inproc://konnect-core-board-endpoint-also-stale".to_string(),
            editor.address().to_string(),
        ];

        let reports = survey(&addresses);

        let listed: Vec<&str> = reports
            .iter()
            .map(|report| report.address.as_str())
            .collect();
        assert_eq!(
            listed,
            [NOTHING_LISTENS, manager.address(), editor.address()],
            "the configured endpoint is always reported; a stale sibling never is"
        );
        assert!(reports[0].configured && !reports[0].available);
        assert!(matches!(
            reports[0].ping,
            Some(konnect_ipc::PingOutcome::Unreachable { .. })
        ));
        assert!(reports[1].available && reports[1].open_boards.is_empty());
        assert!(reports[1].open_boards_error.is_some());
        assert!(reports[2].available && reports[2].open_boards_error.is_none());
        assert_eq!(reports[2].open_boards, [wanted]);
    }

    /// The situation this module exists for, on real sockets: the configured
    /// `api.sock` belongs to one KiCad process (here, one that has exited and
    /// left the file behind) and the board is open in another that listens on
    /// `api-<pid>.sock` beside it.
    #[cfg(unix)]
    mod beside_the_configured_socket {
        use super::*;
        use crate::tools::live_board::{self, LiveBoard};
        use crate::tools::pcb_board::board_mock::{ctx_talking_to, spawn_kicad_holding_board_at};
        use crate::tools::pcb_board::{
            attempt_ipc_write, refuse_if_board_open_in_kicad, BoardWrite,
        };

        struct Sockets {
            dir: tempfile::TempDir,
        }

        impl Sockets {
            fn new() -> Self {
                Sockets {
                    dir: tempfile::tempdir().unwrap(),
                }
            }

            fn address(&self, name: &str) -> String {
                format!("ipc://{}", self.dir.path().join(name).display())
            }

            /// A socket file with nothing listening, as a finished KiCad
            /// leaves it.
            fn stale(&self, name: &str) -> String {
                drop(std::os::unix::net::UnixListener::bind(self.dir.path().join(name)).unwrap());
                self.address(name)
            }
        }

        #[tokio::test]
        async fn a_board_open_in_a_second_kicad_process_is_reached_and_counts_as_live() {
            let sockets = Sockets::new();
            let wanted = board(sockets.dir.path(), "wanted.kicad_pcb");
            let configured = sockets.stale("api.sock");
            let _also_stale = sockets.stale("api-11.sock");
            let editor = spawn_kicad_holding_board_at(sockets.address("api-4242.sock"), &wanted);
            let ctx = ctx_talking_to(configured.clone());

            assert_eq!(
                ctx.ipc_addresses(),
                [
                    configured.clone(),
                    sockets.address("api-11.sock"),
                    editor.address().to_string()
                ]
            );

            // A live read runs against the editor that holds the board.
            let started = std::time::Instant::now();
            let observed =
                live_board::observe(&ctx, &wanted, |client, _| client.get_open_board_paths())
                    .await
                    .unwrap();
            let LiveBoard::Answered(open_boards) = observed else {
                panic!("the board is open at an endpoint beside the configured one")
            };
            assert_eq!(
                open_boards,
                std::slice::from_ref(&wanted),
                "the callback ran against it"
            );
            assert!(
                started.elapsed() < Duration::from_secs(3),
                "stale sockets must be passed over at once: {:?}",
                started.elapsed()
            );
            assert!(ctx.board_session.was_observed_live(&wanted));
            assert_eq!(
                ctx.board_endpoints.last_holder(&wanted).as_deref(),
                Some(editor.address())
            );
            assert_eq!(ctx.ipc_address_for_board(&wanted).await, editor.address());

            // A file-only edit is refused, and a live-preferred write goes to
            // the editor instead of the file.
            assert!(refuse_if_board_open_in_kicad(&ctx, &wanted, "test")
                .await
                .unwrap()
                .is_some());
            assert!(matches!(
                attempt_ipc_write(&ctx, &wanted, "test", |_| Ok(()))
                    .await
                    .unwrap(),
                BoardWrite::Ipc(())
            ));
        }

        #[tokio::test]
        async fn a_board_no_endpoint_holds_is_still_a_closed_board() {
            let sockets = Sockets::new();
            let wanted = board(sockets.dir.path(), "wanted.kicad_pcb");
            let other = board(sockets.dir.path(), "other.kicad_pcb");
            let configured = sockets.stale("api.sock");
            let _editor = spawn_kicad_holding_board_at(sockets.address("api-4242.sock"), &other);
            let ctx = ctx_talking_to(configured);

            assert!(matches!(
                live_board::observe(&ctx, &wanted, |_, _| Ok(()))
                    .await
                    .unwrap(),
                LiveBoard::NotOpen { .. }
            ));
            assert!(!ctx.board_session.was_observed_live(&wanted));
            assert!(refuse_if_board_open_in_kicad(&ctx, &wanted, "test")
                .await
                .unwrap()
                .is_none());
            assert_eq!(ctx.ipc_address_for_board(&wanted).await, ctx.ipc_address());
        }

        #[tokio::test]
        async fn a_configured_endpoint_outside_kicads_naming_is_asked_alone() {
            let sockets = Sockets::new();
            let wanted = board(sockets.dir.path(), "wanted.kicad_pcb");
            let configured = sockets.stale("custom.sock");
            let _editor = spawn_kicad_holding_board_at(sockets.address("api-4242.sock"), &wanted);
            let ctx = ctx_talking_to(configured.clone());

            assert_eq!(ctx.ipc_addresses(), [configured]);
            assert!(matches!(
                live_board::observe(&ctx, &wanted, |_, _| Ok(()))
                    .await
                    .unwrap(),
                LiveBoard::NeverReached(_)
            ));
        }
    }

    #[test]
    fn the_last_holder_is_asked_first_and_only_while_it_is_still_listed() {
        let dir = tempfile::tempdir().unwrap();
        let wanted = board(dir.path(), "wanted.kicad_pcb");
        let other = board(dir.path(), "other.kicad_pcb");
        let endpoints = BoardEndpoints::default();
        let listed = || vec!["a".to_string(), "b".to_string(), "c".to_string()];

        assert_eq!(
            endpoints.candidates_for(&wanted, listed()),
            Candidates {
                addresses: listed(),
                patient: 1
            }
        );

        endpoints.remember(&wanted, "c");
        assert_eq!(endpoints.last_holder(&wanted).as_deref(), Some("c"));
        assert_eq!(
            endpoints.candidates_for(&wanted, listed()),
            Candidates {
                addresses: vec!["c".to_string(), "a".to_string(), "b".to_string()],
                patient: 2
            }
        );
        // Another board is not reordered, and an endpoint that is gone is
        // not asked for.
        assert_eq!(
            endpoints.candidates_for(&other, listed()).addresses,
            listed()
        );
        assert_eq!(
            endpoints
                .candidates_for(&wanted, vec!["a".to_string(), "b".to_string()])
                .addresses,
            ["a", "b"]
        );
        // The configured endpoint holding the board needs no reordering.
        endpoints.remember(&wanted, "a");
        assert_eq!(
            endpoints.candidates_for(&wanted, listed()),
            Candidates {
                addresses: listed(),
                patient: 1
            }
        );
    }
}
