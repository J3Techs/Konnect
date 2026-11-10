//! Streamable HTTP transport (MCP spec 2025-03-26 / 2025-06-18).
//!
//! A single MCP endpoint at `/mcp` supports:
//!   POST /mcp    — client sends one JSON-RPC message.
//!                  Requests get an `application/json` response body;
//!                  notifications/responses get `202 Accepted` with no body.
//!   GET  /mcp    — opens an SSE stream for server-initiated messages
//!                  (e.g. `notifications/tools/list_changed`).
//!   DELETE /mcp  — 405 (this server is stateless; no session termination).
//!
//! `GET /mcp/sse` is kept as an alias for older configs. `GET /health`
//! returns "ok" for probes.
//!
//! Security (per the MCP spec's Streamable HTTP requirements): the server
//! binds to 127.0.0.1 by default, and every request's `Origin` header — when
//! present — must be a localhost origin, otherwise 403. This prevents DNS
//! rebinding attacks from remote websites against the local server.
//!
//! Stopping. The standalone binary serves through [`run_http_until_stopped`]:
//! on the platform's stop signals the listener closes, requests already being
//! handled get a bounded grace period to finish, and a second signal ends the
//! wait at once. The embedded plugin serves through [`run_http`], which
//! installs no signal handler: the signals there belong to the host, KiCad.

use anyhow::Result;
use axum::{
    extract::State,
    http::{HeaderMap, StatusCode},
    middleware::{self, Next},
    response::{
        sse::{Event, Sse},
        IntoResponse, Response,
    },
    routing::get,
    Json, Router,
};
use konnect_core::mcp::handler::McpHandler;
use serde_json::Value;
use std::convert::Infallible;
use std::future::IntoFuture;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::{mpsc, oneshot, watch};
use tokio_stream::wrappers::ReceiverStream;
use tracing::{info, warn};

#[derive(Clone)]
struct AppState {
    handler: Arc<McpHandler>,
    /// True once the server has been asked to stop.
    stopping: watch::Receiver<bool>,
}

/// How a stopped HTTP transport left the requests it was handling.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HttpStop {
    /// Every request was answered and every connection closed.
    Drained,
    /// The grace period ran out, or a second signal arrived, with this many
    /// requests still being handled. They get no response.
    Abandoned { in_flight: usize },
}

/// Run the MCP server over Streamable HTTP until the process ends. No signal
/// handling: this is the entry point for a server embedded in another
/// process.
#[allow(dead_code)] // used by the plugin library, not by the binary
pub async fn run_http(handler: McpHandler, addr: &str) -> Result<()> {
    // Kept alive for the whole call: this server is never asked to stop.
    let (_stopping_tx, stopping) = watch::channel(false);
    let listener = tokio::net::TcpListener::bind(addr).await?;
    info!("Streamable HTTP transport listening on http://{}/mcp", addr);
    axum::serve(listener, router(handler, stopping)).await?;
    Ok(())
}

/// Run the MCP server over Streamable HTTP until a stop signal: SIGINT
/// (Ctrl+C) or SIGTERM on Unix; Ctrl+C, Ctrl+Break, a closed console, logoff
/// or system shutdown on Windows.
///
/// On the first signal the listener closes, and idle connections with it;
/// requests already being handled have `grace` to finish and be answered. A
/// second signal ends the wait at once.
#[allow(dead_code)] // used by the binary, not by the plugin library
pub async fn run_http_until_stopped(
    handler: McpHandler,
    addr: &str,
    grace: Duration,
) -> Result<HttpStop> {
    let (stopping_tx, stopping) = watch::channel(false);
    let listener = tokio::net::TcpListener::bind(addr).await?;
    info!("Streamable HTTP transport listening on http://{}/mcp", addr);
    serve_until_stopped(
        listener,
        router(handler, stopping),
        stopping_tx,
        stop_signals(),
        grace,
    )
    .await
}

fn router(handler: McpHandler, stopping: watch::Receiver<bool>) -> Router {
    let state = AppState {
        handler: Arc::new(handler),
        stopping,
    };

    Router::new()
        .route(
            "/mcp",
            axum::routing::post(handle_post)
                .get(handle_sse)
                .delete(|| async { StatusCode::METHOD_NOT_ALLOWED }),
        )
        .route("/mcp/sse", get(handle_sse)) // legacy alias
        .route("/health", get(handle_health))
        .layer(middleware::from_fn(validate_origin))
        .with_state(state)
}

/// Serve `app` until the first name arrives on `stop`, then drain.
///
/// `stopping` is set at that moment, so that a stream which never ends by
/// itself (SSE) can end; see [`until_stopping`]. Split from
/// [`run_http_until_stopped`] so that a test can supply the stop.
async fn serve_until_stopped(
    listener: tokio::net::TcpListener,
    app: Router,
    stopping: watch::Sender<bool>,
    mut stop: mpsc::Receiver<&'static str>,
    grace: Duration,
) -> Result<HttpStop> {
    let in_flight = Arc::new(AtomicUsize::new(0));
    let counter = in_flight.clone();
    let app = app.layer(middleware::from_fn(
        move |request: axum::extract::Request, next: Next| {
            let counted = InFlight::enter(&counter);
            async move {
                let response = next.run(request).await;
                drop(counted);
                response
            }
        },
    ));

    let (close_listener, listener_closed) = oneshot::channel::<()>();
    let serve = axum::serve(listener, app)
        .with_graceful_shutdown(async move {
            // An error means the sender went away without a stop: keep serving.
            if listener_closed.await.is_err() {
                std::future::pending::<()>().await;
            }
        })
        .into_future();
    tokio::pin!(serve);

    let signal = tokio::select! {
        result = &mut serve => {
            result?;
            return Ok(HttpStop::Drained);
        }
        signal = stop.recv() => signal,
    };
    let Some(signal) = signal else {
        // No signal could be registered: serve as before this existed.
        serve.await?;
        return Ok(HttpStop::Drained);
    };

    stopping.send_replace(true);
    let _ = close_listener.send(());
    info!(
        "{signal} received: no longer accepting requests, waiting up to {} s for {} in flight \
         (a second signal stops at once)",
        grace.as_secs(),
        in_flight.load(Ordering::SeqCst),
    );

    tokio::select! {
        result = &mut serve => {
            result?;
            info!("HTTP transport stopped: no request was left unanswered");
            Ok(HttpStop::Drained)
        }
        _ = tokio::time::sleep(grace) => {
            let in_flight = in_flight.load(Ordering::SeqCst);
            warn!(
                "HTTP transport stopped: {in_flight} request(s) still in flight after {} s \
                 were abandoned",
                grace.as_secs(),
            );
            Ok(HttpStop::Abandoned { in_flight })
        }
        again = stop.recv() => {
            let in_flight = in_flight.load(Ordering::SeqCst);
            warn!(
                "HTTP transport stopped at once on a second signal ({}): \
                 {in_flight} request(s) in flight were abandoned",
                again.unwrap_or("signal source closed"),
            );
            Ok(HttpStop::Abandoned { in_flight })
        }
    }
}

/// Counts a request from its arrival until its handler has produced the
/// response.
struct InFlight(Arc<AtomicUsize>);

impl InFlight {
    fn enter(counter: &Arc<AtomicUsize>) -> Self {
        counter.fetch_add(1, Ordering::SeqCst);
        Self(counter.clone())
    }
}

impl Drop for InFlight {
    fn drop(&mut self) {
        self.0.fetch_sub(1, Ordering::SeqCst);
    }
}

/// The platform's stop requests, by name, in the order they arrive.
fn stop_signals() -> mpsc::Receiver<&'static str> {
    let (tx, rx) = mpsc::channel(4);

    #[cfg(unix)]
    {
        use tokio::signal::unix::{signal, SignalKind};
        for (name, kind) in [
            ("SIGINT", SignalKind::interrupt()),
            ("SIGTERM", SignalKind::terminate()),
        ] {
            match signal(kind) {
                Ok(mut stream) => {
                    let tx = tx.clone();
                    tokio::spawn(async move {
                        while stream.recv().await.is_some() {
                            if tx.send(name).await.is_err() {
                                break;
                            }
                        }
                    });
                }
                Err(error) => warn!("{name} will end the server abruptly: {error}"),
            }
        }
    }

    #[cfg(windows)]
    {
        use tokio::signal::windows;
        macro_rules! forward {
            ($name:literal, $register:path) => {
                match $register() {
                    Ok(mut stream) => {
                        let tx = tx.clone();
                        tokio::spawn(async move {
                            while stream.recv().await.is_some() {
                                if tx.send($name).await.is_err() {
                                    break;
                                }
                            }
                        });
                    }
                    Err(error) => warn!("{} will end the server abruptly: {error}", $name),
                }
            };
        }
        forward!("Ctrl+C", windows::ctrl_c);
        forward!("Ctrl+Break", windows::ctrl_break);
        // Windows ends the process a few seconds after these three whatever
        // it is doing, so the system cuts a longer grace period short.
        forward!("console close", windows::ctrl_close);
        forward!("logoff", windows::ctrl_logoff);
        forward!("system shutdown", windows::ctrl_shutdown);
    }

    rx
}

/// `events`, ended when the server is asked to stop. An SSE stream never ends
/// by itself, and a draining server waits for its connections to close.
fn until_stopping<T: Send + 'static>(
    events: impl tokio_stream::Stream<Item = T> + Send + 'static,
    stopping: watch::Receiver<bool>,
) -> impl tokio_stream::Stream<Item = T> + Send + 'static {
    use tokio_stream::StreamExt;
    // One `None`, produced when the server is asked to stop.
    let stop = tokio_stream::once(()).then(move |()| {
        let mut stopping = stopping.clone();
        async move {
            // A dropped sender means nothing will ever ask: never end.
            if stopping.wait_for(|stopping| *stopping).await.is_err() {
                std::future::pending::<()>().await;
            }
            None
        }
    });
    events
        .map(Some)
        .merge(stop)
        .take_while(Option::is_some)
        .filter_map(|event| event)
}

/// Reject any request whose `Origin` header is present and not a localhost
/// origin (MCP spec: servers MUST validate Origin to prevent DNS rebinding).
/// Requests without an Origin header (curl, native MCP clients) pass through.
async fn validate_origin(
    headers: HeaderMap,
    request: axum::extract::Request,
    next: Next,
) -> Response {
    if let Some(origin) = headers.get("origin").and_then(|v| v.to_str().ok()) {
        let allowed = origin
            .strip_prefix("http://")
            .or_else(|| origin.strip_prefix("https://"))
            .map(|rest| {
                let host = rest.split(':').next().unwrap_or("");
                host == "localhost" || host == "127.0.0.1" || host == "[::1]"
            })
            .unwrap_or(false);
        if !allowed {
            return (StatusCode::FORBIDDEN, "Origin not allowed").into_response();
        }
    }
    next.run(request).await
}

async fn handle_post(State(state): State<AppState>, Json(body): Json<Value>) -> Response {
    match state.handler.handle_message(body).await {
        // JSON-RPC request → single JSON response object.
        Some(resp) => Json(resp).into_response(),
        // JSON-RPC notification or response → 202 Accepted, no body.
        None => StatusCode::ACCEPTED.into_response(),
    }
}

/// GET on the MCP endpoint: open an SSE stream for server-initiated messages.
async fn handle_sse(
    State(state): State<AppState>,
) -> Sse<impl tokio_stream::Stream<Item = Result<Event, Infallible>>> {
    let (tx, rx) = tokio::sync::mpsc::channel::<Event>(32);

    // Register SSE sender with handler so it can push notifications
    // (e.g. tools/list_changed after load_toolset / unload_toolset).
    state.handler.register_sse_sender(tx).await;

    use tokio_stream::StreamExt;
    let stream = until_stopping(ReceiverStream::new(rx), state.stopping.clone()).map(Ok);
    Sse::new(stream)
}

async fn handle_health() -> &'static str {
    "ok"
}

#[cfg(test)]
mod shutdown_tests {
    use super::*;
    use std::net::SocketAddr;
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    use tokio::net::TcpStream;
    use tokio::task::JoinHandle;

    struct Served {
        addr: SocketAddr,
        stop: mpsc::Sender<&'static str>,
        server: JoinHandle<Result<HttpStop>>,
    }

    /// A server with one request that takes `work` to answer and one stream
    /// that, like the SSE endpoint, has nothing to send and never ends.
    async fn serve(work: Duration, grace: Duration) -> Served {
        let (stopping_tx, stopping) = watch::channel(false);
        let app = Router::new()
            .route(
                "/work",
                get(move || async move {
                    tokio::time::sleep(work).await;
                    "done"
                }),
            )
            .route(
                "/events",
                get(move || {
                    let stopping = stopping.clone();
                    async move {
                        let (tx, rx) = mpsc::channel::<Event>(1);
                        // Held by the stream's owner, as the handler holds
                        // the real one: the stream cannot end by itself.
                        let events = until_stopping(ReceiverStream::new(rx), stopping);
                        use tokio_stream::StreamExt;
                        Sse::new(events.map(move |event| {
                            let _held = &tx;
                            Ok::<_, Infallible>(event)
                        }))
                    }
                }),
            );
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let (stop, stop_rx) = mpsc::channel(4);
        let server = tokio::spawn(serve_until_stopped(
            listener,
            app,
            stopping_tx,
            stop_rx,
            grace,
        ));
        Served { addr, stop, server }
    }

    /// Send a request and return the connection once the server has it.
    async fn begin(addr: SocketAddr, path: &str) -> TcpStream {
        let mut stream = TcpStream::connect(addr).await.unwrap();
        stream
            .write_all(format!("GET {path} HTTP/1.1\r\nHost: {addr}\r\n\r\n").as_bytes())
            .await
            .unwrap();
        stream
    }

    async fn read_to_close(mut stream: TcpStream) -> String {
        let mut raw = String::new();
        let _ = stream.read_to_string(&mut raw).await;
        raw
    }

    /// Give the server's tasks a moment to take a request or a signal.
    async fn settle() {
        tokio::time::sleep(Duration::from_millis(150)).await;
    }

    #[tokio::test]
    async fn a_request_in_flight_at_the_stop_signal_is_answered_before_the_server_stops() {
        let served = serve(Duration::from_millis(600), Duration::from_secs(20)).await;
        let request = begin(served.addr, "/work").await;
        settle().await;

        served.stop.send("test stop").await.unwrap();
        let outcome = served.server.await.unwrap().unwrap();

        assert_eq!(outcome, HttpStop::Drained);
        let response = read_to_close(request).await;
        assert!(response.starts_with("HTTP/1.1 200"), "{response}");
        assert!(response.ends_with("done"), "{response}");
    }

    #[tokio::test]
    async fn a_stopping_server_accepts_no_new_connection() {
        let served = serve(Duration::from_secs(2), Duration::from_secs(20)).await;
        let _request = begin(served.addr, "/work").await;
        settle().await;

        served.stop.send("test stop").await.unwrap();
        settle().await;

        assert!(
            !served.server.is_finished(),
            "the server should still be waiting for the request in flight"
        );
        // Refused, or accepted by nobody and closed: either way no response.
        let answered = match TcpStream::connect(served.addr).await {
            Err(_) => false,
            Ok(mut late) => {
                let _ = late
                    .write_all(b"GET /work HTTP/1.1\r\nHost: x\r\n\r\n")
                    .await;
                read_to_close(late).await.starts_with("HTTP/1.1")
            }
        };
        assert!(!answered, "a request sent after the stop was served");
        assert_eq!(served.server.await.unwrap().unwrap(), HttpStop::Drained);
    }

    #[tokio::test]
    async fn the_grace_period_bounds_the_wait_and_reports_what_was_abandoned() {
        let served = serve(Duration::from_secs(30), Duration::from_millis(300)).await;
        let _request = begin(served.addr, "/work").await;
        settle().await;

        let started = std::time::Instant::now();
        served.stop.send("test stop").await.unwrap();
        let outcome = served.server.await.unwrap().unwrap();

        assert_eq!(outcome, HttpStop::Abandoned { in_flight: 1 });
        assert!(
            started.elapsed() < Duration::from_secs(10),
            "the stop waited for the request instead of the grace period"
        );
    }

    #[tokio::test]
    async fn a_second_signal_ends_the_wait_at_once() {
        let served = serve(Duration::from_secs(30), Duration::from_secs(30)).await;
        let _request = begin(served.addr, "/work").await;
        settle().await;

        let started = std::time::Instant::now();
        served.stop.send("first").await.unwrap();
        settle().await;
        assert!(!served.server.is_finished());
        served.stop.send("second").await.unwrap();
        let outcome = served.server.await.unwrap().unwrap();

        assert_eq!(outcome, HttpStop::Abandoned { in_flight: 1 });
        assert!(started.elapsed() < Duration::from_secs(10));
    }

    #[tokio::test]
    async fn an_open_event_stream_does_not_hold_a_stopping_server() {
        let served = serve(Duration::ZERO, Duration::from_secs(30)).await;
        let events = begin(served.addr, "/events").await;
        settle().await;

        let started = std::time::Instant::now();
        served.stop.send("test stop").await.unwrap();
        let outcome = served.server.await.unwrap().unwrap();

        assert_eq!(outcome, HttpStop::Drained);
        assert!(
            started.elapsed() < Duration::from_secs(10),
            "the open stream kept the server waiting for the grace period"
        );
        let response = read_to_close(events).await;
        assert!(response.starts_with("HTTP/1.1 200"), "{response}");
    }

    #[tokio::test]
    async fn an_idle_server_stops_at_once() {
        let served = serve(Duration::ZERO, Duration::from_secs(30)).await;
        let started = std::time::Instant::now();
        served.stop.send("test stop").await.unwrap();
        assert_eq!(served.server.await.unwrap().unwrap(), HttpStop::Drained);
        assert!(started.elapsed() < Duration::from_secs(10));
    }

    #[tokio::test]
    async fn a_server_with_no_signal_source_keeps_serving() {
        let served = serve(Duration::ZERO, Duration::from_secs(30)).await;
        drop(served.stop);
        settle().await;
        assert!(!served.server.is_finished());
        let mut request = begin(served.addr, "/work").await;
        let mut head = [0u8; 12];
        request.read_exact(&mut head).await.unwrap();
        assert_eq!(&head, b"HTTP/1.1 200");
        served.server.abort();
    }
}
