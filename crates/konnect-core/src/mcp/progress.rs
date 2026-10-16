//! MCP progress notifications for long-running tool calls.
//!
//! A client that wants progress puts `_meta.progressToken` on `tools/call`.
//! The handler runs the tool inside [`scope`] with a [`Reporter`] for that
//! token, and any code on the call's task can then call [`report`] to send
//! `notifications/progress`. Clients reset their idle timeout on progress, so
//! a long Freerouting job is not abandoned as silent; without a token,
//! [`report`] does nothing.

use axum::response::sse::Event;
use serde_json::{json, Value};
use std::future::Future;
use std::sync::Arc;
use tokio::sync::{mpsc, RwLock};

pub const PROGRESS: &str = "notifications/progress";

/// Where one call's progress goes: its token and the transports' sinks.
#[derive(Clone)]
pub struct Reporter {
    token: Value,
    sse: Arc<RwLock<Vec<mpsc::Sender<Event>>>>,
    lines: Arc<RwLock<Vec<mpsc::Sender<String>>>>,
}

impl Reporter {
    pub fn new(
        token: Value,
        sse: Arc<RwLock<Vec<mpsc::Sender<Event>>>>,
        lines: Arc<RwLock<Vec<mpsc::Sender<String>>>>,
    ) -> Self {
        Self { token, sse, lines }
    }

    fn notification(&self, progress: f64, total: Option<f64>, message: &str) -> String {
        let mut params = json!({ "progressToken": self.token, "progress": progress });
        if let Some(total) = total {
            params["total"] = json!(total);
        }
        if !message.is_empty() {
            params["message"] = json!(message);
        }
        json!({ "jsonrpc": "2.0", "method": PROGRESS, "params": params }).to_string()
    }

    async fn send(&self, progress: f64, total: Option<f64>, message: &str) {
        let json = self.notification(progress, total, message);
        {
            let event = Event::default().data(json.clone());
            let mut senders = self.sse.write().await;
            // Keep a sender whose channel is merely full; drop closed ones.
            senders.retain(|tx| tx.try_send(event.clone()).is_ok() || !tx.is_closed());
        }
        let mut lines = self.lines.write().await;
        lines.retain(|tx| !tx.is_closed());
        for tx in lines.iter() {
            // A full channel drops this update; the next one follows shortly.
            let _ = tx.try_send(json.clone());
        }
    }
}

tokio::task_local! {
    static CURRENT: Reporter;
}

/// Run `future` with `reporter` as the call's progress destination.
pub async fn scope<F: Future>(reporter: Option<Reporter>, future: F) -> F::Output {
    match reporter {
        Some(reporter) => CURRENT.scope(reporter, future).await,
        None => future.await,
    }
}

/// Send `notifications/progress` for the current call, if the client asked.
/// `progress` must increase from one report to the next.
pub async fn report(progress: f64, total: Option<f64>, message: &str) {
    if let Ok(reporter) = CURRENT.try_with(Reporter::clone) {
        reporter.send(progress, total, message).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn reports_reach_the_line_sink_only_inside_a_scope() {
        let (tx, mut rx) = mpsc::channel::<String>(4);
        let lines = Arc::new(RwLock::new(vec![tx]));
        let reporter = Reporter::new(json!("tok-1"), Arc::new(RwLock::new(Vec::new())), lines);

        report(1.0, None, "outside").await;
        assert!(rx.try_recv().is_err(), "no scope, no notification");

        scope(Some(reporter), async {
            report(2.0, Some(10.0), "routing").await;
        })
        .await;
        let sent: Value = serde_json::from_str(&rx.try_recv().unwrap()).unwrap();
        assert_eq!(sent["method"], PROGRESS);
        assert_eq!(sent["params"]["progressToken"], "tok-1");
        assert_eq!(sent["params"]["progress"], 2.0);
        assert_eq!(sent["params"]["total"], 10.0);
        assert_eq!(sent["params"]["message"], "routing");

        scope(None, async { report(3.0, None, "").await }).await;
        assert!(
            rx.try_recv().is_err(),
            "a call without a token reports nothing"
        );
    }
}
