//! Failure rendering: one JSON object on `--json`, `error: …` otherwise.
//!
//! Both forms go to stderr (stdout is left empty on failure), so scripts can
//! branch on the exit code and read the message from stderr.

use std::io::Write;

use serde::Serialize;

use crate::CliError;

/// Writes the failure report to `err` (stderr in `main`); stdout is untouched.
pub fn report_error(error: &CliError, json: bool, err: &mut dyn Write) {
    let _ = writeln!(err, "{}", error_text(error, json));
}

/// Renders the one-line failure report, JSON or human.
pub fn error_text(error: &CliError, json: bool) -> String {
    if !json {
        return format!("error: {error}");
    }
    error_json(error).unwrap_or_else(|_| format!("error: {error}"))
}

/// Error object shape for `--json`; always goes to stderr, never stdout.
fn error_json(err: &CliError) -> Result<String, serde_json::Error> {
    let message = err.to_string();
    serde_json::to_string(&ErrorReport {
        command: "error",
        exit: err.exit_code(),
        error: err.kind(),
        proto_error: err.proto_error(),
        message: &message,
    })
}

/// Serialized with the fields in this order, matching the documented shape.
#[derive(Serialize)]
struct ErrorReport<'a> {
    command: &'static str,
    exit: u8,
    error: &'static str,
    proto_error: Option<u8>,
    message: &'a str,
}
