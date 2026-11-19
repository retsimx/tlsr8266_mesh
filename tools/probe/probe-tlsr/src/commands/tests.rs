//! Tests for the shared request helpers in [`super`].
//!
//! `sws_activate` is the one place that knows what the probe really answers,
//! so the tests pin both accepted replies and the fault for anything else.

use probe_proto::{Request, Response};

use super::sws_activate;
use crate::commands::test_support::data;
use crate::transport::MockTransport;
use crate::CliError;

/// Every target-touching command starts here, and both accepted replies come
/// from real firmware: `probe-pico` sends `ChipId`, older/other peers `Ack`.
#[test]
fn sws_activate_accepts_ack_and_chip_id() {
    for reply in [
        Response::Ack,
        Response::ChipId {
            rev: 127,
            chip_id: 21285,
        },
    ] {
        let mut mock = MockTransport::new([reply.clone()]);

        sws_activate(&mut mock)
            .unwrap_or_else(|error| panic!("{reply:?} must be accepted: {error:?}"));

        assert_eq!(
            mock.requests,
            [Request::SwsActivate { div: 3 }],
            "the activation request is unchanged"
        );
    }
}

/// A reply that is neither form means the bus is not up: a device fault.
#[test]
fn sws_activate_rejects_any_other_reply() {
    let mut mock = MockTransport::new([data([1, 2, 3])]);

    let err = sws_activate(&mut mock).expect_err("a data reply is not an activation");

    let CliError::Device(message) = err else {
        panic!("expected Device, got {err:?}");
    };
    assert!(
        message.contains("expected Ack or ChipId response"),
        "{message}"
    );
    assert!(message.contains("Data"), "{message}");
}
