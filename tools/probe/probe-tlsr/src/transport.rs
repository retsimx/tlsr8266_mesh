//! Probe link: request/response exchange over the USB CDC serial port.
//!
//! [`SerialTransport`] is the real link; [`MockTransport`] is a scripted
//! in-process peer used by the command tests.

use std::io::{ErrorKind, Write};
use std::path::Path;
use std::time::{Duration, Instant};

use probe_proto::{
    decode_frame, encode_frame, Direction, Message, Request, Response, MAX_WIRE_FRAME,
};
use serialport::{ClearBuffer, SerialPort};

use crate::CliError;

/// USB vendor ID of the RP2040 probe.
const PROBE_USB_VID: u16 = 0x2e8a;

/// Device node used when enumeration finds no probe.
const FALLBACK_PORT: &str = "/dev/ttyACM0";

/// USB CDC ignores the line rate, but `serialport` requires one.
const BAUD_RATE: u32 = 115_200;

/// One request/response exchange with the probe.
pub trait ProbeTransport {
    /// Sends `req` and returns the probe's reply.
    fn exchange(&mut self, req: &Request) -> Result<Response, CliError>;
}

/// USB CDC link to a probe, framing every request with `probe-proto`.
pub struct SerialTransport {
    port: Box<dyn SerialPort>,
    seq: u8,
    timeout: Duration,
}

impl SerialTransport {
    /// Opens `path`, or auto-detects the probe when `path` is `None`.
    pub fn open(path: Option<&Path>, timeout_ms: u64) -> Result<Self, CliError> {
        let name = match path {
            Some(path) => path.to_string_lossy().into_owned(),
            None => detect_port()?,
        };
        let timeout = Duration::from_millis(timeout_ms);
        let port = serialport::new(&name, BAUD_RATE)
            .timeout(timeout)
            .open()
            .map_err(|err| CliError::Transport(format!("cannot open {name}: {err}")))?;
        Ok(Self {
            port,
            seq: 0,
            timeout,
        })
    }
}

impl ProbeTransport for SerialTransport {
    fn exchange(&mut self, req: &Request) -> Result<Response, CliError> {
        let seq = self.seq;
        self.seq = self.seq.wrapping_add(1);

        // A late reply to an earlier command would otherwise be mistaken for
        // this command's response.
        self.port
            .clear(ClearBuffer::Input)
            .map_err(|err| CliError::Transport(format!("cannot flush input: {err}")))?;

        let mut wire = [0u8; MAX_WIRE_FRAME];
        let len = encode_frame(seq, &Message::Request(req.clone()), &mut wire)
            .map_err(CliError::Protocol)?;
        self.port
            .write_all(&wire[..len])
            .map_err(|err| CliError::Transport(format!("write failed: {err}")))?;
        self.port
            .flush()
            .map_err(|err| CliError::Transport(format!("flush failed: {err}")))?;

        let deadline = Instant::now() + self.timeout;
        let mut frame = [0u8; MAX_WIRE_FRAME];
        let got = read_frame(&mut *self.port, &mut frame, deadline)?;
        decode_response(&frame[..got], seq)
    }
}

/// Returns the USB vendor ID of a port, if it is a USB serial port.
fn usb_vid(port: &serialport::SerialPortInfo) -> Option<u16> {
    match &port.port_type {
        serialport::SerialPortType::UsbPort(info) => Some(info.vid),
        _ => None,
    }
}

/// Returns the probe's device node: the first port with the probe's vendor ID,
/// else [`FALLBACK_PORT`] when it exists.
fn detect_port() -> Result<String, CliError> {
    if let Ok(ports) = serialport::available_ports() {
        if let Some(port) = ports
            .into_iter()
            .find(|port| usb_vid(port) == Some(PROBE_USB_VID))
        {
            return Ok(port.port_name);
        }
    }
    if Path::new(FALLBACK_PORT).exists() {
        return Ok(FALLBACK_PORT.to_owned());
    }
    Err(CliError::Transport(format!(
        "no probe found (no USB VID {PROBE_USB_VID:#06x} port and {FALLBACK_PORT} missing); pass --port <path>"
    )))
}

/// Reads one delimiter-terminated frame into `out`, returning its length.
///
/// Bytes following the delimiter are dropped: the link is strictly
/// request/response, so a second frame means the peer ignored the contract.
fn read_frame(
    port: &mut dyn SerialPort,
    out: &mut [u8],
    deadline: Instant,
) -> Result<usize, CliError> {
    let mut len = 0;
    let mut chunk = [0u8; 64];
    loop {
        if Instant::now() >= deadline {
            return Err(CliError::Transport(
                "timed out waiting for a probe response".to_owned(),
            ));
        }
        let read = match port.read(&mut chunk) {
            Ok(0) => {
                return Err(CliError::Transport(
                    "probe closed the serial port".to_owned(),
                ))
            }
            Ok(read) => read,
            Err(err) if err.kind() == ErrorKind::TimedOut => {
                return Err(CliError::Transport(
                    "timed out waiting for a probe response".to_owned(),
                ))
            }
            Err(err) => return Err(CliError::Transport(format!("read failed: {err}"))),
        };
        for &byte in &chunk[..read] {
            if len == out.len() {
                return Err(CliError::Transport(
                    "probe frame is longer than the wire limit".to_owned(),
                ));
            }
            out[len] = byte;
            len += 1;
            if byte == 0 {
                return Ok(len);
            }
        }
    }
}

/// Decodes a response frame, requiring the echoed sequence byte to match.
fn decode_response(wire: &[u8], expected_seq: u8) -> Result<Response, CliError> {
    let (seq, message) = decode_frame(Direction::Response, wire).map_err(CliError::Protocol)?;
    if seq != expected_seq {
        return Err(CliError::Transport(format!(
            "response seq {seq} does not match request seq {expected_seq}"
        )));
    }
    match message {
        Message::Response(response) => map_response(response),
        Message::Request(_) => Err(CliError::Device(
            "probe replied with a request frame".to_owned(),
        )),
    }
}

/// Surfaces a probe-reported failure as a protocol error, preserving its code.
fn map_response(response: Response) -> Result<Response, CliError> {
    match response {
        Response::Error(err) => Err(CliError::Protocol(err)),
        response => Ok(response),
    }
}

/// Scripted in-process transport: replays responses and records requests.
#[cfg(test)]
pub struct MockTransport {
    responses: std::collections::VecDeque<Response>,
    /// Every request issued by the code under test, in order.
    pub requests: Vec<Request>,
}

#[cfg(test)]
impl MockTransport {
    /// Replays `responses` in order; running out is a device error.
    pub fn new(responses: impl IntoIterator<Item = Response>) -> Self {
        Self {
            responses: responses.into_iter().collect(),
            requests: Vec::new(),
        }
    }

    /// Replays exactly one response.
    pub fn from_response(response: Response) -> Self {
        Self::new([response])
    }
}

#[cfg(test)]
impl ProbeTransport for MockTransport {
    fn exchange(&mut self, req: &Request) -> Result<Response, CliError> {
        self.requests.push(req.clone());
        let response = self.responses.pop_front().ok_or_else(|| {
            CliError::Device("mock transport has no scripted response left".to_owned())
        })?;
        map_response(response)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use probe_proto::ProtoError;

    fn response_frame(seq: u8, response: Response) -> Vec<u8> {
        let mut wire = [0u8; MAX_WIRE_FRAME];
        let len =
            encode_frame(seq, &Message::Response(response), &mut wire).expect("response encodes");
        wire[..len].to_vec()
    }

    #[test]
    fn mock_records_requests_in_order() {
        let mut mock = MockTransport::new([Response::Ack, Response::Byte(0x42)]);

        assert_eq!(mock.exchange(&Request::FlashJedecId), Ok(Response::Ack));
        assert_eq!(
            mock.exchange(&Request::ReadAnalog { addr: 0x0c }),
            Ok(Response::Byte(0x42))
        );

        let expected = [Request::FlashJedecId, Request::ReadAnalog { addr: 0x0c }];
        assert_eq!(mock.requests, expected);
    }

    #[test]
    fn encoded_response_round_trips_through_probe_proto() {
        let response = Response::JedecId([0xef, 0x40, 0x18]);
        let frame = response_frame(7, response.clone());

        assert_eq!(decode_response(&frame, 7), Ok(response));
    }

    #[test]
    fn mismatched_sequence_byte_is_rejected() {
        let frame = response_frame(7, Response::Ack);
        let err = decode_response(&frame, 8).expect_err("mismatched seq must fail");

        assert!(matches!(err, CliError::Transport(_)), "{err:?}");
    }

    #[test]
    fn probe_error_response_maps_to_protocol_error() {
        let frame = response_frame(3, Response::Error(ProtoError::SwsTimeout));
        assert_eq!(
            decode_response(&frame, 3),
            Err(CliError::Protocol(ProtoError::SwsTimeout))
        );

        let mut mock = MockTransport::from_response(Response::Error(ProtoError::FlashBusyTimeout));
        assert_eq!(
            mock.exchange(&Request::FlashRead { addr: 0, len: 4 }),
            Err(CliError::Protocol(ProtoError::FlashBusyTimeout))
        );
    }

    #[test]
    fn opening_a_missing_port_reports_a_transport_error() {
        let result = SerialTransport::open(Some(Path::new("/dev/probe-tlsr-does-not-exist")), 10);
        let err = result.err().expect("opening a missing device must fail");

        assert!(matches!(err, CliError::Transport(_)), "{err:?}");
    }
}
