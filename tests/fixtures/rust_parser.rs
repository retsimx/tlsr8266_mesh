#[cfg(test)]
use std::collections::{HashMap, HashSet};

#[cfg(test)]
#[derive(Debug)]
pub struct Parser {
    values: HashMap<String, String>,
    consumed: HashSet<String>,
}

#[cfg(test)]
#[allow(dead_code)]
impl Parser {
    pub fn parse(text: &str) -> Result<Self, String> {
        let mut values = HashMap::new();

        for (line_index, raw_line) in text.lines().enumerate() {
            let line = raw_line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }

            let mut parts = line.splitn(2, '=');
            let key = parts.next().unwrap_or("").trim();
            let value = parts.next().map(str::trim);

            if key.is_empty() {
                return Err(format!("fixture line {} has no key", line_index + 1));
            }
            if key.split_whitespace().count() != 1 || key.chars().any(char::is_whitespace) {
                return Err(format!("fixture line {} has malformed key: {key}", line_index + 1));
            }
            let Some(value) = value else {
                return Err(format!("fixture line {} is missing '='", line_index + 1));
            };

            if values.contains_key(key) {
                return Err(format!("duplicate fixture key: {key}"));
            }
            values.insert(key.to_owned(), value.to_owned());
        }

        Ok(Self { values, consumed: HashSet::new() })
    }

    fn get(&mut self, key: &str) -> Result<&str, String> {
        self.consumed.insert(key.to_owned());
        self.values
            .get(key)
            .map(String::as_str)
            .ok_or_else(|| format!("missing fixture key: {key}"))
    }

    pub fn str(&mut self, key: &str) -> Result<&str, String> {
        self.get(key)
    }

    pub fn u8(&mut self, key: &str) -> Result<u8, String> {
        let value = self.get(key)?;
        if key.contains(".opcode.") || key.contains("uart.op.") {
            u8::from_str_radix(value.strip_prefix("0x").unwrap_or(value), 16)
                .map_err(|_| format!("fixture key {key} has invalid u8 hex value: {value}"))
        } else {
            parse_numeric(value, key, 8)
        }
    }

    pub fn u16(&mut self, key: &str) -> Result<u16, String> {
        let value = self.get(key)?;
        if key.contains("vendor_id")
            || key.contains("brightness")
            || key.contains("golden_crc")
        {
            u16::from_str_radix(value.strip_prefix("0x").unwrap_or(value), 16)
                .map_err(|_| format!("fixture key {key} has invalid u16 hex value: {value}"))
        } else {
            parse_numeric(value, key, 16)
        }
    }

    pub fn u32(&mut self, key: &str) -> Result<u32, String> {
        parse_numeric(self.get(key)?, key, 32)
    }

    pub fn bytes(&mut self, key: &str) -> Result<std::vec::Vec<u8>, String> {
        let value = self.get(key)?;
        if value.eq_ignore_ascii_case("none") || value == "-" {
            return Ok(std::vec::Vec::new());
        }

        let mut out = std::vec::Vec::new();
        if value.is_empty() {
            return Ok(out);
        }

        for token in value.split(|ch: char| ch.is_ascii_whitespace() || ch == ',') {
            if token.is_empty() {
                continue;
            }
            let token = token.strip_prefix("0x").unwrap_or(token);
            if token.len() != 2 {
                return Err(format!("fixture bytes key {key} has invalid token: {token}"));
            }
            let byte = u8::from_str_radix(token, 16)
                .map_err(|_| format!("fixture bytes key {key} has invalid hex: {token}"))?;
            out.push(byte);
        }
        if out.is_empty() {
            return Err(format!("fixture bytes key {key} has no bytes"));
        }
        Ok(out)
    }

    pub fn bool(&mut self, key: &str) -> Result<bool, String> {
        match self.get(key)? {
            "true" => Ok(true),
            "false" => Ok(false),
            value => Err(format!("fixture bool key {key} has invalid value: {value}")),
        }
    }

    pub fn consumed_keys(&self) -> &HashSet<String> {
        &self.consumed
    }

    pub fn key_count(&self) -> usize {
        self.values.len()
    }

    pub fn consume_all(&mut self) {
        let keys: std::vec::Vec<String> = self.values.keys().cloned().collect();
        for key in keys {
            self.consumed.insert(key);
        }
    }

    pub fn unconsumed_keys(&self) -> std::vec::Vec<&str> {
        let mut keys: std::vec::Vec<&str> = self
            .values
            .keys()
            .filter(|key| !self.consumed.contains(*key))
            .map(String::as_str)
            .collect();
        keys.sort_unstable();
        keys
    }

    pub fn contains(&mut self, key: &str) -> bool {
        self.consumed.insert(key.to_owned());
        self.values.contains_key(key)
    }
}

#[cfg(test)]
#[allow(dead_code)]
pub fn parse_fixture(text: &str) -> Result<Parser, String> {
    Parser::parse(text)
}

#[cfg(test)]
fn parse_numeric<T>(value: &str, key: &str, bits: u32) -> Result<T, String>
where
    T: std::str::FromStr,
{
    let hex = value.strip_prefix("0x").unwrap_or(value);
    let parsed = if value.parse::<T>().is_ok() {
        value.parse::<T>().ok()
    } else {
        match bits {
            8 => u8::from_str_radix(hex, 16)
                .ok()
                .and_then(|v| <T>::from_str(&v.to_string()).ok()),
            16 => u16::from_str_radix(hex, 16)
                .ok()
                .and_then(|v| <T>::from_str(&v.to_string()).ok()),
            32 => u32::from_str_radix(hex, 16)
                .ok()
                .and_then(|v| <T>::from_str(&v.to_string()).ok()),
            _ => None,
        }
    };

    parsed.ok_or_else(|| format!("fixture key {key} has invalid u{bits} value: {value}"))
}
