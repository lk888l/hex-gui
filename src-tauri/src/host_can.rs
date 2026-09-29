//! Linux SocketCAN setup for the Lingkong workspace. All `ip` arguments are
//! fixed or validated; the password is passed only to sudo's stdin.

use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct HostCanStatus {
    pub iface: String,
    pub kind: String,
    pub up: bool,
    pub bitrate: Option<u32>,
    pub fd_enabled: bool,
}

fn validate_iface(iface: &str) -> Result<(), String> {
    if iface.is_empty()
        || iface.len() > 15
        || !iface.as_bytes()[0].is_ascii_alphanumeric()
        || !iface
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.'))
    {
        return Err("invalid CAN interface name".into());
    }
    Ok(())
}

#[cfg(target_os = "linux")]
pub fn inspect(iface: &str) -> Result<HostCanStatus, String> {
    use std::process::Command;

    validate_iface(iface)?;
    let output = Command::new("ip")
        .args(["-details", "-json", "link", "show", "dev", iface])
        .output()
        .map_err(|error| format!("cannot run ip: {error}"))?;
    if !output.status.success() {
        return Err(format!(
            "cannot inspect {iface}: {}",
            String::from_utf8_lossy(&output.stderr).trim()
        ));
    }
    let value: serde_json::Value = serde_json::from_slice(&output.stdout)
        .map_err(|error| format!("invalid ip JSON output: {error}"))?;
    let link = value
        .as_array()
        .and_then(|array| array.first())
        .ok_or_else(|| format!("CAN interface {iface} was not found"))?;
    let kind = link["linkinfo"]["info_kind"]
        .as_str()
        .unwrap_or("unknown")
        .to_string();
    if kind != "can" {
        return Err(format!(
            "{iface} is a {kind} interface, not a physical CAN interface"
        ));
    }
    let up = link["flags"]
        .as_array()
        .is_some_and(|flags| flags.iter().any(|flag| flag.as_str() == Some("UP")));
    let bitrate = link["linkinfo"]["info_data"]["bittiming"]["bitrate"]
        .as_u64()
        .and_then(|value| u32::try_from(value).ok());
    let fd_enabled = link["mtu"].as_u64() == Some(72)
        || link["linkinfo"]["info_data"]["ctrlmode"]
            .as_array()
            .is_some_and(|modes| modes.iter().any(|mode| mode.as_str() == Some("FD")));
    Ok(HostCanStatus {
        iface: iface.to_owned(),
        kind,
        up,
        bitrate,
        fd_enabled,
    })
}

#[cfg(not(target_os = "linux"))]
pub fn inspect(_iface: &str) -> Result<HostCanStatus, String> {
    Err("SocketCAN configuration is available only on Linux".into())
}

#[cfg(target_os = "linux")]
fn privileged_ip(password: &[u8], args: &[&str]) -> Result<(), String> {
    use std::io::Write;
    use std::process::{Command, Stdio};

    let mut child = Command::new("sudo")
        .args(["-S", "-p", "", "--", "ip"])
        .args(args)
        .stdin(Stdio::piped())
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|error| format!("cannot start sudo: {error}"))?;
    if let Some(mut stdin) = child.stdin.take() {
        // The child may have completed through a cached sudo credential.
        let _ = stdin.write_all(password);
        let _ = stdin.write_all(b"\n");
    }
    let output = child
        .wait_with_output()
        .map_err(|error| error.to_string())?;
    if output.status.success() {
        Ok(())
    } else {
        Err(String::from_utf8_lossy(&output.stderr).trim().to_string())
    }
}

#[cfg(target_os = "linux")]
pub fn apply(iface: &str, bitrate: u32, password: String) -> Result<HostCanStatus, String> {
    let mut secret = password.into_bytes();
    let result = (|| {
        validate_iface(iface)?;
        if ![100_000, 125_000, 250_000, 500_000, 1_000_000].contains(&bitrate) {
            return Err("unsupported Lingkong CAN bitrate".into());
        }
        let before = inspect(iface)?;
        if before.up && before.bitrate == Some(bitrate) && !before.fd_enabled {
            return Ok(before);
        }
        if secret.is_empty() || secret.iter().any(|byte| *byte == b'\n' || *byte == b'\r') {
            return Err("enter the sudo password without a newline".into());
        }
        let bitrate_arg = bitrate.to_string();
        privileged_ip(&secret, &["link", "set", "dev", iface, "down"])
            .map_err(|error| format!("taking {iface} down failed: {error}"))?;
        privileged_ip(
            &secret,
            &[
                "link",
                "set",
                "dev",
                iface,
                "type",
                "can",
                "bitrate",
                &bitrate_arg,
                "fd",
                "off",
            ],
        )
        .map_err(|error| format!("configuring {iface} failed (interface is down): {error}"))?;
        privileged_ip(&secret, &["link", "set", "dev", iface, "up"])
            .map_err(|error| format!("bringing {iface} up failed: {error}"))?;
        let after = inspect(iface)?;
        if !after.up || after.bitrate != Some(bitrate) || after.fd_enabled {
            return Err(format!("{iface} configuration could not be verified"));
        }
        Ok(after)
    })();
    // Minimize the time the sudo credential remains in the backend heap.
    secret.fill(0);
    result
}

#[cfg(not(target_os = "linux"))]
pub fn apply(_iface: &str, _bitrate: u32, _password: String) -> Result<HostCanStatus, String> {
    Err("SocketCAN configuration is available only on Linux".into())
}

#[cfg(test)]
mod tests {
    use super::validate_iface;

    #[test]
    fn interface_names_cannot_become_command_options_or_paths() {
        for bad in [
            "",
            "-help",
            "can0/foo",
            "can0;true",
            "can0\n",
            "abcdefghijklmnop",
        ] {
            assert!(validate_iface(bad).is_err());
        }
        assert!(validate_iface("can0").is_ok());
    }
}
