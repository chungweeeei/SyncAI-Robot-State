//! `RobotState.network_status.wifi_info`, which is a JSON **string** rather than a typed
//! sub-message.
//!
//! The C++ node builds it with nlohmann/json; serde_json produces the same bytes. Two properties
//! of that output are load bearing, because the backend parses this field:
//!
//! * **Keys come out alphabetically.** nlohmann's `json` object is a `std::map`, so the
//!   initializer-list order in the C++ source is not the output order. serde_json's default `Map`
//!   is a `BTreeMap`, which orders the same way.
//! * **No sample dumps to the literal string `"null"`**, not `""` and not `{}`. An empty
//!   nlohmann json is a null, and the backend parses defensively against exactly that value.

use serde_json::json;

/// The `syncai_common/WifiStatus` fields this node forwards, decoded at the subscription boundary
/// so everything below is plain Rust.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct WifiInfo {
    pub ssid: String,
    pub bssid: String,
    pub rssi: i8,
    pub ip_address: String,
    pub mac_address: String,
}

/// Flatten the latest `wifi_status` into the JSON string the field carries.
pub fn wifi_info_json(info: Option<&WifiInfo>) -> String {
    let Some(info) = info else {
        return "null".to_owned();
    };

    json!({
        "ssid": info.ssid,
        "bssid": info.bssid,
        "rssi": info.rssi,
        "ip_address": info.ip_address,
        "mac_address": info.mac_address,
    })
    .to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> WifiInfo {
        WifiInfo {
            ssid: "syncai-5g".into(),
            bssid: "aa:bb:cc:dd:ee:ff".into(),
            rssi: -47,
            ip_address: "192.168.1.103".into(),
            mac_address: "11:22:33:44:55:66".into(),
        }
    }

    #[test]
    fn no_sample_is_the_literal_null_the_backend_expects() {
        assert_eq!(wifi_info_json(None), "null");
    }

    /// Byte-for-byte, including the alphabetical key order nlohmann/json produces.
    #[test]
    fn a_sample_is_flattened_with_alphabetically_ordered_keys() {
        assert_eq!(
            wifi_info_json(Some(&sample())),
            r#"{"bssid":"aa:bb:cc:dd:ee:ff","ip_address":"192.168.1.103","mac_address":"11:22:33:44:55:66","rssi":-47,"ssid":"syncai-5g"}"#
        );
    }

    /// An SSID is user-chosen and may contain anything; a hand-rolled writer is where this breaks.
    #[test]
    fn quotes_and_backslashes_in_an_ssid_are_escaped() {
        let info = WifiInfo {
            ssid: r#"it's "guest" \ net"#.into(),
            ..WifiInfo::default()
        };
        let json = wifi_info_json(Some(&info));

        assert!(json.contains(r#""ssid":"it's \"guest\" \\ net""#), "{json}");
        // Still parses back to exactly what went in
        let parsed: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed["ssid"], info.ssid);
    }

    #[test]
    fn rssi_is_a_number_not_a_string() {
        let json = wifi_info_json(Some(&sample()));
        let parsed: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed["rssi"], -47);
    }
}
