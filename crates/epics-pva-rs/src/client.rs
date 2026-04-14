use std::ops::ControlFlow;

use spvirit_client::PvGetError;
use spvirit_codec::spvd_decode::{DecodedValue, StructureDesc};

use crate::error::{PvaError, PvaResult};

// ─── Error conversion ────────────────────────────────────────────────────────

fn pva_err(e: PvGetError) -> PvaError {
    match e {
        PvGetError::Io(e) => PvaError::Io(e),
        PvGetError::Timeout(_) => PvaError::Timeout,
        PvGetError::Search(s) => PvaError::ChannelNotFound(s.to_string()),
        PvGetError::Protocol(s) => PvaError::Protocol(s),
        PvGetError::Decode(s) => PvaError::Decode(s),
    }
}

// ─── JSON value conversion for PUT ───────────────────────────────────────────

/// Convert a string value to a serde_json::Value, attempting numeric/boolean
/// parsing first, falling back to a plain string.
fn str_to_json_value(s: &str) -> serde_json::Value {
    if let Ok(v) = serde_json::from_str::<serde_json::Value>(s) {
        return v;
    }
    serde_json::Value::String(s.to_string())
}

/// pvAccess client — delegates to [`spvirit_client::PvaClient`].
pub struct PvaClient {
    inner: spvirit_client::PvaClient,
}

impl PvaClient {
    pub fn new() -> PvaResult<Self> {
        let udp_port = epics_base_rs::runtime::net::pva_broadcast_port();
        let tcp_port = epics_base_rs::runtime::net::pva_server_port();
        Ok(Self {
            inner: spvirit_client::PvaClient::builder()
                .udp_port(udp_port)
                .port(tcp_port)
                .build(),
        })
    }

    /// Create a client targeting specific ports (useful for testing).
    pub fn with_ports(udp_port: u16, tcp_port: u16) -> Self {
        Self {
            inner: spvirit_client::PvaClient::builder()
                .udp_port(udp_port)
                .port(tcp_port)
                .build(),
        }
    }

    // ─── pvaget ──────────────────────────────────────────────────────────

    pub async fn pvaget(&self, pv_name: &str) -> PvaResult<DecodedValue> {
        let result = self.inner.pvget(pv_name).await.map_err(pva_err)?;
        Ok(result.value)
    }

    // ─── pvaput ──────────────────────────────────────────────────────────

    pub async fn pvaput(&self, pv_name: &str, value_str: &str) -> PvaResult<()> {
        let json_val = str_to_json_value(value_str);
        self.inner.pvput(pv_name, json_val).await.map_err(pva_err)
    }

    // ─── pvamonitor ──────────────────────────────────────────────────────

    pub async fn pvamonitor<F>(&self, pv_name: &str, mut callback: F) -> PvaResult<()>
    where
        F: FnMut(&DecodedValue),
    {
        self.inner
            .pvmonitor(pv_name, |val| {
                callback(val);
                ControlFlow::Continue(())
            })
            .await
            .map_err(pva_err)
    }

    // ─── pvainfo ─────────────────────────────────────────────────────────

    pub async fn pvainfo(&self, pv_name: &str) -> PvaResult<StructureDesc> {
        self.inner.pvinfo(pv_name).await.map_err(pva_err)
    }
}
