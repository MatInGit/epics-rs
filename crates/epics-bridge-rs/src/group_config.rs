//! Group PV JSON configuration parser.
//!
//! Parses C++ QSRV-compatible group definitions from JSON.
//! See `~/epics-base/modules/pva2pva/pdbApp/configparse.cpp` for the
//! original C++ parser.
//!
//! # JSON format
//!
//! ```json
//! {
//!   "GROUP:NAME": {
//!     "+id": "some/NT:1.0",
//!     "+atomic": true,
//!     "fieldName": {
//!       "+type": "scalar",
//!       "+channel": "RECORD:FIELD",
//!       "+trigger": "*",
//!       "+putorder": 0
//!     }
//!   }
//! }
//! ```

use serde::Deserialize;
use std::collections::HashMap;

use crate::error::{BridgeError, BridgeResult};
use crate::pvif::FieldMapping;

/// Definition of a group PV (multiple records composited into one PvStructure).
#[derive(Debug, Clone)]
pub struct GroupPvDef {
    pub name: String,
    pub struct_id: Option<String>,
    pub atomic: bool,
    pub members: Vec<GroupMember>,
}

/// A single member within a group PV.
#[derive(Debug, Clone)]
pub struct GroupMember {
    /// Field path within the group structure (e.g., "temperature").
    pub field_name: String,
    /// Source record and field (e.g., "TEMP:ai.VAL").
    pub channel: String,
    /// How to map the record field to PVA structure.
    pub mapping: FieldMapping,
    /// Which fields to update when this member changes.
    pub triggers: TriggerDef,
    /// Ordering for put operations.
    pub put_order: i32,
    /// Optional structure ID for this member (from `+id`).
    pub struct_id: Option<String>,
}

/// Defines which group fields are updated when a member's source record changes.
#[derive(Debug, Clone)]
pub enum TriggerDef {
    /// `"*"` — update all fields in the group.
    All,
    /// Named fields — update only these fields.
    Fields(Vec<String>),
    /// `""` — never trigger a group update for this member.
    None,
}

/// Parse group definitions from a JSON string.
///
/// The JSON is a top-level object where each key is a group name.
pub fn parse_group_config(json: &str) -> BridgeResult<Vec<GroupPvDef>> {
    let root: HashMap<String, RawGroupDef> =
        serde_json::from_str(json).map_err(|e| BridgeError::GroupConfigError(e.to_string()))?;

    let mut groups = Vec::new();
    for (name, raw) in root {
        groups.push(raw_to_group_def(name, raw)?);
    }
    groups.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(groups)
}

// ---------------------------------------------------------------------------
// Internal JSON deserialization types
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
struct RawGroupDef {
    #[serde(rename = "+id")]
    id: Option<String>,
    #[serde(rename = "+atomic", default = "default_atomic")]
    atomic: bool,
    #[serde(flatten)]
    fields: HashMap<String, serde_json::Value>,
}

fn default_atomic() -> bool {
    true
}

fn raw_to_group_def(name: String, raw: RawGroupDef) -> BridgeResult<GroupPvDef> {
    let mut members = Vec::new();

    for (field_name, value) in &raw.fields {
        // Skip meta-keys (already extracted via named fields)
        if field_name.starts_with('+') {
            continue;
        }

        let member = parse_member(field_name, value)?;
        members.push(member);
    }

    // Sort by put_order for deterministic ordering
    members.sort_by_key(|m| m.put_order);

    Ok(GroupPvDef {
        name,
        struct_id: raw.id,
        atomic: raw.atomic,
        members,
    })
}

fn parse_member(field_name: &str, value: &serde_json::Value) -> BridgeResult<GroupMember> {
    let obj = value
        .as_object()
        .ok_or_else(|| BridgeError::GroupConfigError(format!("field '{field_name}' must be an object")))?;

    let channel = obj
        .get("+channel")
        .and_then(|v| v.as_str())
        .ok_or_else(|| {
            BridgeError::GroupConfigError(format!("field '{field_name}' missing +channel"))
        })?
        .to_string();

    let mapping = match obj.get("+type").and_then(|v| v.as_str()) {
        Some("scalar") | None => FieldMapping::Scalar,
        Some("plain") => FieldMapping::Plain,
        Some("meta") => FieldMapping::Meta,
        Some("any") => FieldMapping::Any,
        Some("proc") => FieldMapping::Proc,
        Some(other) => {
            return Err(BridgeError::GroupConfigError(format!(
                "unknown +type '{other}' for field '{field_name}'"
            )));
        }
    };

    let triggers = match obj.get("+trigger").and_then(|v| v.as_str()) {
        Some("*") | None => TriggerDef::All,
        Some("") => TriggerDef::None,
        Some(s) => TriggerDef::Fields(s.split(',').map(|f| f.trim().to_string()).collect()),
    };

    let put_order = obj
        .get("+putorder")
        .and_then(|v| v.as_i64())
        .unwrap_or(0) as i32;

    let struct_id = obj
        .get("+id")
        .and_then(|v| v.as_str())
        .map(|s| s.to_string());

    Ok(GroupMember {
        field_name: field_name.to_string(),
        channel,
        mapping,
        triggers,
        put_order,
        struct_id,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_basic_group() {
        let json = r#"{
            "TEST:group": {
                "+id": "epics:nt/NTTable:1.0",
                "+atomic": true,
                "temperature": {
                    "+type": "scalar",
                    "+channel": "TEMP:ai",
                    "+trigger": "*",
                    "+putorder": 0
                },
                "pressure": {
                    "+type": "scalar",
                    "+channel": "PRESS:ai",
                    "+trigger": "temperature,pressure",
                    "+putorder": 1
                }
            }
        }"#;

        let groups = parse_group_config(json).unwrap();
        assert_eq!(groups.len(), 1);

        let g = &groups[0];
        assert_eq!(g.name, "TEST:group");
        assert_eq!(g.struct_id.as_deref(), Some("epics:nt/NTTable:1.0"));
        assert!(g.atomic);
        assert_eq!(g.members.len(), 2);

        let temp = &g.members[0];
        assert_eq!(temp.field_name, "temperature");
        assert_eq!(temp.channel, "TEMP:ai");
        assert_eq!(temp.mapping, FieldMapping::Scalar);
        assert!(matches!(temp.triggers, TriggerDef::All));
        assert_eq!(temp.put_order, 0);

        let press = &g.members[1];
        assert_eq!(press.field_name, "pressure");
        assert_eq!(press.channel, "PRESS:ai");
        if let TriggerDef::Fields(ref fields) = press.triggers {
            assert_eq!(fields, &["temperature", "pressure"]);
        } else {
            panic!("expected TriggerDef::Fields");
        }
    }

    #[test]
    fn parse_minimal_member() {
        let json = r#"{
            "GRP:min": {
                "val": {
                    "+channel": "REC:val"
                }
            }
        }"#;

        let groups = parse_group_config(json).unwrap();
        let m = &groups[0].members[0];
        assert_eq!(m.mapping, FieldMapping::Scalar); // default
        assert!(matches!(m.triggers, TriggerDef::All)); // default
        assert_eq!(m.put_order, 0); // default
    }

    #[test]
    fn parse_proc_mapping() {
        let json = r#"{
            "GRP:proc": {
                "trigger": {
                    "+type": "proc",
                    "+channel": "REC:proc",
                    "+trigger": ""
                }
            }
        }"#;

        let groups = parse_group_config(json).unwrap();
        let m = &groups[0].members[0];
        assert_eq!(m.mapping, FieldMapping::Proc);
        assert!(matches!(m.triggers, TriggerDef::None));
    }

    #[test]
    fn parse_error_missing_channel() {
        let json = r#"{
            "GRP:bad": {
                "val": {
                    "+type": "scalar"
                }
            }
        }"#;

        assert!(parse_group_config(json).is_err());
    }

    #[test]
    fn parse_multiple_groups() {
        let json = r#"{
            "GRP:b": {
                "x": { "+channel": "B:x" }
            },
            "GRP:a": {
                "y": { "+channel": "A:y" }
            }
        }"#;

        let groups = parse_group_config(json).unwrap();
        assert_eq!(groups.len(), 2);
        // Sorted by name
        assert_eq!(groups[0].name, "GRP:a");
        assert_eq!(groups[1].name, "GRP:b");
    }

    #[test]
    fn parse_member_id() {
        let json = r#"{
            "GRP:id": {
                "sensor": {
                    "+channel": "SENSOR:ai",
                    "+id": "epics:nt/NTScalar:1.0"
                }
            }
        }"#;

        let groups = parse_group_config(json).unwrap();
        let m = &groups[0].members[0];
        assert_eq!(m.struct_id.as_deref(), Some("epics:nt/NTScalar:1.0"));
    }

    #[test]
    fn parse_member_no_id() {
        let json = r#"{
            "GRP:noid": {
                "val": { "+channel": "REC:val" }
            }
        }"#;

        let groups = parse_group_config(json).unwrap();
        assert!(groups[0].members[0].struct_id.is_none());
    }
}
