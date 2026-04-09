//! PVIF: PVData Interface — converts between EPICS record state and PVA structures.
//!
//! Corresponds to C++ QSRV's `pvif.h/pvif.cpp` (ScalarBuilder, etc.).

use std::time::{SystemTime, UNIX_EPOCH};

use epics_base_rs::server::snapshot::{ControlInfo, DisplayInfo, Snapshot};
use epics_base_rs::types::EpicsValue;
use epics_pva_rs::pvdata::{FieldDesc, PvField, PvStructure, ScalarType, ScalarValue};

use crate::convert::{epics_to_pv_field, epics_to_scalar};

/// Field mapping type, corresponding to C++ QSRV PVIFBuilder types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FieldMapping {
    /// NTScalar/NTScalarArray with full metadata (alarm, timestamp, display, control)
    Scalar,
    /// Value only, no metadata
    Plain,
    /// Alarm + timestamp only, no value
    Meta,
    /// Variant union wrapping
    Any,
    /// Process-only: put triggers record processing, no value transfer
    Proc,
}

/// NormativeType classification derived from record type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NtType {
    /// ai, ao, longin, longout, stringin, stringout, calc, calcout
    Scalar,
    /// bi, bo, mbbi, mbbo
    Enum,
    /// waveform, compress, histogram
    ScalarArray,
}

impl NtType {
    /// Determine NtType from EPICS record type name.
    pub fn from_record_type(rtyp: &str) -> Self {
        match rtyp {
            "bi" | "bo" | "mbbi" | "mbbo" => NtType::Enum,
            "waveform" | "compress" | "histogram" => NtType::ScalarArray,
            _ => NtType::Scalar,
        }
    }
}

// ---------------------------------------------------------------------------
// Snapshot → PvStructure conversion
// ---------------------------------------------------------------------------

/// Convert a Snapshot into an NTScalar PvStructure.
///
/// Structure ID: `epics:nt/NTScalar:1.0`
/// Fields: value, alarm, timeStamp, display (optional), control (optional)
pub fn snapshot_to_nt_scalar(snapshot: &Snapshot) -> PvStructure {
    let mut pv = PvStructure::new("epics:nt/NTScalar:1.0");

    // value
    pv.fields
        .push(("value".into(), PvField::Scalar(epics_to_scalar(&snapshot.value))));

    // alarm
    pv.fields
        .push(("alarm".into(), PvField::Structure(build_alarm(snapshot))));

    // timeStamp
    pv.fields.push((
        "timeStamp".into(),
        PvField::Structure(build_timestamp(snapshot.timestamp)),
    ));

    // display
    if let Some(ref disp) = snapshot.display {
        pv.fields
            .push(("display".into(), PvField::Structure(build_display(disp))));
    }

    // control
    if let Some(ref ctrl) = snapshot.control {
        pv.fields
            .push(("control".into(), PvField::Structure(build_control(ctrl))));
    }

    pv
}

/// Convert a Snapshot into an NTEnum PvStructure.
///
/// Structure ID: `epics:nt/NTEnum:1.0`
/// Fields: value{index, choices}, alarm, timeStamp
pub fn snapshot_to_nt_enum(snapshot: &Snapshot) -> PvStructure {
    let mut pv = PvStructure::new("epics:nt/NTEnum:1.0");

    // value sub-structure with index + choices
    let index = match &snapshot.value {
        EpicsValue::Enum(v) => *v as i32,
        EpicsValue::Short(v) => *v as i32,
        other => other.to_f64().map(|f| f as i32).unwrap_or(0),
    };

    let choices: Vec<ScalarValue> = snapshot
        .enums
        .as_ref()
        .map(|e| e.strings.iter().map(|s| ScalarValue::String(s.clone())).collect())
        .unwrap_or_default();

    let mut value_struct = PvStructure::new("enum_t");
    value_struct
        .fields
        .push(("index".into(), PvField::Scalar(ScalarValue::Int(index))));
    value_struct
        .fields
        .push(("choices".into(), PvField::ScalarArray(choices)));

    pv.fields
        .push(("value".into(), PvField::Structure(value_struct)));
    pv.fields
        .push(("alarm".into(), PvField::Structure(build_alarm(snapshot))));
    pv.fields.push((
        "timeStamp".into(),
        PvField::Structure(build_timestamp(snapshot.timestamp)),
    ));

    pv
}

/// Convert a Snapshot into an NTScalarArray PvStructure.
///
/// Structure ID: `epics:nt/NTScalarArray:1.0`
/// Fields: value[], alarm, timeStamp, display (optional)
pub fn snapshot_to_nt_scalar_array(snapshot: &Snapshot) -> PvStructure {
    let mut pv = PvStructure::new("epics:nt/NTScalarArray:1.0");

    // value (array)
    pv.fields
        .push(("value".into(), epics_to_pv_field(&snapshot.value)));

    // alarm
    pv.fields
        .push(("alarm".into(), PvField::Structure(build_alarm(snapshot))));

    // timeStamp
    pv.fields.push((
        "timeStamp".into(),
        PvField::Structure(build_timestamp(snapshot.timestamp)),
    ));

    // display
    if let Some(ref disp) = snapshot.display {
        pv.fields
            .push(("display".into(), PvField::Structure(build_display(disp))));
    }

    pv
}

/// Convert a Snapshot to the appropriate NormativeType based on NtType.
pub fn snapshot_to_pv_structure(snapshot: &Snapshot, nt_type: NtType) -> PvStructure {
    match nt_type {
        NtType::Scalar => snapshot_to_nt_scalar(snapshot),
        NtType::Enum => snapshot_to_nt_enum(snapshot),
        NtType::ScalarArray => snapshot_to_nt_scalar_array(snapshot),
    }
}

// ---------------------------------------------------------------------------
// PvStructure → EpicsValue extraction (for put path)
// ---------------------------------------------------------------------------

/// Extract the primary value from a PvStructure (for put operations).
///
/// For NTScalar: extracts "value" scalar field.
/// For NTEnum: extracts "value.index" as Enum.
/// For NTScalarArray: extracts "value" array.
pub fn pv_structure_to_epics(pv: &PvStructure) -> Option<EpicsValue> {
    let field = pv.get_field("value")?;
    match field {
        PvField::Scalar(sv) => Some(crate::convert::scalar_to_epics(sv)),
        PvField::ScalarArray(_) => crate::convert::pv_field_to_epics(field),
        PvField::Structure(s) => {
            // NTEnum: value is a sub-structure with "index" field
            if let Some(PvField::Scalar(ScalarValue::Int(idx))) = s.get_field("index") {
                Some(EpicsValue::Enum(*idx as u16))
            } else {
                None
            }
        }
    }
}

// ---------------------------------------------------------------------------
// FieldDesc builders (type introspection, no values)
// ---------------------------------------------------------------------------

/// Build a PVA FieldDesc for an NTScalar with the given scalar type.
pub fn build_nt_scalar_desc(scalar_type: ScalarType) -> FieldDesc {
    FieldDesc::Structure {
        struct_id: "epics:nt/NTScalar:1.0".into(),
        fields: vec![
            ("value".into(), FieldDesc::Scalar(scalar_type)),
            ("alarm".into(), alarm_desc()),
            ("timeStamp".into(), timestamp_desc()),
            ("display".into(), display_desc()),
            ("control".into(), control_desc()),
        ],
    }
}

/// Build a PVA FieldDesc for an NTEnum.
pub fn build_nt_enum_desc() -> FieldDesc {
    FieldDesc::Structure {
        struct_id: "epics:nt/NTEnum:1.0".into(),
        fields: vec![
            (
                "value".into(),
                FieldDesc::Structure {
                    struct_id: "enum_t".into(),
                    fields: vec![
                        ("index".into(), FieldDesc::Scalar(ScalarType::Int)),
                        ("choices".into(), FieldDesc::ScalarArray(ScalarType::String)),
                    ],
                },
            ),
            ("alarm".into(), alarm_desc()),
            ("timeStamp".into(), timestamp_desc()),
        ],
    }
}

/// Build a PVA FieldDesc for an NTScalarArray with the given element type.
pub fn build_nt_scalar_array_desc(element_type: ScalarType) -> FieldDesc {
    FieldDesc::Structure {
        struct_id: "epics:nt/NTScalarArray:1.0".into(),
        fields: vec![
            ("value".into(), FieldDesc::ScalarArray(element_type)),
            ("alarm".into(), alarm_desc()),
            ("timeStamp".into(), timestamp_desc()),
            ("display".into(), display_desc()),
        ],
    }
}

/// Build the appropriate FieldDesc based on NtType and scalar type.
pub fn build_field_desc_for_nt(nt_type: NtType, scalar_type: ScalarType) -> FieldDesc {
    match nt_type {
        NtType::Scalar => build_nt_scalar_desc(scalar_type),
        NtType::Enum => build_nt_enum_desc(),
        NtType::ScalarArray => build_nt_scalar_array_desc(scalar_type),
    }
}

// ---------------------------------------------------------------------------
// Helper builders
// ---------------------------------------------------------------------------

fn build_alarm(snapshot: &Snapshot) -> PvStructure {
    let mut alarm = PvStructure::new("alarm_t");
    alarm.fields.push((
        "severity".into(),
        PvField::Scalar(ScalarValue::Int(snapshot.alarm.severity as i32)),
    ));
    alarm.fields.push((
        "status".into(),
        PvField::Scalar(ScalarValue::Int(snapshot.alarm.status as i32)),
    ));
    alarm.fields.push((
        "message".into(),
        PvField::Scalar(ScalarValue::String(alarm_severity_string(
            snapshot.alarm.severity,
        ))),
    ));
    alarm
}

fn build_timestamp(time: SystemTime) -> PvStructure {
    let mut ts = PvStructure::new("time_t");
    let (secs, nanos) = match time.duration_since(UNIX_EPOCH) {
        Ok(d) => (d.as_secs() as i64, d.subsec_nanos() as i32),
        Err(_) => (0, 0),
    };
    // PVA timestamps use EPICS epoch (1990-01-01), but for now use UNIX epoch
    // to match the Rust SystemTime. Epoch adjustment can be added when
    // epics-pva-rs server serialization handles it.
    ts.fields
        .push(("secondsPastEpoch".into(), PvField::Scalar(ScalarValue::Long(secs))));
    ts.fields
        .push(("nanoseconds".into(), PvField::Scalar(ScalarValue::Int(nanos))));
    ts.fields
        .push(("userTag".into(), PvField::Scalar(ScalarValue::Int(0))));
    ts
}

fn build_display(disp: &DisplayInfo) -> PvStructure {
    let mut d = PvStructure::new("display_t");
    d.fields.push((
        "limitLow".into(),
        PvField::Scalar(ScalarValue::Double(disp.lower_disp_limit)),
    ));
    d.fields.push((
        "limitHigh".into(),
        PvField::Scalar(ScalarValue::Double(disp.upper_disp_limit)),
    ));
    d.fields.push((
        "description".into(),
        PvField::Scalar(ScalarValue::String(String::new())),
    ));
    d.fields.push((
        "units".into(),
        PvField::Scalar(ScalarValue::String(disp.units.clone())),
    ));
    d.fields.push((
        "precision".into(),
        PvField::Scalar(ScalarValue::Int(disp.precision as i32)),
    ));
    d.fields.push((
        "form".into(),
        PvField::Scalar(ScalarValue::Int(0)), // Default form
    ));
    d
}

fn build_control(ctrl: &ControlInfo) -> PvStructure {
    let mut c = PvStructure::new("control_t");
    c.fields.push((
        "limitLow".into(),
        PvField::Scalar(ScalarValue::Double(ctrl.lower_ctrl_limit)),
    ));
    c.fields.push((
        "limitHigh".into(),
        PvField::Scalar(ScalarValue::Double(ctrl.upper_ctrl_limit)),
    ));
    c.fields.push((
        "minStep".into(),
        PvField::Scalar(ScalarValue::Double(0.0)),
    ));
    c
}

fn alarm_desc() -> FieldDesc {
    FieldDesc::Structure {
        struct_id: "alarm_t".into(),
        fields: vec![
            ("severity".into(), FieldDesc::Scalar(ScalarType::Int)),
            ("status".into(), FieldDesc::Scalar(ScalarType::Int)),
            ("message".into(), FieldDesc::Scalar(ScalarType::String)),
        ],
    }
}

fn timestamp_desc() -> FieldDesc {
    FieldDesc::Structure {
        struct_id: "time_t".into(),
        fields: vec![
            ("secondsPastEpoch".into(), FieldDesc::Scalar(ScalarType::Long)),
            ("nanoseconds".into(), FieldDesc::Scalar(ScalarType::Int)),
            ("userTag".into(), FieldDesc::Scalar(ScalarType::Int)),
        ],
    }
}

fn display_desc() -> FieldDesc {
    FieldDesc::Structure {
        struct_id: "display_t".into(),
        fields: vec![
            ("limitLow".into(), FieldDesc::Scalar(ScalarType::Double)),
            ("limitHigh".into(), FieldDesc::Scalar(ScalarType::Double)),
            ("description".into(), FieldDesc::Scalar(ScalarType::String)),
            ("units".into(), FieldDesc::Scalar(ScalarType::String)),
            ("precision".into(), FieldDesc::Scalar(ScalarType::Int)),
            ("form".into(), FieldDesc::Scalar(ScalarType::Int)),
        ],
    }
}

fn control_desc() -> FieldDesc {
    FieldDesc::Structure {
        struct_id: "control_t".into(),
        fields: vec![
            ("limitLow".into(), FieldDesc::Scalar(ScalarType::Double)),
            ("limitHigh".into(), FieldDesc::Scalar(ScalarType::Double)),
            ("minStep".into(), FieldDesc::Scalar(ScalarType::Double)),
        ],
    }
}

fn alarm_severity_string(severity: u16) -> String {
    match severity {
        0 => "NO_ALARM".into(),
        1 => "MINOR".into(),
        2 => "MAJOR".into(),
        3 => "INVALID".into(),
        _ => format!("UNKNOWN({severity})"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_base_rs::server::snapshot::{AlarmInfo, EnumInfo, Snapshot};

    fn test_snapshot(value: EpicsValue) -> Snapshot {
        Snapshot {
            value,
            alarm: AlarmInfo {
                status: 0,
                severity: 0,
            },
            timestamp: UNIX_EPOCH,
            display: Some(DisplayInfo {
                units: "degC".into(),
                precision: 3,
                upper_disp_limit: 100.0,
                lower_disp_limit: 0.0,
                upper_alarm_limit: 90.0,
                upper_warning_limit: 80.0,
                lower_warning_limit: 10.0,
                lower_alarm_limit: 5.0,
            }),
            control: Some(ControlInfo {
                upper_ctrl_limit: 100.0,
                lower_ctrl_limit: 0.0,
            }),
            enums: None,
        }
    }

    #[test]
    fn nt_scalar_structure() {
        let snap = test_snapshot(EpicsValue::Double(42.5));
        let pv = snapshot_to_nt_scalar(&snap);

        assert_eq!(pv.struct_id, "epics:nt/NTScalar:1.0");
        assert_eq!(pv.get_value(), Some(&ScalarValue::Double(42.5)));
        assert!(pv.get_alarm().is_some());
        assert!(pv.get_timestamp().is_some());
        assert!(pv.get_field("display").is_some());
        assert!(pv.get_field("control").is_some());
    }

    #[test]
    fn nt_enum_structure() {
        let snap = Snapshot {
            value: EpicsValue::Enum(1),
            alarm: AlarmInfo {
                status: 0,
                severity: 0,
            },
            timestamp: UNIX_EPOCH,
            display: None,
            control: None,
            enums: Some(EnumInfo {
                strings: vec!["Off".into(), "On".into()],
            }),
        };
        let pv = snapshot_to_nt_enum(&snap);

        assert_eq!(pv.struct_id, "epics:nt/NTEnum:1.0");
        // value is a sub-structure
        if let Some(PvField::Structure(val)) = pv.get_field("value") {
            if let Some(PvField::Scalar(ScalarValue::Int(idx))) = val.get_field("index") {
                assert_eq!(*idx, 1);
            } else {
                panic!("expected index scalar");
            }
            if let Some(PvField::ScalarArray(choices)) = val.get_field("choices") {
                assert_eq!(choices.len(), 2);
            } else {
                panic!("expected choices array");
            }
        } else {
            panic!("expected value structure");
        }
    }

    #[test]
    fn nt_scalar_array_structure() {
        let snap = test_snapshot(EpicsValue::DoubleArray(vec![1.0, 2.0, 3.0]));
        let pv = snapshot_to_nt_scalar_array(&snap);

        assert_eq!(pv.struct_id, "epics:nt/NTScalarArray:1.0");
        if let Some(PvField::ScalarArray(arr)) = pv.get_field("value") {
            assert_eq!(arr.len(), 3);
        } else {
            panic!("expected value array");
        }
    }

    #[test]
    fn put_roundtrip_scalar() {
        let snap = test_snapshot(EpicsValue::Double(99.0));
        let pv = snapshot_to_nt_scalar(&snap);
        let back = pv_structure_to_epics(&pv).unwrap();
        assert_eq!(back, EpicsValue::Double(99.0));
    }

    #[test]
    fn put_roundtrip_enum() {
        let snap = Snapshot {
            value: EpicsValue::Enum(2),
            alarm: AlarmInfo {
                status: 0,
                severity: 0,
            },
            timestamp: UNIX_EPOCH,
            display: None,
            control: None,
            enums: Some(EnumInfo {
                strings: vec!["A".into(), "B".into(), "C".into()],
            }),
        };
        let pv = snapshot_to_nt_enum(&snap);
        let back = pv_structure_to_epics(&pv).unwrap();
        assert_eq!(back, EpicsValue::Enum(2));
    }

    #[test]
    fn nt_type_from_record_type() {
        assert_eq!(NtType::from_record_type("ai"), NtType::Scalar);
        assert_eq!(NtType::from_record_type("bi"), NtType::Enum);
        assert_eq!(NtType::from_record_type("waveform"), NtType::ScalarArray);
        assert_eq!(NtType::from_record_type("calc"), NtType::Scalar);
        assert_eq!(NtType::from_record_type("mbbi"), NtType::Enum);
    }

    #[test]
    fn field_desc_nt_scalar() {
        let desc = build_nt_scalar_desc(ScalarType::Double);
        assert_eq!(desc.value_scalar_type(), Some(ScalarType::Double));
        assert_eq!(desc.field_count(), 5);
    }
}
