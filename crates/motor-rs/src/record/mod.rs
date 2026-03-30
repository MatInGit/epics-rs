mod state_machine;
mod command_planner;
mod status_update;

use epics_base_rs::error::{CaError, CaResult};
use epics_base_rs::server::record::{FieldDesc, Record, RecordProcessResult};
use epics_base_rs::types::{DbFieldType, EpicsValue};

use crate::coordinate;
use crate::device_state::*;
use crate::fields::*;
use crate::flags::*;

/// EPICS Motor Record implementation.
#[derive(Debug, Clone)]
pub struct MotorRecord {
    pub pos: PositionFields,
    pub conv: ConversionFields,
    pub vel: VelocityFields,
    pub retry: RetryFields,
    pub limits: LimitFields,
    pub ctrl: ControlFields,
    pub stat: StatusFields,
    pub pid: PidFields,
    pub disp: DisplayFields,
    pub timing: TimingFields,
    pub internal: InternalFields,
    /// Pending event for next process() call
    pending_event: Option<MotorEvent>,
    /// Track which field was last written (for process)
    last_write: Option<CommandSource>,
    /// Suppress FLNK during motion
    suppress_flnk: bool,
    /// Shared state mailbox for device communication
    device_state: Option<SharedDeviceState>,
    /// Last seen status sequence number
    last_seen_seq: u64,
    /// Whether initial readback has been performed
    initialized: bool,
    /// Monotonic counter for delay request IDs
    next_delay_id: u64,
}

impl Default for MotorRecord {
    fn default() -> Self {
        Self {
            pos: PositionFields::default(),
            conv: ConversionFields::default(),
            vel: VelocityFields::default(),
            retry: RetryFields::default(),
            limits: LimitFields::default(),
            ctrl: ControlFields::default(),
            stat: StatusFields::default(),
            pid: PidFields::default(),
            disp: DisplayFields::default(),
            timing: TimingFields::default(),
            internal: InternalFields::default(),
            pending_event: None,
            last_write: None,
            suppress_flnk: false,
            device_state: None,
            last_seen_seq: 0,
            initialized: false,
            next_delay_id: 0,
        }
    }
}

/// Motion direction for hardware limit checks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MotionDirection {
    Positive,
    Negative,
}

impl MotorRecord {
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a motor record wired to a shared device state mailbox.
    pub fn with_device_state(mut self, state: SharedDeviceState) -> Self {
        self.device_state = Some(state);
        self
    }

    /// Set the shared device state (for late injection by device support init).
    pub fn set_device_state(&mut self, state: SharedDeviceState) {
        self.device_state = Some(state);
    }

    /// Set a pending event for the next process() call.
    pub fn set_event(&mut self, event: MotorEvent) {
        self.pending_event = Some(event);
    }
}

static FIELDS: &[FieldDesc] = &[
    // Position
    FieldDesc { name: "VAL", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "RBV", dbf_type: DbFieldType::Double, read_only: true },
    FieldDesc { name: "RLV", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "OFF", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "DIFF", dbf_type: DbFieldType::Double, read_only: true },
    FieldDesc { name: "RDIF", dbf_type: DbFieldType::Double, read_only: true },
    FieldDesc { name: "DVAL", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "DRBV", dbf_type: DbFieldType::Double, read_only: true },
    FieldDesc { name: "RVAL", dbf_type: DbFieldType::Long, read_only: false },
    FieldDesc { name: "RRBV", dbf_type: DbFieldType::Long, read_only: true },
    FieldDesc { name: "RMP", dbf_type: DbFieldType::Long, read_only: true },
    FieldDesc { name: "REP", dbf_type: DbFieldType::Long, read_only: true },
    // Conversion
    FieldDesc { name: "DIR", dbf_type: DbFieldType::Short, read_only: false },
    FieldDesc { name: "FOFF", dbf_type: DbFieldType::Short, read_only: false },
    FieldDesc { name: "SET", dbf_type: DbFieldType::Short, read_only: false },
    FieldDesc { name: "IGSET", dbf_type: DbFieldType::Short, read_only: false },
    FieldDesc { name: "MRES", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "ERES", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "SREV", dbf_type: DbFieldType::Long, read_only: false },
    FieldDesc { name: "UREV", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "UEIP", dbf_type: DbFieldType::Short, read_only: false },
    FieldDesc { name: "URIP", dbf_type: DbFieldType::Short, read_only: false },
    FieldDesc { name: "RRES", dbf_type: DbFieldType::Double, read_only: false },
    // Velocity
    FieldDesc { name: "VELO", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "VBAS", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "VMAX", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "S", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "SBAS", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "SMAX", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "ACCL", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "BVEL", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "BACC", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "HVEL", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "JVEL", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "JAR", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "SBAK", dbf_type: DbFieldType::Double, read_only: false },
    // Retry
    FieldDesc { name: "BDST", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "FRAC", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "RDBD", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "SPDB", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "RTRY", dbf_type: DbFieldType::Short, read_only: false },
    FieldDesc { name: "RMOD", dbf_type: DbFieldType::Short, read_only: false },
    FieldDesc { name: "RCNT", dbf_type: DbFieldType::Short, read_only: true },
    FieldDesc { name: "MISS", dbf_type: DbFieldType::Short, read_only: true },
    // Limits
    FieldDesc { name: "HLM", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "LLM", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "DHLM", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "DLLM", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "LVIO", dbf_type: DbFieldType::Short, read_only: true },
    FieldDesc { name: "HLS", dbf_type: DbFieldType::Short, read_only: true },
    FieldDesc { name: "LLS", dbf_type: DbFieldType::Short, read_only: true },
    FieldDesc { name: "HLSV", dbf_type: DbFieldType::Short, read_only: false },
    // Control
    FieldDesc { name: "SPMG", dbf_type: DbFieldType::Short, read_only: false },
    FieldDesc { name: "STOP", dbf_type: DbFieldType::Short, read_only: false },
    FieldDesc { name: "HOMF", dbf_type: DbFieldType::Short, read_only: false },
    FieldDesc { name: "HOMR", dbf_type: DbFieldType::Short, read_only: false },
    FieldDesc { name: "JOGF", dbf_type: DbFieldType::Short, read_only: false },
    FieldDesc { name: "JOGR", dbf_type: DbFieldType::Short, read_only: false },
    FieldDesc { name: "TWF", dbf_type: DbFieldType::Short, read_only: false },
    FieldDesc { name: "TWR", dbf_type: DbFieldType::Short, read_only: false },
    FieldDesc { name: "TWV", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "CNEN", dbf_type: DbFieldType::Short, read_only: false },
    // Status
    FieldDesc { name: "DMOV", dbf_type: DbFieldType::Short, read_only: true },
    FieldDesc { name: "MOVN", dbf_type: DbFieldType::Short, read_only: true },
    FieldDesc { name: "MSTA", dbf_type: DbFieldType::Long, read_only: true },
    FieldDesc { name: "MIP", dbf_type: DbFieldType::Short, read_only: true },
    FieldDesc { name: "CDIR", dbf_type: DbFieldType::Short, read_only: true },
    FieldDesc { name: "TDIR", dbf_type: DbFieldType::Short, read_only: true },
    FieldDesc { name: "ATHM", dbf_type: DbFieldType::Short, read_only: true },
    FieldDesc { name: "STUP", dbf_type: DbFieldType::Short, read_only: false },
    // PID
    FieldDesc { name: "PCOF", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "ICOF", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "DCOF", dbf_type: DbFieldType::Double, read_only: false },
    // Display
    FieldDesc { name: "EGU", dbf_type: DbFieldType::String, read_only: false },
    FieldDesc { name: "PREC", dbf_type: DbFieldType::Short, read_only: false },
    FieldDesc { name: "ADEL", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "MDEL", dbf_type: DbFieldType::Double, read_only: false },
    // Timing
    FieldDesc { name: "DLY", dbf_type: DbFieldType::Double, read_only: false },
    FieldDesc { name: "NTM", dbf_type: DbFieldType::Short, read_only: false },
    FieldDesc { name: "NTMF", dbf_type: DbFieldType::Double, read_only: false },
];

impl Record for MotorRecord {
    fn record_type(&self) -> &'static str {
        "motor"
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }

    fn can_device_write(&self) -> bool {
        true
    }

    fn is_put_complete(&self) -> bool {
        self.stat.dmov
    }

    fn process(&mut self) -> CaResult<RecordProcessResult> {
        // If wired to device state, determine event from shared mailbox
        if self.device_state.is_some() {
            if let Some(event) = self.determine_event() {
                self.pending_event = Some(event);
            }
        }

        let effects = self.do_process();
        let move_started = !self.stat.dmov && !effects.commands.is_empty();

        // Write effects to shared mailbox for DeviceSupport.write() to consume
        if let Some(state) = self.device_state.clone() {
            self.suppress_flnk = effects.suppress_forward_link;
            let actions = self.effects_to_actions(&effects);
            match state.lock() {
                Ok(mut ds) => { ds.pending_actions = Some(actions); }
                Err(e) => { tracing::error!("device state lock poisoned in process: {e}"); }
            }
        }

        if move_started {
            // Flush DMOV=0 immediately so monitors see the 1->0 transition
            // before the move completes. The next I/O Intr cycle will
            // process again and eventually notify DMOV=1.
            use epics_base_rs::types::EpicsValue;
            Ok(RecordProcessResult::AsyncPendingNotify(vec![
                ("DMOV".to_string(), EpicsValue::Short(0)),
                ("MOVN".to_string(), EpicsValue::Short(1)),
                ("VAL".to_string(), EpicsValue::Double(self.pos.val)),
                ("DVAL".to_string(), EpicsValue::Double(self.pos.dval)),
                ("RVAL".to_string(), EpicsValue::Long(self.pos.rval)),
            ]))
        } else {
            Ok(RecordProcessResult::Complete)
        }
    }

    fn should_fire_forward_link(&self) -> bool {
        !self.suppress_flnk
    }

    fn get_field(&self, name: &str) -> Option<EpicsValue> {
        match name {
            // Position
            "VAL" => Some(EpicsValue::Double(self.pos.val)),
            "RBV" => Some(EpicsValue::Double(self.pos.rbv)),
            "RLV" => Some(EpicsValue::Double(self.pos.rlv)),
            "OFF" => Some(EpicsValue::Double(self.pos.off)),
            "DIFF" => Some(EpicsValue::Double(self.pos.diff)),
            "RDIF" => Some(EpicsValue::Double(self.pos.rdif)),
            "DVAL" => Some(EpicsValue::Double(self.pos.dval)),
            "DRBV" => Some(EpicsValue::Double(self.pos.drbv)),
            "RVAL" => Some(EpicsValue::Long(self.pos.rval)),
            "RRBV" => Some(EpicsValue::Long(self.pos.rrbv)),
            "RMP" => Some(EpicsValue::Long(self.pos.rmp)),
            "REP" => Some(EpicsValue::Long(self.pos.rep)),
            // Conversion
            "DIR" => Some(EpicsValue::Short(self.conv.dir as i16)),
            "FOFF" => Some(EpicsValue::Short(self.conv.foff as i16)),
            "SET" => Some(EpicsValue::Short(if self.conv.set { 1 } else { 0 })),
            "IGSET" => Some(EpicsValue::Short(if self.conv.igset { 1 } else { 0 })),
            "MRES" => Some(EpicsValue::Double(self.conv.mres)),
            "ERES" => Some(EpicsValue::Double(self.conv.eres)),
            "SREV" => Some(EpicsValue::Long(self.conv.srev)),
            "UREV" => Some(EpicsValue::Double(self.conv.urev)),
            "UEIP" => Some(EpicsValue::Short(if self.conv.ueip { 1 } else { 0 })),
            "URIP" => Some(EpicsValue::Short(if self.conv.urip { 1 } else { 0 })),
            "RRES" => Some(EpicsValue::Double(self.conv.rres)),
            // Velocity
            "VELO" => Some(EpicsValue::Double(self.vel.velo)),
            "VBAS" => Some(EpicsValue::Double(self.vel.vbas)),
            "VMAX" => Some(EpicsValue::Double(self.vel.vmax)),
            "S" => Some(EpicsValue::Double(self.vel.s)),
            "SBAS" => Some(EpicsValue::Double(self.vel.sbas)),
            "SMAX" => Some(EpicsValue::Double(self.vel.smax)),
            "ACCL" => Some(EpicsValue::Double(self.vel.accl)),
            "BVEL" => Some(EpicsValue::Double(self.vel.bvel)),
            "BACC" => Some(EpicsValue::Double(self.vel.bacc)),
            "HVEL" => Some(EpicsValue::Double(self.vel.hvel)),
            "JVEL" => Some(EpicsValue::Double(self.vel.jvel)),
            "JAR" => Some(EpicsValue::Double(self.vel.jar)),
            "SBAK" => Some(EpicsValue::Double(self.vel.sbak)),
            // Retry
            "BDST" => Some(EpicsValue::Double(self.retry.bdst)),
            "FRAC" => Some(EpicsValue::Double(self.retry.frac)),
            "RDBD" => Some(EpicsValue::Double(self.retry.rdbd)),
            "SPDB" => Some(EpicsValue::Double(self.retry.spdb)),
            "RTRY" => Some(EpicsValue::Short(self.retry.rtry)),
            "RMOD" => Some(EpicsValue::Short(self.retry.rmod as i16)),
            "RCNT" => Some(EpicsValue::Short(self.retry.rcnt)),
            "MISS" => Some(EpicsValue::Short(if self.retry.miss { 1 } else { 0 })),
            // Limits
            "HLM" => Some(EpicsValue::Double(self.limits.hlm)),
            "LLM" => Some(EpicsValue::Double(self.limits.llm)),
            "DHLM" => Some(EpicsValue::Double(self.limits.dhlm)),
            "DLLM" => Some(EpicsValue::Double(self.limits.dllm)),
            "LVIO" => Some(EpicsValue::Short(if self.limits.lvio { 1 } else { 0 })),
            "HLS" => Some(EpicsValue::Short(if self.limits.hls { 1 } else { 0 })),
            "LLS" => Some(EpicsValue::Short(if self.limits.lls { 1 } else { 0 })),
            "HLSV" => Some(EpicsValue::Short(self.limits.hlsv)),
            // Control
            "SPMG" => Some(EpicsValue::Short(self.ctrl.spmg as i16)),
            "STOP" => Some(EpicsValue::Short(if self.ctrl.stop { 1 } else { 0 })),
            "HOMF" => Some(EpicsValue::Short(if self.ctrl.homf { 1 } else { 0 })),
            "HOMR" => Some(EpicsValue::Short(if self.ctrl.homr { 1 } else { 0 })),
            "JOGF" => Some(EpicsValue::Short(if self.ctrl.jogf { 1 } else { 0 })),
            "JOGR" => Some(EpicsValue::Short(if self.ctrl.jogr { 1 } else { 0 })),
            "TWF" => Some(EpicsValue::Short(if self.ctrl.twf { 1 } else { 0 })),
            "TWR" => Some(EpicsValue::Short(if self.ctrl.twr { 1 } else { 0 })),
            "TWV" => Some(EpicsValue::Double(self.ctrl.twv)),
            "CNEN" => Some(EpicsValue::Short(if self.ctrl.cnen { 1 } else { 0 })),
            // Status
            "DMOV" => Some(EpicsValue::Short(if self.stat.dmov { 1 } else { 0 })),
            "MOVN" => Some(EpicsValue::Short(if self.stat.movn { 1 } else { 0 })),
            "MSTA" => Some(EpicsValue::Long(self.stat.msta.bits() as i32)),
            "MIP" => Some(EpicsValue::Short(self.stat.mip.bits() as i16)),
            "CDIR" => Some(EpicsValue::Short(if self.stat.cdir { 1 } else { 0 })),
            "TDIR" => Some(EpicsValue::Short(if self.stat.tdir { 1 } else { 0 })),
            "ATHM" => Some(EpicsValue::Short(if self.stat.athm { 1 } else { 0 })),
            "STUP" => Some(EpicsValue::Short(self.stat.stup)),
            // PID
            "PCOF" => Some(EpicsValue::Double(self.pid.pcof)),
            "ICOF" => Some(EpicsValue::Double(self.pid.icof)),
            "DCOF" => Some(EpicsValue::Double(self.pid.dcof)),
            // Display
            "EGU" => Some(EpicsValue::String(self.disp.egu.clone())),
            "PREC" => Some(EpicsValue::Short(self.disp.prec)),
            "ADEL" => Some(EpicsValue::Double(self.disp.adel)),
            "MDEL" => Some(EpicsValue::Double(self.disp.mdel)),
            // Timing
            "DLY" => Some(EpicsValue::Double(self.timing.dly)),
            "NTM" => Some(EpicsValue::Short(if self.timing.ntm { 1 } else { 0 })),
            "NTMF" => Some(EpicsValue::Double(self.timing.ntmf)),
            _ => None,
        }
    }

    fn put_field(&mut self, name: &str, value: EpicsValue) -> CaResult<()> {
        match name {
            // Position writes -- cascade and set command source
            "VAL" => {
                let v = match value { EpicsValue::Double(v) => v, _ => return Err(CaError::TypeMismatch(name.into())) };
                if self.conv.set && !self.conv.igset {
                    // SET mode: recalculate offset, signal SetPosition
                    if let Ok((dval, rval, off)) = coordinate::cascade_from_val(
                        v, self.conv.dir, self.pos.off, self.conv.foff, self.conv.mres, true, self.pos.dval,
                    ) {
                        self.pos.val = v;
                        self.pos.dval = dval;
                        self.pos.rval = rval;
                        self.pos.off = off;
                    }
                    self.last_write = Some(CommandSource::Set);
                } else {
                    if let Ok((dval, rval, off)) = coordinate::cascade_from_val(
                        v, self.conv.dir, self.pos.off, self.conv.foff, self.conv.mres, false, self.pos.dval,
                    ) {
                        self.pos.val = v;
                        self.pos.dval = dval;
                        self.pos.rval = rval;
                        self.pos.off = off;
                    }
                    self.last_write = Some(CommandSource::Val);
                }
                Ok(())
            }
            "DVAL" => {
                let v = match value { EpicsValue::Double(v) => v, _ => return Err(CaError::TypeMismatch(name.into())) };
                if self.conv.set && !self.conv.igset {
                    if let Ok((val, rval, off)) = coordinate::cascade_from_dval(
                        v, self.conv.dir, self.pos.off, self.conv.foff, self.conv.mres, true, self.pos.val,
                    ) {
                        self.pos.dval = v;
                        self.pos.val = val;
                        self.pos.rval = rval;
                        self.pos.off = off;
                    }
                    self.last_write = Some(CommandSource::Set);
                } else {
                    if let Ok((val, rval, off)) = coordinate::cascade_from_dval(
                        v, self.conv.dir, self.pos.off, self.conv.foff, self.conv.mres, false, self.pos.val,
                    ) {
                        self.pos.dval = v;
                        self.pos.val = val;
                        self.pos.rval = rval;
                        self.pos.off = off;
                    }
                    self.last_write = Some(CommandSource::Dval);
                }
                Ok(())
            }
            "RVAL" => {
                let v = match value { EpicsValue::Long(v) => v, _ => return Err(CaError::TypeMismatch(name.into())) };
                if self.conv.set && !self.conv.igset {
                    let (val, dval, off) = coordinate::cascade_from_rval(
                        v, self.conv.dir, self.pos.off, self.conv.foff, self.conv.mres, true, self.pos.val,
                    );
                    self.pos.rval = v;
                    self.pos.val = val;
                    self.pos.dval = dval;
                    self.pos.off = off;
                    self.last_write = Some(CommandSource::Set);
                } else {
                    let (val, dval, off) = coordinate::cascade_from_rval(
                        v, self.conv.dir, self.pos.off, self.conv.foff, self.conv.mres, false, self.pos.val,
                    );
                    self.pos.rval = v;
                    self.pos.val = val;
                    self.pos.dval = dval;
                    self.pos.off = off;
                    self.last_write = Some(CommandSource::Rval);
                }
                Ok(())
            }
            "RLV" => {
                let v = match value { EpicsValue::Double(v) => v, _ => return Err(CaError::TypeMismatch(name.into())) };
                self.pos.rlv = v;
                self.last_write = Some(CommandSource::Rlv);
                Ok(())
            }
            "OFF" => {
                match value {
                    EpicsValue::Double(v) => {
                        self.pos.off = v;
                        // Recalculate user coords from dial
                        self.pos.val = coordinate::dial_to_user(self.pos.dval, self.conv.dir, self.pos.off);
                        self.pos.rbv = coordinate::dial_to_user(self.pos.drbv, self.conv.dir, self.pos.off);
                        let (hlm, llm) = coordinate::dial_limits_to_user(
                            self.limits.dhlm, self.limits.dllm, self.conv.dir, self.pos.off,
                        );
                        self.limits.hlm = hlm;
                        self.limits.llm = llm;
                        Ok(())
                    }
                    _ => Err(CaError::TypeMismatch(name.into()))
                }
            }
            // Conversion
            "DIR" => {
                match value {
                    EpicsValue::Short(v) => {
                        self.conv.dir = MotorDir::from_i16(v);
                        // Recalculate user coords from dial using new direction
                        self.pos.val = coordinate::dial_to_user(self.pos.dval, self.conv.dir, self.pos.off);
                        self.pos.rbv = coordinate::dial_to_user(self.pos.drbv, self.conv.dir, self.pos.off);
                        let (hlm, llm) = coordinate::dial_limits_to_user(
                            self.limits.dhlm, self.limits.dllm, self.conv.dir, self.pos.off,
                        );
                        self.limits.hlm = hlm;
                        self.limits.llm = llm;
                        Ok(())
                    }
                    _ => Err(CaError::TypeMismatch(name.into()))
                }
            }
            "FOFF" => { match value { EpicsValue::Short(v) => { self.conv.foff = FreezeOffset::from_i16(v); Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "SET" => { match value { EpicsValue::Short(v) => { self.conv.set = v != 0; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "IGSET" => { match value { EpicsValue::Short(v) => { self.conv.igset = v != 0; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "MRES" => { match value { EpicsValue::Double(v) => { self.conv.mres = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "ERES" => { match value { EpicsValue::Double(v) => { self.conv.eres = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "SREV" => { match value { EpicsValue::Long(v) => { self.conv.srev = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "UREV" => { match value { EpicsValue::Double(v) => { self.conv.urev = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "UEIP" => { match value { EpicsValue::Short(v) => { self.conv.ueip = v != 0; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "URIP" => { match value { EpicsValue::Short(v) => { self.conv.urip = v != 0; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "RRES" => { match value { EpicsValue::Double(v) => { self.conv.rres = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            // Velocity
            "VELO" => { match value { EpicsValue::Double(v) => { self.vel.velo = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "VBAS" => { match value { EpicsValue::Double(v) => { self.vel.vbas = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "VMAX" => { match value { EpicsValue::Double(v) => { self.vel.vmax = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "S" => { match value { EpicsValue::Double(v) => { self.vel.s = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "SBAS" => { match value { EpicsValue::Double(v) => { self.vel.sbas = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "SMAX" => { match value { EpicsValue::Double(v) => { self.vel.smax = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "ACCL" => { match value { EpicsValue::Double(v) => { self.vel.accl = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "BVEL" => { match value { EpicsValue::Double(v) => { self.vel.bvel = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "BACC" => { match value { EpicsValue::Double(v) => { self.vel.bacc = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "HVEL" => { match value { EpicsValue::Double(v) => { self.vel.hvel = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "JVEL" => { match value { EpicsValue::Double(v) => { self.vel.jvel = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "JAR" => { match value { EpicsValue::Double(v) => { self.vel.jar = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "SBAK" => { match value { EpicsValue::Double(v) => { self.vel.sbak = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            // Retry
            "BDST" => { match value { EpicsValue::Double(v) => { self.retry.bdst = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "FRAC" => { match value { EpicsValue::Double(v) => { self.retry.frac = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "RDBD" => { match value { EpicsValue::Double(v) => { self.retry.rdbd = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "SPDB" => { match value { EpicsValue::Double(v) => { self.retry.spdb = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "RTRY" => { match value { EpicsValue::Short(v) => { self.retry.rtry = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "RMOD" => { match value { EpicsValue::Short(v) => { self.retry.rmod = RetryMode::from_i16(v); Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            // Limits
            "HLM" => {
                match value {
                    EpicsValue::Double(v) => {
                        self.limits.hlm = v;
                        let (dhlm, dllm) = coordinate::user_limits_to_dial(
                            self.limits.hlm, self.limits.llm, self.conv.dir, self.pos.off,
                        );
                        self.limits.dhlm = dhlm;
                        self.limits.dllm = dllm;
                        Ok(())
                    }
                    _ => Err(CaError::TypeMismatch(name.into()))
                }
            }
            "LLM" => {
                match value {
                    EpicsValue::Double(v) => {
                        self.limits.llm = v;
                        let (dhlm, dllm) = coordinate::user_limits_to_dial(
                            self.limits.hlm, self.limits.llm, self.conv.dir, self.pos.off,
                        );
                        self.limits.dhlm = dhlm;
                        self.limits.dllm = dllm;
                        Ok(())
                    }
                    _ => Err(CaError::TypeMismatch(name.into()))
                }
            }
            "DHLM" => {
                match value {
                    EpicsValue::Double(v) => {
                        self.limits.dhlm = v;
                        let (hlm, llm) = coordinate::dial_limits_to_user(
                            self.limits.dhlm, self.limits.dllm, self.conv.dir, self.pos.off,
                        );
                        self.limits.hlm = hlm;
                        self.limits.llm = llm;
                        Ok(())
                    }
                    _ => Err(CaError::TypeMismatch(name.into()))
                }
            }
            "DLLM" => {
                match value {
                    EpicsValue::Double(v) => {
                        self.limits.dllm = v;
                        let (hlm, llm) = coordinate::dial_limits_to_user(
                            self.limits.dhlm, self.limits.dllm, self.conv.dir, self.pos.off,
                        );
                        self.limits.hlm = hlm;
                        self.limits.llm = llm;
                        Ok(())
                    }
                    _ => Err(CaError::TypeMismatch(name.into()))
                }
            }
            "HLSV" => { match value { EpicsValue::Short(v) => { self.limits.hlsv = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            // Control
            "SPMG" => {
                match value {
                    EpicsValue::Short(v) => {
                        self.ctrl.spmg = SpmgMode::from_i16(v);
                        self.last_write = Some(CommandSource::Spmg);
                        Ok(())
                    }
                    _ => Err(CaError::TypeMismatch(name.into()))
                }
            }
            "STOP" => {
                match value {
                    EpicsValue::Short(v) => {
                        if v != 0 {
                            self.ctrl.stop = true;
                            self.last_write = Some(CommandSource::Stop);
                        }
                        Ok(())
                    }
                    _ => Err(CaError::TypeMismatch(name.into()))
                }
            }
            "HOMF" => {
                match value {
                    EpicsValue::Short(v) => {
                        if v != 0 {
                            self.ctrl.homf = true;
                            self.last_write = Some(CommandSource::Homf);
                        }
                        Ok(())
                    }
                    _ => Err(CaError::TypeMismatch(name.into()))
                }
            }
            "HOMR" => {
                match value {
                    EpicsValue::Short(v) => {
                        if v != 0 {
                            self.ctrl.homr = true;
                            self.last_write = Some(CommandSource::Homr);
                        }
                        Ok(())
                    }
                    _ => Err(CaError::TypeMismatch(name.into()))
                }
            }
            "JOGF" => {
                match value {
                    EpicsValue::Short(v) => {
                        self.ctrl.jogf = v != 0;
                        self.last_write = Some(CommandSource::Jogf);
                        Ok(())
                    }
                    _ => Err(CaError::TypeMismatch(name.into()))
                }
            }
            "JOGR" => {
                match value {
                    EpicsValue::Short(v) => {
                        self.ctrl.jogr = v != 0;
                        self.last_write = Some(CommandSource::Jogr);
                        Ok(())
                    }
                    _ => Err(CaError::TypeMismatch(name.into()))
                }
            }
            "TWF" => {
                match value {
                    EpicsValue::Short(v) => {
                        if v != 0 {
                            self.ctrl.twf = true;
                            self.last_write = Some(CommandSource::Twf);
                        }
                        Ok(())
                    }
                    _ => Err(CaError::TypeMismatch(name.into()))
                }
            }
            "TWR" => {
                match value {
                    EpicsValue::Short(v) => {
                        if v != 0 {
                            self.ctrl.twr = true;
                            self.last_write = Some(CommandSource::Twr);
                        }
                        Ok(())
                    }
                    _ => Err(CaError::TypeMismatch(name.into()))
                }
            }
            "TWV" => { match value { EpicsValue::Double(v) => { self.ctrl.twv = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "CNEN" => {
                match value {
                    EpicsValue::Short(v) => {
                        self.ctrl.cnen = v != 0;
                        self.last_write = Some(CommandSource::Cnen);
                        Ok(())
                    }
                    _ => Err(CaError::TypeMismatch(name.into()))
                }
            }
            // Status (read-only handled by validate_put)
            "STUP" => { match value { EpicsValue::Short(v) => { self.stat.stup = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            // PID
            "PCOF" => { match value { EpicsValue::Double(v) => { self.pid.pcof = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "ICOF" => { match value { EpicsValue::Double(v) => { self.pid.icof = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "DCOF" => { match value { EpicsValue::Double(v) => { self.pid.dcof = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            // Display
            "EGU" => { match value { EpicsValue::String(v) => { self.disp.egu = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "PREC" => { match value { EpicsValue::Short(v) => { self.disp.prec = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "ADEL" => { match value { EpicsValue::Double(v) => { self.disp.adel = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "MDEL" => { match value { EpicsValue::Double(v) => { self.disp.mdel = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            // Timing
            "DLY" => { match value { EpicsValue::Double(v) => { self.timing.dly = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "NTM" => { match value { EpicsValue::Short(v) => { self.timing.ntm = v != 0; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            "NTMF" => { match value { EpicsValue::Double(v) => { self.timing.ntmf = v; Ok(()) } _ => Err(CaError::TypeMismatch(name.into())) } }
            // Sync
            "SYNC" => {
                self.last_write = Some(CommandSource::Sync);
                Ok(())
            }
            _ => Err(CaError::FieldNotFound(name.into())),
        }
    }

    fn field_list(&self) -> &'static [FieldDesc] {
        FIELDS
    }

    fn primary_field(&self) -> &'static str {
        "VAL"
    }

}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_values() {
        let rec = MotorRecord::new();
        assert_eq!(rec.pos.val, 0.0);
        assert!(rec.stat.dmov);
        assert!(!rec.stat.movn);
        assert_eq!(rec.stat.phase, MotionPhase::Idle);
        assert_eq!(rec.stat.mip, MipFlags::empty());
        assert_eq!(rec.ctrl.spmg, SpmgMode::Go);
        assert_eq!(rec.conv.mres, 1.0);
        assert_eq!(rec.vel.velo, 1.0);
        assert_eq!(rec.vel.accl, 0.5);
        assert_eq!(rec.retry.rtry, 10);
        assert!(rec.limits.lvio); // default true (no limits set)
    }

    #[test]
    fn test_record_type() {
        let rec = MotorRecord::new();
        assert_eq!(rec.record_type(), "motor");
    }

    #[test]
    fn test_field_roundtrip_double() {
        let mut rec = MotorRecord::new();
        rec.put_field("VAL", EpicsValue::Double(42.0)).unwrap();
        assert_eq!(rec.get_field("VAL"), Some(EpicsValue::Double(42.0)));
    }

    #[test]
    fn test_field_roundtrip_short() {
        let mut rec = MotorRecord::new();
        rec.put_field("PREC", EpicsValue::Short(3)).unwrap();
        assert_eq!(rec.get_field("PREC"), Some(EpicsValue::Short(3)));
    }

    #[test]
    fn test_field_roundtrip_string() {
        let mut rec = MotorRecord::new();
        rec.put_field("EGU", EpicsValue::String("mm".into())).unwrap();
        assert_eq!(rec.get_field("EGU"), Some(EpicsValue::String("mm".into())));
    }

    #[test]
    fn test_val_cascades_to_dval_rval() {
        let mut rec = MotorRecord::new();
        rec.conv.mres = 0.01;
        rec.put_field("VAL", EpicsValue::Double(10.0)).unwrap();
        assert_eq!(rec.pos.dval, 10.0);
        assert_eq!(rec.pos.rval, 1000);
    }

    #[test]
    fn test_dval_cascades_to_val_rval() {
        let mut rec = MotorRecord::new();
        rec.conv.mres = 0.01;
        rec.put_field("DVAL", EpicsValue::Double(5.0)).unwrap();
        assert_eq!(rec.pos.val, 5.0);
        assert_eq!(rec.pos.rval, 500);
    }

    #[test]
    fn test_rval_cascades_to_val_dval() {
        let mut rec = MotorRecord::new();
        rec.conv.mres = 0.01;
        rec.put_field("RVAL", EpicsValue::Long(1000)).unwrap();
        assert_eq!(rec.pos.dval, 10.0);
        assert_eq!(rec.pos.val, 10.0);
    }

    #[test]
    fn test_set_mode_updates_offset() {
        let mut rec = MotorRecord::new();
        rec.conv.mres = 0.01;
        rec.pos.dval = 5.0;
        rec.conv.set = true;
        rec.put_field("VAL", EpicsValue::Double(100.0)).unwrap();
        // Offset should be updated, DVAL unchanged
        assert_eq!(rec.pos.dval, 5.0);
        assert_eq!(rec.pos.off, 95.0); // 100 - 1*5
        // SET mode produces SetPosition command via process path
        assert_eq!(rec.last_write, Some(CommandSource::Set));
    }

    #[test]
    fn test_type_mismatch() {
        let mut rec = MotorRecord::new();
        let result = rec.put_field("VAL", EpicsValue::String("bad".into()));
        assert!(result.is_err());
    }

    #[test]
    fn test_unknown_field() {
        let mut rec = MotorRecord::new();
        let result = rec.put_field("NONEXIST", EpicsValue::Double(0.0));
        assert!(result.is_err());
    }

    #[test]
    fn test_hlm_cascades_to_dhlm() {
        let mut rec = MotorRecord::new();
        rec.put_field("HLM", EpicsValue::Double(100.0)).unwrap();
        rec.put_field("LLM", EpicsValue::Double(-100.0)).unwrap();
        assert_eq!(rec.limits.dhlm, 100.0);
        assert_eq!(rec.limits.dllm, -100.0);
    }

    #[test]
    fn test_dir_neg_limit_mapping() {
        let mut rec = MotorRecord::new();
        rec.conv.dir = MotorDir::Neg;
        rec.put_field("HLM", EpicsValue::Double(100.0)).unwrap();
        rec.put_field("LLM", EpicsValue::Double(-100.0)).unwrap();
        // DIR=Neg: user 100 -> dial -100, user -100 -> dial 100
        assert_eq!(rec.limits.dhlm, 100.0);
        assert_eq!(rec.limits.dllm, -100.0);
    }

    #[test]
    fn test_spmg_blocks_commands() {
        let mut rec = MotorRecord::new();
        rec.ctrl.spmg = SpmgMode::Stop;
        assert!(!rec.can_accept_command());
        rec.ctrl.spmg = SpmgMode::Pause;
        assert!(!rec.can_accept_command());
        rec.ctrl.spmg = SpmgMode::Go;
        assert!(rec.can_accept_command());
        rec.ctrl.spmg = SpmgMode::Move;
        assert!(rec.can_accept_command());
    }

    #[test]
    fn test_compute_dmov() {
        let mut rec = MotorRecord::new();
        rec.stat.msta = MstaFlags::DONE;
        rec.stat.phase = MotionPhase::Idle;
        assert!(rec.compute_dmov());

        rec.stat.msta = MstaFlags::MOVING;
        assert!(!rec.compute_dmov());

        rec.stat.msta = MstaFlags::DONE;
        rec.stat.phase = MotionPhase::MainMove;
        assert!(!rec.compute_dmov());
    }

    #[test]
    fn test_process_motor_info() {
        let mut rec = MotorRecord::new();
        rec.conv.mres = 0.001;
        let status = asyn_rs::interfaces::motor::MotorStatus {
            position: 10.0,
            encoder_position: 10.001,
            done: true,
            moving: false,
            high_limit: false,
            low_limit: false,
            home: false,
            powered: true,
            problem: false,
        };
        rec.process_motor_info(&status);
        assert_eq!(rec.pos.rmp, 10000);
        assert_eq!(rec.pos.drbv, 10.0);
        assert_eq!(rec.pos.rbv, 10.0);
        assert!(!rec.stat.movn);
        assert!(rec.stat.msta.contains(MstaFlags::DONE));
    }

    #[test]
    fn test_sync_positions() {
        let mut rec = MotorRecord::new();
        rec.pos.drbv = 5.0;
        rec.pos.rbv = 5.0;
        rec.pos.rrbv = 500;
        rec.sync_positions();
        assert_eq!(rec.pos.dval, 5.0);
        assert_eq!(rec.pos.val, 5.0);
        assert_eq!(rec.pos.rval, 500);
        assert_eq!(rec.pos.diff, 0.0);
    }

    #[test]
    fn test_soft_limit_rejects_move() {
        let mut rec = MotorRecord::new();
        rec.conv.mres = 0.01;
        rec.limits.dhlm = 100.0;
        rec.limits.dllm = -100.0;
        rec.limits.lvio = false;
        rec.stat.msta = MstaFlags::DONE;

        // Try to move beyond limits
        rec.pos.dval = 200.0;
        let effects = rec.plan_motion(CommandSource::Val);
        assert!(rec.limits.lvio);
        assert!(effects.commands.is_empty());
    }

    #[test]
    fn test_absolute_move_sets_dmov_false() {
        let mut rec = MotorRecord::new();
        rec.conv.mres = 0.01;
        rec.limits.dhlm = 100.0;
        rec.limits.dllm = -100.0;
        rec.stat.msta = MstaFlags::DONE;
        rec.pos.dval = 50.0;

        let effects = rec.plan_motion(CommandSource::Val);
        assert!(!rec.stat.dmov);
        assert_eq!(rec.stat.phase, MotionPhase::MainMove);
        assert_eq!(effects.commands.len(), 1);
        assert!(effects.request_poll);
        assert!(matches!(effects.commands[0], MotorCommand::MoveAbsolute { .. }));
    }

    #[test]
    fn test_stop_during_move() {
        let mut rec = MotorRecord::new();
        rec.stat.phase = MotionPhase::MainMove;
        rec.stat.mip = MipFlags::MOVE;
        rec.stat.dmov = false;
        rec.pos.rbv = 25.0;
        rec.pos.drbv = 25.0;
        rec.pos.rrbv = 2500;

        let effects = rec.plan_motion(CommandSource::Stop);
        assert!(rec.stat.mip.contains(MipFlags::STOP));
        assert_eq!(effects.commands.len(), 1);
        assert!(matches!(effects.commands[0], MotorCommand::Stop { .. }));
        // VAL synced to RBV
        assert_eq!(rec.pos.val, 25.0);
    }

    #[test]
    fn test_jog_start_stop() {
        let mut rec = MotorRecord::new();
        rec.conv.mres = 0.01;
        rec.stat.msta = MstaFlags::DONE;

        // Start jog forward
        rec.ctrl.jogf = true;
        let effects = rec.plan_motion(CommandSource::Jogf);
        assert!(!rec.stat.dmov);
        assert_eq!(rec.stat.phase, MotionPhase::Jog);
        assert!(rec.stat.mip.contains(MipFlags::JOGF));
        assert!(matches!(effects.commands[0], MotorCommand::MoveVelocity { direction: true, .. }));

        // Stop jog
        rec.ctrl.jogf = false;
        let effects = rec.plan_motion(CommandSource::Jogf);
        assert!(rec.stat.mip.contains(MipFlags::JOG_STOP));
        assert!(matches!(effects.commands[0], MotorCommand::Stop { .. }));
    }

    #[test]
    fn test_home_forward() {
        let mut rec = MotorRecord::new();
        rec.stat.msta = MstaFlags::DONE;

        rec.ctrl.homf = true;
        let effects = rec.plan_motion(CommandSource::Homf);
        assert!(!rec.stat.dmov);
        assert_eq!(rec.stat.phase, MotionPhase::Homing);
        assert!(rec.stat.mip.contains(MipFlags::HOMF));
        assert!(!rec.ctrl.homf); // pulse cleared
        assert!(matches!(effects.commands[0], MotorCommand::Home { forward: true, .. }));
    }

    #[test]
    fn test_tweak_forward() {
        let mut rec = MotorRecord::new();
        rec.conv.mres = 0.01;
        rec.limits.dhlm = 100.0;
        rec.limits.dllm = -100.0;
        rec.stat.msta = MstaFlags::DONE;
        rec.ctrl.twv = 5.0;
        rec.pos.val = 10.0;
        rec.pos.dval = 10.0;

        rec.ctrl.twf = true;
        let effects = rec.plan_motion(CommandSource::Twf);
        assert_eq!(rec.pos.val, 15.0); // 10 + 5
        assert!(!rec.ctrl.twf); // pulse cleared
        assert!(!effects.commands.is_empty());
    }

    #[test]
    fn test_should_fire_forward_link() {
        let mut rec = MotorRecord::new();
        assert!(rec.should_fire_forward_link());

        rec.suppress_flnk = true;
        assert!(!rec.should_fire_forward_link());
    }

    #[test]
    fn test_field_list_coverage() {
        let rec = MotorRecord::new();
        let fields = rec.field_list();
        // All fields in the list should be gettable
        for fd in fields {
            assert!(
                rec.get_field(fd.name).is_some(),
                "field {} not gettable",
                fd.name
            );
        }
    }

    #[test]
    fn test_dly_delays_finalization() {
        let mut rec = MotorRecord::new();
        rec.timing.dly = 1.0;
        rec.stat.msta = MstaFlags::DONE;
        rec.stat.phase = MotionPhase::MainMove;
        rec.stat.dmov = false; // motion in progress
        rec.retry.rdbd = 0.0; // no retry

        let effects = rec.check_completion();
        assert_eq!(rec.stat.phase, MotionPhase::DelayWait);
        assert!(effects.schedule_delay.is_some());
        assert!(!rec.stat.dmov); // still false during delay
    }

    #[test]
    fn test_retry_on_position_error() {
        let mut rec = MotorRecord::new();
        rec.stat.msta = MstaFlags::DONE;
        rec.stat.phase = MotionPhase::MainMove;
        rec.retry.rdbd = 0.1;
        rec.retry.rtry = 3;
        rec.pos.dval = 10.0;
        rec.pos.drbv = 9.5; // error = 0.5 > rdbd

        let effects = rec.check_completion();
        assert_eq!(rec.stat.phase, MotionPhase::Retry);
        assert_eq!(rec.retry.rcnt, 1);
        assert!(!effects.commands.is_empty());
    }

    #[test]
    fn test_miss_when_retries_exhausted() {
        let mut rec = MotorRecord::new();
        rec.stat.msta = MstaFlags::DONE;
        rec.stat.phase = MotionPhase::MainMove;
        rec.retry.rdbd = 0.1;
        rec.retry.rtry = 3;
        rec.retry.rcnt = 3; // exhausted
        rec.pos.dval = 10.0;
        rec.pos.drbv = 9.5;

        let _effects = rec.check_completion();
        assert!(rec.retry.miss);
        // Should finalize (or delay)
        assert_eq!(rec.stat.phase, MotionPhase::Idle);
    }

    #[test]
    fn test_ntm_retarget_direction_change() {
        let mut rec = MotorRecord::new();
        rec.timing.ntm = true;
        rec.timing.ntmf = 2.0;
        rec.retry.bdst = 0.0;
        rec.retry.rdbd = 0.0;
        rec.internal.ldvl = 10.0;
        rec.pos.drbv = 5.0;

        // Same direction, farther -> ExtendMove
        assert_eq!(rec.handle_retarget(15.0), RetargetAction::ExtendMove);

        // Opposite direction -> StopAndReplan
        assert_eq!(rec.handle_retarget(-5.0), RetargetAction::StopAndReplan);
    }

    #[test]
    fn test_ueip_eres_readback() {
        let mut rec = MotorRecord::new();
        rec.conv.mres = 0.001;
        rec.conv.ueip = true;
        rec.conv.eres = 0.002;
        let status = asyn_rs::interfaces::motor::MotorStatus {
            position: 10.0,
            encoder_position: 10.0,
            done: true,
            ..Default::default()
        };
        rec.process_motor_info(&status);
        // REP = round(10.0 / 0.002) = 5000
        // RRBV = REP = 5000 (UEIP=true)
        // DRBV = 5000 * 0.002 = 10.0
        assert_eq!(rec.pos.rep, 5000);
        assert_eq!(rec.pos.rrbv, 5000);
        assert_eq!(rec.pos.drbv, 10.0);
    }

    #[test]
    fn test_ueip_eres_nan_fallback_to_mres() {
        let mut rec = MotorRecord::new();
        rec.conv.mres = 0.001;
        rec.conv.ueip = true;
        rec.conv.eres = f64::NAN;
        let status = asyn_rs::interfaces::motor::MotorStatus {
            position: 10.0,
            encoder_position: 10.0,
            done: true,
            ..Default::default()
        };
        rec.process_motor_info(&status);
        // Should fall back to MRES for both REP and DRBV
        assert_eq!(rec.pos.rep, 10000);
        assert_eq!(rec.pos.drbv, 10.0);
    }

    #[test]
    fn test_ueip_false_uses_motor_position() {
        let mut rec = MotorRecord::new();
        rec.conv.mres = 0.001;
        rec.conv.ueip = false;
        rec.conv.eres = 0.002;
        let status = asyn_rs::interfaces::motor::MotorStatus {
            position: 10.0,
            encoder_position: 20.0,
            done: true,
            ..Default::default()
        };
        rec.process_motor_info(&status);
        // UEIP=false: uses RMP path with MRES
        assert_eq!(rec.pos.rmp, 10000);
        assert_eq!(rec.pos.rrbv, 10000); // RMP, not REP
        assert_eq!(rec.pos.drbv, 10.0);  // rrbv * mres
    }

    #[test]
    fn test_stup_triggers_status_refresh() {
        let mut rec = MotorRecord::new();
        rec.stat.stup = 1;
        let effects = rec.do_process();
        assert!(effects.status_refresh);
        assert_eq!(rec.stat.stup, 0);
        assert!(effects.commands.is_empty());
    }

    #[test]
    fn test_hls_blocks_positive_move() {
        let mut rec = MotorRecord::new();
        rec.conv.mres = 0.01;
        rec.limits.dhlm = 100.0;
        rec.limits.dllm = -100.0;
        rec.limits.hls = true;
        rec.stat.msta = MstaFlags::DONE;
        rec.pos.dval = 50.0;

        let effects = rec.plan_motion(CommandSource::Val);
        assert!(effects.commands.is_empty());
        assert!(rec.stat.dmov); // no motion started
    }

    #[test]
    fn test_hls_allows_negative_move() {
        let mut rec = MotorRecord::new();
        rec.conv.mres = 0.01;
        rec.limits.dhlm = 100.0;
        rec.limits.dllm = -100.0;
        rec.limits.hls = true;
        rec.stat.msta = MstaFlags::DONE;
        rec.pos.dval = -10.0; // negative direction

        let effects = rec.plan_motion(CommandSource::Val);
        assert!(!effects.commands.is_empty());
        assert!(!rec.stat.dmov);
    }

    #[test]
    fn test_lls_blocks_negative_move() {
        let mut rec = MotorRecord::new();
        rec.conv.mres = 0.01;
        rec.limits.dhlm = 100.0;
        rec.limits.dllm = -100.0;
        rec.limits.lls = true;
        rec.stat.msta = MstaFlags::DONE;
        rec.pos.dval = -50.0;

        let effects = rec.plan_motion(CommandSource::Val);
        assert!(effects.commands.is_empty());
    }

    #[test]
    fn test_lls_allows_positive_move() {
        let mut rec = MotorRecord::new();
        rec.conv.mres = 0.01;
        rec.limits.dhlm = 100.0;
        rec.limits.dllm = -100.0;
        rec.limits.lls = true;
        rec.stat.msta = MstaFlags::DONE;
        rec.pos.dval = 10.0;

        let effects = rec.plan_motion(CommandSource::Val);
        assert!(!effects.commands.is_empty());
    }

    #[test]
    fn test_both_limits_block_all_moves() {
        let mut rec = MotorRecord::new();
        rec.conv.mres = 0.01;
        rec.limits.dhlm = 100.0;
        rec.limits.dllm = -100.0;
        rec.limits.hls = true;
        rec.limits.lls = true;
        rec.stat.msta = MstaFlags::DONE;

        rec.pos.dval = 10.0;
        let effects = rec.plan_motion(CommandSource::Val);
        assert!(effects.commands.is_empty());

        rec.pos.dval = -10.0;
        let effects = rec.plan_motion(CommandSource::Val);
        assert!(effects.commands.is_empty());
    }

    #[test]
    fn test_hls_blocks_forward_jog() {
        let mut rec = MotorRecord::new();
        rec.limits.hls = true;
        rec.ctrl.jogf = true;
        let effects = rec.plan_motion(CommandSource::Jogf);
        assert!(effects.commands.is_empty());
        assert!(rec.stat.dmov);
    }

    #[test]
    fn test_cnen_emits_set_closed_loop() {
        let mut rec = MotorRecord::new();
        rec.ctrl.cnen = true;
        let effects = rec.plan_motion(CommandSource::Cnen);
        assert_eq!(effects.commands.len(), 1);
        assert!(matches!(
            effects.commands[0],
            MotorCommand::SetClosedLoop { enable: true }
        ));
    }

    #[test]
    fn test_cnen_false_emits_disable() {
        let mut rec = MotorRecord::new();
        rec.ctrl.cnen = false;
        let effects = rec.plan_motion(CommandSource::Cnen);
        assert_eq!(effects.commands.len(), 1);
        assert!(matches!(
            effects.commands[0],
            MotorCommand::SetClosedLoop { enable: false }
        ));
    }

    #[test]
    fn test_spmg_stop_finalizes() {
        let mut rec = MotorRecord::new();
        rec.stat.phase = MotionPhase::MainMove;
        rec.stat.mip = MipFlags::MOVE;
        rec.stat.dmov = false;
        rec.pos.rbv = 25.0;
        rec.pos.drbv = 25.0;
        rec.pos.rrbv = 2500;

        rec.ctrl.spmg = SpmgMode::Stop;
        let effects = rec.plan_motion(CommandSource::Spmg);
        assert!(rec.stat.dmov); // finalized
        assert_eq!(rec.stat.phase, MotionPhase::Idle);
        assert_eq!(rec.pos.val, 25.0); // synced
        assert!(matches!(effects.commands[0], MotorCommand::Stop { .. }));
    }
}
