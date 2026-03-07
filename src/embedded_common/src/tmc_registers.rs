use core::{f32, fmt::Debug, ops::Mul};

use bitfield_struct::bitfield;

/// Asume the highest microstepping resolution
const MICROSTEPS_PER_STEP: u32 = 256;
/// Number of microsteps per motor revolution
const MICROSTEPS_PER_REV: u32 = 200 * MICROSTEPS_PER_STEP;

pub trait Register: From<u32> + Into<u32> + Debug {
    const ADDRESS: u8;
}

#[derive(Debug, PartialEq, PartialOrd)]
pub struct TmcPosition(pub f32);

impl TmcPosition {
    const CONVERSION_FACTOR: f32 = MICROSTEPS_PER_REV as f32 / (2. * f32::consts::PI);

    pub fn from_bits(value: u32) -> Self {
        Self(value as i32 as f32 / Self::CONVERSION_FACTOR)
    }

    pub fn bits(&self) -> u32 {
        (self.0 * Self::CONVERSION_FACTOR) as i32 as u32
    }
}

impl Mul<f32> for TmcPosition {
    type Output = TmcPosition;

    fn mul(self, rhs: f32) -> Self::Output {
        TmcPosition(self.0 * rhs)
    }
}

#[derive(Debug, PartialEq, PartialOrd)]
pub struct TmcVelocity(f32);

impl TmcVelocity {
    const CONVERSION_FACTOR: f32 =
        MICROSTEPS_PER_REV as f32 / (2. * f32::consts::PI) / 12_000_000. * 2. * (1 << 23) as f32;

    pub fn from_bits(value: u32) -> Self {
        Self(value as i32 as f32 * Self::CONVERSION_FACTOR)
    }

    pub fn bits(&self) -> u32 {
        (self.0 * Self::CONVERSION_FACTOR) as i32 as u32
    }
}

#[derive(Debug, PartialEq, PartialOrd)]
pub struct TmcAcceleration(f32);

impl TmcAcceleration {
    const CONVERSION_FACTOR: f32 =
        MICROSTEPS_PER_REV as f32 / (2. * f32::consts::PI) / (12_000_000.) / (12_000_000.)
            * (512. * 256.)
            * (1 << 24) as f32;

    pub fn from_bits(value: u32) -> Self {
        Self(value as i32 as f32 * Self::CONVERSION_FACTOR)
    }

    pub fn bits(&self) -> u32 {
        (self.0 * Self::CONVERSION_FACTOR) as i32 as u32
    }
}

#[derive(Debug)]
pub struct TmcUnitless(i32);

impl TmcUnitless {
    pub fn from_bits(value: u32) -> Self {
        Self(value as i32)
    }

    pub fn bits(&self) -> u32 {
        self.0 as u32
    }
}

pub trait UnitsExt {
    fn rads(self) -> TmcPosition;
    fn rps(self) -> TmcVelocity;
    fn rps2(self) -> TmcAcceleration;
}

impl UnitsExt for f32 {
    fn rads(self) -> TmcPosition {
        TmcPosition(self)
    }

    fn rps(self) -> TmcVelocity {
        TmcVelocity(self)
    }

    fn rps2(self) -> TmcAcceleration {
        TmcAcceleration(self)
    }
}

pub trait UnitlessExt {
    fn ul(self) -> TmcUnitless;
}

impl UnitlessExt for i32 {
    fn ul(self) -> TmcUnitless {
        TmcUnitless(self)
    }
}

macro_rules! motion_register {
    ($name:ident, $type:ty, $addr:literal, $doc:literal) => {
        #[doc = $doc]
        #[derive(Debug)]
        pub struct $name(pub $type);
        impl Register for $name {
            const ADDRESS: u8 = $addr;
        }
        impl From<u32> for $name {
            fn from(value: u32) -> Self {
                Self {
                    0: <$type>::from_bits(value),
                }
            }
        }
        impl Into<u32> for $name {
            fn into(self) -> u32 {
                self.0.bits()
            }
        }
    };
}

/// Global Configuration Flags
#[bitfield(u32)]
pub struct GConf {
    /// Zero crossing recalibration during driver disable (via DRV_ENN or via TOFF setting)
    #[bits(1)]
    pub recalibrate: bool,

    /// Timeout for step execution until standstill detection:
    /// - Short time: 2^18 clocks
    /// - Normal time: 2^20 clocks
    #[bits(1)]
    pub faststandstill: bool,

    /// StealthChop voltage PWM mode enabled (depending on velocity thresholds). Switch from off to on state while in
    /// stand-still and at IHOLD=nominal IRUN current, only.
    #[bits(1)]
    pub en_pwm_mode: bool,

    /// Enable step input filtering for StealthChop optimization with external step source
    #[bits(1)]
    multistep_filt: bool,

    /// Inverse motor direction
    #[bits(1)]
    pub shaft: bool,

    #[bits(9)]
    __: u32,

    /// - false: Hysteresis for step frequency comparison is 1/16
    /// - true: Hysteresis for step frequency comparison is 1/32
    #[bits(1)]
    small_hysteresis: bool,

    /// - false: Normal operation
    /// - true: Emergency stop, ENCA_DCIN stops the sequencer when tied high (no steps become executed by the sequencer,
    /// motor goes to standstill state).
    #[bits(1)]
    stop_enable: bool,

    /// - false: Normal operation
    /// - true: Motor coil currents and polarity directly programmed via serial interface: Register XTARGET (0x2D)
    /// specifies signed coil A current (bits 8..0) and coil B current (bits 24..16). In this mode, the current is
    /// scaled by IHOLD setting. Velocity based current regulation of StealthChop is not available in this mode. The
    /// automatic StealthChop current regulation will work only for low stepper motor velocities.
    #[bits(1)]
    direct_mode: bool,

    #[bits(15)]
    __: u32,
}

impl Register for GConf {
    const ADDRESS: u8 = 0x00;
}

/// Global status flags
#[bitfield(u32)]
pub struct GStat {
    /// Indicates that the IC has been reset. All registers have been cleared to reset values.
    #[bits(1)]
    pub reset: bool,

    /// Indicates, that the driver has been shut down due to overtemperature or short circuit detection. Read DRV_STATUS
    /// for details. The flag can only be cleared when the temperature is below the limit again.
    #[bits(1)]
    pub drv_err: bool,

    /// Indicates an undervoltage on the charge pump. The driver is disabled during undervoltage. This flag is latched
    /// for information.
    #[bits(1)]
    pub uv_cp: bool,

    #[bits(29)]
    __: u32,
}

impl Register for GStat {
    const ADDRESS: u8 = 0x01;
}

/// Short Configuration
#[bitfield(u32)]
pub struct ShortConf {
    /// Short to VS detector level for lowside FETs. Checks for voltage drop in LS MOSFET and sense resistor.
    /// 
    /// 4(highest sensitivity) … 15 (lowest sensitivity)
    /// 
    /// Hint: Settings from 1 to 3 will trigger during normal operation due to voltage drop on sense resistor.
    #[bits(4)]
    pub s2vs_level: u8,

    #[bits(4)]
    __: u32,

    /// Short to GND detector level for highside FETs. Checks for voltage drop on high side MOSFET.
    /// 
    /// 2 (highest sensitivity) … 15 (lowest sensitivity)
    /// 
    /// Attention: Settings below 6 not recommended at >52V operation – false detection might result.
    #[bits(4)]
    pub s2g_level: u8,

    #[bits(4)]
    __: u32,

    /// Spike filtering bandwidth for short detection
    /// 
    /// 0 (lowest, 100ns), 1 (1μs), 2 (2μs) 3 (3μs)
    /// 
    /// Hint: A good PCB layout will allow using setting 0. Increase value, if erroneous short detection occurs.
    #[bits(2)]
    pub short_filter: u8,

    /// Short detection delay
    /// 
    /// 0=750ns: normal, 1=1500ns: high
    /// 
    /// The short detection delay shall cover the bridge switching time. 0 will work for most applications.
    #[bits(1)]
    pub short_delay: bool,

    #[bits(13)]
    __: u32,
}

impl Register for ShortConf {
    const ADDRESS: u8 = 0x09;
}

/// Driver Configuration
#[bitfield(u32)]
pub struct DrvConf {
    /// Break-Before make delay
    /// 
    /// 0=shortest (100ns) … 16 (200ns) … 24=longest (375ns)
    /// 
    /// >24 not recommended, use BBMCLKS instead
    /// 
    /// Hint: Choose the lowest setting safely covering the switching event to avoid bridge cross-conduction. Add
    /// roughly 30% of reserve.
    #[bits(5)]
    pub bbm_time: u8,

    #[bits(3)]
    __: u32,

    /// 0..15: Digital BBM time in clock cycles (typ. 83ns).
    /// 
    /// The longer setting rules (BBMTIME vs. BBMCLKS).
    #[bits(4)]
    pub bbm_clks: u8,

    #[bits(4)]
    __: u32,

    /// Selection of over temperature level for bridge disable, switch on after cool down to 120°C / OTPW level.
    /// 
    /// - 0: 150°C
    /// - 1: 143°C
    /// - 2: 136°C (not recommended when VSA > 24V)
    /// - 3: 120°C (not recommended, no hysteresis)
    /// 
    /// Hint: Adapt overtemperature threshold as required to protect the MOSFETs or other components on the PCB.
    #[bits(2)]
    pub ot_select: u8,

    /// Selection of gate driver current. Adapts the gate driver current to the gate charge of the external MOSFETs.
    /// 
    /// - 0: weak
    /// - 1: weak+TC (medium above OTPW level)
    /// - 2: medium
    /// - 3: strong
    /// 
    /// Hint: Choose the lowest setting giving slopes <100ns.
    #[bits(2)]
    pub drv_strength: u8,

    /// Filter time constant of sense amplifier to suppress ringing and coupling from second coil operation
    /// 
    /// - 0: low – 100ns
    /// - 1: – 200ns
    /// - 2: – 300ns
    /// - 3: high– 400ns
    /// 
    /// Hint: Increase setting if motor chopper noise occurs due to cross-coupling of both coils.
    #[bits(2)]
    pub filt_isense: u8,

    #[bits(10)]
    __: u32,
}

impl Register for DrvConf {
    const ADDRESS: u8 = 0x0A;
}

motion_register!(GlobalScalar, TmcUnitless, 0x0B, "
Global scaling of Motor current. This value is multiplied to the current scaling to adapt a drive to a certain motor
type. This value should be chosen before tuning other settings because it also influences chopper hysteresis.

- 0: Full Scale (or write 256)
- 1 … 31: Not allowed for operation
- 32 … 255: 32/256 … 255/256 of maximum current.

Hint: Values >128 recommended for best results
");

/// Driver current control
#[bitfield(u32)]
pub struct IHoldIRun {
    /// Standstill current (0=1/32…31=32/32)
    /// 
    /// In combination with StealthChop mode, setting IHOLD=0 allows to choose
    /// freewheeling or coil short circuit for motor stand still.
    #[bits(5)]
    pub ihold: u8,

    #[bits(3)]
    __: u32,

    /// Motor run current (0=1/32…31=32/32)
    /// 
    /// Hint: Choose sense resistors in a way, that normal IRUN is 16 to 31 for best microstep performance.
    #[bits(5)]
    pub irun: u8,

    #[bits(3)]
    __: u32,

    /// Controls the number of clock cycles for motor power down after a motion as soon as standstill is detected
    /// (stst=1) and TPOWERDOWN has expired. The smooth transition avoids a motor jerk upon power down.
    /// 
    /// - 0: instant power down
    /// - 1..15: Delay per current reduction step in multiple of 2^18 clocks
    #[bits(4)]
    pub ihold_delay: u8,

    #[bits(12)]
    __: u32,
}

impl Register for IHoldIRun {
    const ADDRESS: u8 = 0x10;
}

motion_register!(TPowerDown, TmcUnitless, 0x11, "
sets the delay time after stand still (stst) of the motor to motor current power down. Time range is about 0 to 4
seconds.

Attention: A minimum setting of 2 is required to allow automatic tuning of StealthChop PWM_OFS_AUTO.
");
motion_register!(TPwmThrs, TmcVelocity, 0x13, "
This is the upper velocity for StealthChop voltage PWM mode. TSTEP ≥ TPWMTHRS

- StealthChop PWM mode is enabled, if configured
- DcStep is disabled
");
motion_register!(TCoolThrs, TmcVelocity, 0x14, "
This is the lower threshold velocity for switching on smart energy CoolStep and StallGuard feature. (unsigned)

Set this parameter to disable CoolStep at low speeds, where it cannot work reliably. The stop on stall function (enable
with sg_stop when using internal motion controller) and the stall output signal become enabled when exceeding this
velocity. In non-DcStep mode, it becomes disabled again once the velocity falls below this threshold.

TCOOLTHRS ≥ TSTEP ≥ THIGH:
- CoolStep is enabled, if configured
- StealthChop voltage PWM mode is disabled

TCOOLTHRS ≥ TSTEP
- Stop on stall is enabled, if configured
- Stall output signal (DIAG0/1) is enabled, if configured
");
motion_register!(THigh, TmcVelocity, 0x15, "
This velocity setting allows velocity dependent switching into a different chopper mode and fullstepping to maximize
torque. (unsigned)

The stall detection feature becomes switched off for 2-3 electrical periods whenever passing THIGH threshold to
compensate for the effect of switching modes.

TSTEP ≤ THIGH:
- CoolStep is disabled (motor runs with normal current scale)
- StealthChop voltage PWM mode is disabled
- If vhighchm is set, the chopper switches to chm=1 with TFD=0 (constant off time with slow decay, only).
- If vhighfs is set, the motor operates in fullstep mode, and the stall detection becomes switched over to DcStep stall
detection.
");

motion_register!(RampMode, TmcUnitless, 0x20, "
- 0: Positioning mode (using all A, D and V parameters)
- 1: Velocity mode to positive VMAX (using AMAX acceleration)
- 2: Velocity mode to negative VMAX (using AMAX acceleration)
- 3: Hold mode (velocity remains unchanged, unless stop event occurs)
");
motion_register!(XActual, TmcPosition, 0x21, "
Actual motor position (signed)

Hint: This value normally should only be modified, when homing the drive. In positioning mode, modifying the register
content will start a motion.
");
motion_register!(VActual, TmcVelocity, 0x22, "
Actual motor velocity from ramp generator (signed)

The sign matches the motion direction. A negative sign means motion to lower XACTUAL.
");
motion_register!(VStart, TmcVelocity, 0x23, "
Motor start velocity (unsigned)

For universal use, set VSTOP ≥ VSTART. This is not required if the motion distance is sufficient to ensure deceleration
from VSTART to VSTOP.
");

motion_register!(A1, TmcAcceleration, 0x24, "
First acceleration between VSTART and V1 (unsigned)
");
motion_register!(V1, TmcVelocity, 0x25, "
First acceleration / deceleration phase threshold velocity (unsigned)

0: Disables A1 and D1 phase, use AMAX, DMAX only
");
motion_register!(AMax, TmcAcceleration, 0x26, "
Second acceleration between V1 and VMAX (unsigned)

This is the acceleration and deceleration value for velocity mode.
");
motion_register!(VMax, TmcVelocity, 0x27, "
Motion ramp target velocity (for positioning ensure VMAX ≥ VSTART) (unsigned)

This is the target velocity in velocity mode. It can be changed any time during a motion.
");
motion_register!(DMax, TmcAcceleration, 0x28, "
Deceleration between VMAX and V1 (unsigned)
");
motion_register!(D1, TmcAcceleration, 0x2A, "
Deceleration between V1 and VSTOP (unsigned)

Attention: Do not set 0 in positioning mode, even if V1=0!
");
motion_register!(VStop, TmcVelocity, 0x2B, "
Motor stop velocity (unsigned)

Hint: Set VSTOP ≥ VSTART to allow positioning for short distances

Attention: Do not set 0 in positioning mode, minimum 10 recommend, set >100 for faster ramp termination!
");
motion_register!(TZeroWait, TmcUnitless, 0x2C, "
Defines the waiting time after ramping down to zero velocity before next movement or direction inversion can start. Time
range is about 0 to 2 seconds.

This setting avoids excess acceleration e.g. from VSTOP to -VSTART.
");
motion_register!(XTarget, TmcPosition, 0x2D, "
Target position for ramp mode (signed). Write a new target position to this register in order to activate the ramp
generator positioning in RAMPMODE=0. Initialize all velocity, acceleration, and deceleration parameters before.

Hint: The position is allowed to wrap around, thus, XTARGET value optionally can be treated as an unsigned number.

Hint: The maximum possible displacement is +/-((2^31)-1).

Hint: When increasing V1, D1 or DMAX during a motion, rewrite XTARGET afterwards to trigger a second acceleration phase,
if desired.
");

motion_register!(VDcMin, TmcVelocity, 0x33, "
Automatic commutation DcStep becomes enabled above velocity VDCMIN (unsigned) (only when using internal ramp generator,
not for STEP/DIR interface - in STEP/DIR mode, DcStep becomes enabled by the external signal DCEN)

In this mode, the actual position is determined by the sensorless motor commutation and becomes fed back to XACTUAL. In
case the motor becomes heavily loaded, VDCMIN also is used as the minimum step velocity. Activate stop on stall
(sg_stop) to detect step loss.

0: Disable, DcStep off

|VACT| ≥ VDCMIN ≥ 256:
- Triggers the same actions as exceeding THIGH setting.
- Switches on automatic commutation DcStep

Hint: Also set DCCTRL parameters to operate DcStep. (Only bits 22… 8 are used for value and for comparison)
");

/// Chopper Configuration
#[bitfield(u32)]
pub struct ChopConf {
    /// Off time setting controls duration of slow decay phase NCLK= 24 + 32*TOFF
    /// - 0: Driver disable, all bridges off
    /// - 1: Use only with TBL ≥ 2
    #[bits(4)]
    pub toff: u8,

    /// hysteresis start value added to HEND OR fast decay time setting
    /// 
    /// **chm=0:**
    /// Add 1, 2, …, 8 to hysteresis low value HEND (1/512 of this setting adds to current setting)
    /// 
    /// Attention: Effective HEND+HSTRT ≤ 16.
    /// 
    /// Hint: Hysteresis decrement is done each 16 clocks
    /// 
    /// **chm=1:**
    /// Fast decay time setting (MSB: fd3)
    /// 
    /// Fast decay time setting TFD with NCLK= 32*TFD
    #[bits(3)]
    pub hstrt: u8,

    /// hysteresis low value OR sine wave offset
    /// 
    /// **chm=0:**
    /// Hysteresis is -3, -2, -1, 0, 1, …, 12 (1/512 of this setting adds to current setting) This is the hysteresis
    /// value which becomes used for the hysteresis chopper.
    /// 
    /// **chm=1:**
    /// Offset is -3, -2, -1, 0, 1, …, 12 This is the sine wave offset and 1/512 of the value becomes added to the
    /// absolute value of each sine wave entry.
    #[bits(4)]
    pub hend: u8,

    /// chm=1: MSB of fast decay time setting TFD
    #[bits(1)]
    pub fd3: u8,

    /// fast decay mode
    /// 
    /// chm=1: disfdcc=1 disables current comparator usage for termination of the fast decay cycle
    #[bits(1)]
    pub disfdcc: bool,

    #[bits(1)]
    __: u32,

    /// chopper mode
    /// 
    /// - true: Standard mode (SpreadCycle)
    /// - false: Constant off time with fast decay time. Fast decay time is also terminated when the negative nominal
    /// current is reached. Fast decay is after on time.
    #[bits(1)]
    pub chm: bool,

    /// blank time select
    /// 
    /// Set comparator blank time to 16, 24, 36 or 54 clocks
    /// 
    /// Hint: 1 or 2 is recommended for most applications
    #[bits(2)]
    pub tbl: u8,

    #[bits(1)]
    __: u32,

    /// high velocity fullstep selection
    /// 
    /// This bit enables switching to fullstep, when VHIGH is exceeded. Switching takes place only at 45° position. The
    /// fullstep target current uses the current value from the microstep table at the 45° position.
    #[bits(1)]
    pub vhighfs: bool,

    /// high velocity chopper mode
    /// 
    /// This bit enables switching to chm=1 and fd=0, when VHIGH is exceeded. This way, a higher velocity can be
    /// achieved. Can be combined with vhighfs=1. If set, the TOFF setting automatically becomes doubled during high
    /// velocity operation in order to avoid doubling of the chopper frequency.
    #[bits(1)]
    pub vhighchm: bool,

    /// passive fast decay time
    /// 
    /// TPFD allows dampening of motor mid-range resonances. Passive fast decay time setting controls duration of the
    /// fast decay phase inserted after bridge polarity change NCLK= 128*TPFD
    #[bits(4)]
    pub tpdf: u8,

    /// micro step resolution
    /// 
    /// 0: Native 256 microstep setting. Normally use this setting with the internal motion controller.
    /// 
    /// 1…8: 128, 64, 32, 16, 8, 4, 2, FULLSTEP Reduced microstep resolution esp. for STEP/DIR operation. 
    /// 
    /// The resolution gives the number of microstep entries per sine quarter wave. The driver automatically uses
    /// microstep positions which result in a symmetrical wave, when choosing a lower microstep resolution. step
    /// width=2^MRES [microsteps]
    #[bits(4)]
    pub mres: u8,

    /// interpolation to 256 microsteps
    /// 
    /// The actual microstep resolution (MRES) becomes extrapolated to 256 microsteps for smoothest motor operation
    /// (useful for STEP/DIR operation, only)
    #[bits(1)]
    pub interpol: bool,

    /// enable double edge step pulses
    /// 
    /// Enable step impulse at each step edge to reduce step frequency requirement.
    #[bits(1)]
    pub dedge: bool,

    /// short to GND protection disable
    #[bits(1)]
    pub diss2g: bool,

    /// short to supply protection disable
    #[bits(1)]
    pub diss2vs: bool,
}

impl Register for ChopConf {
    const ADDRESS: u8 = 0x6C;
}

/// Smart Energy Control CoolStep and StallGuard2
#[bitfield(u32)]
pub struct CoolConf {
    /// minimum StallGuard2 value for smart current control and smart current enable
    /// 
    /// If the StallGuard2 result falls below SEMIN*32, the motor current becomes increased to reduce motor load angle.
    /// - 0: smart current control CoolStep off
    #[bits(4)]
    pub semin: u8,

    #[bits(1)]
    __: u32,

    /// current up step width
    /// 
    /// Current increment steps per measured StallGuard2 value
    /// 
    /// 0 … 3: 1, 2, 4, 8
    #[bits(2)]
    pub seup: u8,

    #[bits(1)]
    __: u32,

    /// StallGuard2 hysteresis value for smart current control
    /// 
    /// If the StallGuard2 result is equal to or above (SEMIN+SEMAX+1)*32, the motor current becomes decreased to save
    /// energy.
    #[bits(4)]
    pub semax: u8,

    #[bits(1)]
    __: u32,

    /// current down step speed
    /// 
    /// - 0: For each 32 StallGuard2 values decrease by one
    /// - 1: For each 8 StallGuard2 values decrease by one
    /// - 2: For each 2 StallGuard2 values decrease by one
    /// - 3: For each StallGuard2 value decrease by one
    #[bits(2)]
    pub sedn: u8,

    /// minimum current for smart current control
    /// 
    /// - false: 1/2 of current setting (IRUN)
    /// - true: 1/4 of current setting (IRUN)
    #[bits(1)]
    pub seimin: bool,

    /// StallGuard2 threshold value
    /// 
    /// This signed value controls StallGuard2 level for stall output and sets the optimum measurement range for
    /// readout. A lower value gives a higher sensitivity. Zero is the starting value working with most motors.
    /// 
    /// A higher value makes StallGuard2 less sensitive and requires more torque to indicate a stall.
    #[bits(7)]
    pub sgt: i8,

    #[bits(1)]
    __: u32,

    /// StallGuard2 filter enable
    /// 
    /// false: Standard mode, high time resolution for StallGuard2
    /// true: Filtered mode, StallGuard2 signal updated for each four fullsteps (resp. six fullsteps for 3 phase motor)
    /// only to compensate for motor pole tolerances
    #[bits(1)]
    pub sfilt: bool,

    #[bits(7)]
    __: u32,
}

impl Register for CoolConf {
    const ADDRESS: u8 = 0x6D;
}

/// DcStep (DC) automatic commutation configuration register (enable via pin DCEN or via VDCMIN)
/// 
/// Hint: Using a higher microstep resolution or interpolated operation, DcStep delivers a better StallGuard signal.
/// DC_SG is also available above VHIGH if vhighfs is activated. For best result also set vhighchm.
#[bitfield(u32)]
pub struct DcCtrl {
    /// Upper PWM on time limit for commutation (DC_TIME *1/fCLK). Set slightly above effective blank time TBL.
    #[bits(10)]
    pub dc_time: u16,

    #[bits(6)]
    __: u32,

    /// Max. PWM on time for step loss detection using DcStep StallGuard2 in DcStep mode. (DC_SG * 16/fCLK). Set
    /// slightly higher than DC_TIME/16
    /// 
    /// 0=disable
    #[bits(8)]
    pub dc_sg: u8,

    #[bits(8)]
    __: u32,
}

impl Register for DcCtrl {
    const ADDRESS: u8 = 0x6E;
}

/// StallGuard2 Value and Driver Error Flags
#[bitfield(u32)]
pub struct DrvStatus {
    /// StallGuard2 result respectively PWM on time for coil A in standstill for motor temperature detection
    /// 
    /// **Mechanical load measurement:**
    /// The StallGuard2 result gives a means to measure mechanical motor load. A higher value means lower mechanical
    /// load. A value of 0 signals highest load. With optimum SGT setting, this is an indicator for a motor stall. The
    /// stall detection compares SG_RESULT to 0 to detect a stall. SG_RESULT is used as a base for CoolStep operation,
    /// by comparing it to a programmable upper and a lower limit. It is not applicable in StealthChop mode.
    /// 
    /// StallGuard2 works best with microstep operation or DcStep.
    /// 
    /// **Temperature measurement:**
    /// In standstill, no StallGuard2 result can be obtained. SG_RESULT shows the chopper on-time for motor coil A
    /// instead. Move the motor to a determined microstep position at a certain current setting to get a rough
    /// estimation of motor temperature by a reading the chopper on-time. As the motor heats up, its coil resistance
    /// rises and the chopper on-time increases.
    #[bits(10)]
    pub sg_result: u16,

    #[bits(2)]
    __: u32,

    /// short to supply indicator phase A
    /// 
    /// The driver becomes disabled. The flags stay active, until the driver is disabled by software (TOFF=0) or by the
    /// DRV_ENN input. Sense resistor voltage drop is included in the measurement!
    #[bits(1)]
    pub s2vsa: bool,

    /// short to supply indicator phase B
    /// 
    /// The driver becomes disabled. The flags stay active, until the driver is disabled by software (TOFF=0) or by the
    /// DRV_ENN input. Sense resistor voltage drop is included in the measurement!
    #[bits(1)]
    pub s2vsb: bool,

    /// StealthChop indicator 
    /// 
    /// Driver operates in StealthChop mode
    #[bits(1)]
    pub stealth: bool,

    /// full step active indicator
    /// 
    /// Indicates that the driver has switched to fullstep as defined by chopper mode settings and velocity thresholds.
    #[bits(1)]
    pub fsactive: bool,

    /// actual motor current / smart energy current
    /// 
    /// Actual current control scaling, for monitoring smart energy current scaling controlled via settings in register
    /// COOLCONF, or for monitoring the function of the automatic current scaling.
    #[bits(5)]
    pub cs_actual: u8,

    #[bits(3)]
    __: u32,

    /// StallGuard2 status
    /// 
    /// Motor stall detected (SG_RESULT=0) or DcStep stall in DcStep mode.
    #[bits(1)]
    pub stall_guard: bool,

    /// overtemperature flag
    /// 
    /// Overtemperature limit has been reached. Drivers become disabled until otpw is also cleared due to cooling down
    /// of the IC. The overtemperature flag is common for both bridges.
    #[bits(1)]
    pub ot: bool,

    /// overtemperature pre-warning flag
    /// 
    /// Overtemperature pre-warning threshold is exceeded. The overtemperature pre-warning flag is common for both
    /// bridges.
    #[bits(1)]
    pub otpw: bool,

    /// short to ground indicator phase A
    /// 
    /// The driver becomes disabled. The flags stay active, until the driver is disabled by software (TOFF=0) or by the
    /// DRV_ENN input.
    #[bits(1)]
    pub s2ga: bool,

    /// short to ground indicator phase B
    /// 
    /// The driver becomes disabled. The flags stay active, until the driver is disabled by software (TOFF=0) or by the
    /// DRV_ENN input.
    #[bits(1)]
    pub s2gb: bool,

    /// open load indicator phase A
    /// 
    /// Hint: This is just an informative flag. The driver takes no action upon it. False detection may occur in fast
    /// motion and standstill. Check during slow motion, only.
    #[bits(1)]
    pub ola: bool,

    /// open load indicator phase B
    /// 
    /// Hint: This is just an informative flag. The driver takes no action upon it. False detection may occur in fast
    /// motion and standstill. Check during slow motion, only.
    #[bits(1)]
    pub olb: bool,

    /// standstill indicator
    /// 
    /// This flag indicates motor stand still in each operation mode. This occurs 2^20 clocks after the last step pulse.
    #[bits(1)]
    pub stst: bool,
}

impl Register for DrvStatus {
    const ADDRESS: u8 = 0x6F;
}

/// Voltage Mode PWM StealthChop
#[bitfield(u32)]
pub struct PwmConf {
    /// User defined amplitude (offset)
    /// 
    /// User defined PWM amplitude offset (0-255) related to full motor current (CS_ACTUAL=31) in stand still.
    /// 
    /// Use PWM_OFS as initial value for automatic scaling to speed up the automatic tuning process. To do this, set
    /// PWM_OFS to the determined, application specific value, with pwm_autoscale=0. Only afterwards, set
    /// pwm_autoscale=1. Enable StealthChop when finished.
    /// 
    /// PWM_OFS = 0 will disable scaling down motor current below a motor specific lower measurement threshold. This
    /// setting should only be used under certain conditions, i.e., when the power supply voltage can vary up and down
    /// by a factor of two or more. It prevents the motor going out of regulation, but it also prevents power down below
    /// the regulation limit.
    /// 
    /// PWM_OFS > 0 allows automatic scaling to low PWM duty cycles even below the lower regulation threshold. This
    /// allows low (standstill) current settings based on the actual (hold) current scale (register IHOLD_IRUN).
    #[bits(8)]
    pub pwm_ofs: u8,

    /// User defined amplitude gradient
    /// 
    /// Velocity dependent gradient for PWM amplitude: PWM_GRAD * 256 / TSTEP. This value is added to PWM_OFS to
    /// compensate for the velocity-dependent motor back-EMF.
    /// 
    /// Use PWM_GRAD as initial value for automatic scaling to speed up the automatic tuning process. To do this, set
    /// PWM_GRAD to the determined, application specific value, with pwm_autoscale=0. Only afterwards, set
    /// pwm_autoscale=1. Enable StealthChop when finished.
    /// 
    /// Hint: After initial tuning, the required initial value can be read out from PWM_GRAD_AUTO.
    #[bits(8)]
    pub pwm_grad: u8,

    /// PWM frequency selection
    /// - 0: fPWM=2/1024 fCLK (Reset default)
    /// - 1: fPWM=2/683 fCLK
    /// - 2: fPWM=2/512 fCLK
    /// - 3: fPWM=2/410 fCLK
    #[bits(2)]
    pub pwm_freq: u8,

    /// PWM automatic amplitude scaling
    /// 
    /// - false: User defined feed forward PWM amplitude. The current settings IRUN and IHOLD are not enforced by
    /// regulation, but scale the PWM amplitude, only! The resulting PWM amplitude (limited to 0…255) is: PWM_OFS *
    /// ((CS_ACTUAL+1) / 32) + PWM_GRAD * 256 / TSTEP
    /// - true: Enable automatic current control
    #[bits(1)]
    pub pwm_autoscale: bool,

    /// PWM automatic gradient adaptation
    /// 
    /// **false:** Fixed value for PWM_GRAD (PWM_GRAD_AUTO = PWM_GRAD)
    /// 
    /// **true:** Automatic tuning (only with pwm_autoscale=1)
    /// 
    /// PWM_GRAD_AUTO is initialized with PWM_GRAD while pwm_autograd=0 and becomes optimized automatically during
    /// motion.
    /// 
    /// **Preconditions:**
    /// - 1. PWM_OFS_AUTO has been automatically initialized. This requires standstill at IRUN for >130ms to a) detect
    /// standstill b) wait > 128 chopper cycles at IRUN and c) regulate PWM_OFS_AUTO so that -1 < PWM_SCALE_AUTO < 1
    /// - 2. Motor running and PWM_SCALE_SUM < 255 and 1.5 * PWM_OFS_AUTO * (IRUN+1)/32 < PWM_SCALE_SUM < 4 *
    /// PWM_OFS_AUTO * (IRUN+1)/32.
    /// 
    /// **Time required for tuning PWM_GRAD_AUTO** About 8 fullsteps per change of +/-1. Also enables use of reduced
    /// chopper frequency for tuning PWM_OFS_AUTO.
    #[bits(1)]
    pub pwm_autograd: bool,

    /// Allows different standstill modes
    /// 
    /// Stand still option when motor current setting is zero (I_HOLD=0).
    /// 
    /// - 0: Normal operation
    /// - 1: Freewheeling
    /// - 2: Coil shorted using LS drivers
    /// - 3: Coil shorted using HS drivers
    #[bits(2)]
    pub freewheel: u8,

    #[bits(2)]
    __: u32,

    /// Regulation loop gradient
    /// 
    /// User defined maximum PWM amplitude change per half wave when using pwm_autoscale=1. (1…15):
    /// - 1: 0.5 increments (slowest regulation)
    /// - 2: 1 increment
    /// - 3: 1.5 increments
    /// - 4: 2 increments (Reset default))
    /// - …
    /// - 8: 4 increments
    /// - …
    /// - 15: 7.5 increments (fastest regulation)
    #[bits(4)]
    pub pwm_reg: u8,

    /// PWM automatic scale amplitude limit when switching on
    /// 
    /// Limit for PWM_SCALE_AUTO when switching back from SpreadCycle to StealthChop. This value defines the upper limit
    /// for bits 7 to 4 of the automatic current control when switching back. It can be set to reduce the current jerk
    /// during mode change back to StealthChop. It does not limit PWM_GRAD or PWM_GRAD_AUTO offset.
    #[bits(4)]
    pub pwm_lim: u8,
}

impl Register for PwmConf {
    const ADDRESS: u8 = 0x70;
}

/// These automatically generated values can be read out in order to determine a default / power up setting for PWM_GRAD
/// and PWM_OFS.
#[bitfield(u32)]
pub struct PwmAuto {
    /// Automatically determined offset value
    #[bits(8)]
    pub pwm_ofs_auto: u8,

    #[bits(8)]
    __: u32,

    /// Automatically determined gradient value
    #[bits(8)]
    pub pwm_grad_auto: u8,

    #[bits(8)]
    __: u32,
}

impl Register for PwmAuto {
    const ADDRESS: u8 = 0x72;
}
