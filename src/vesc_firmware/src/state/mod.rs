use fixed::types::I16F16;

#[derive(Default)]
pub enum ControlMode {
    Position,      // Normal usage
    Velocity,      // Normal usage
    TorqueCurrent, // First real FOC mode
    Voltage {
        voltage: I16F16,
    }, // Testing -> Hall effects + FIXME: Vbus sense
    #[default]
    Disabled,
}
