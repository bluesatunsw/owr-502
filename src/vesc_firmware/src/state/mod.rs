use fixed::types::I16F16;

pub struct AppState {
    pub motioncontrol: MotionControlState,
    pub commutation: CommutationState,
}

pub enum MotionControlState {
    Position,
    Velocity,
    None,
}

#[derive(Default)]
pub enum CommutationState {
    /* Current {
        current: I16F16,
    }, TODO: After Jonah implements current sensing (assuming we do the transform) */
    Voltage {
        voltage: I16F16,
    }, // Testing -> Hall effects + FIXME: Vbus sense
    #[default]
    Disabled,
}
