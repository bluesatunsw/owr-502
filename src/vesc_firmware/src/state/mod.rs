pub struct AppState {
    pub motioncontrol: MotionControlState,
    pub commutation: CommutationState,
}

pub enum MotionControlState {
    Position,
    Velocity,
    None,
}

pub enum CommutationState {
    /* Current {
        current: I16F16,
    }, TODO: After Jonah implements current sensing (assuming we do the transform) */
    Voltage { voltage: f32 }, // Testing -> Hall effects + FIXME: Vbus sense
    Disabled,
}
