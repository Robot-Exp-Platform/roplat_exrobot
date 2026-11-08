//! What the example robots promise under the `Robot` and category contracts.

use roplat_behavior::{Robot, RobotException, VelocityControlledBase};
use roplat_exrobot::{ExHumanoid, ExMobileBase, ExQuadruped, ExRobot};

fn reports_no_latch(result: Result<(), RobotException>) -> bool {
    matches!(
        result,
        Err(RobotException::UnprocessableInstructionError(_))
    )
}

#[test]
fn emergency_stop_says_it_cannot_latch() {
    // None of the example robots can latch a stop, so none returns `Ok`.
    assert!(reports_no_latch(ExRobot::<6>::new().emergency_stop()));
    assert!(reports_no_latch(ExMobileBase::new().emergency_stop()));
    assert!(reports_no_latch(ExQuadruped::<12>::new().emergency_stop()));
    assert!(reports_no_latch(ExHumanoid::<30>::new().emergency_stop()));
}

#[test]
fn the_mobile_base_also_takes_velocity_commands() {
    fn velocity_controlled(base: &mut impl VelocityControlledBase) {
        base.base_state().unwrap();
    }
    velocity_controlled(&mut ExMobileBase::new());
}
