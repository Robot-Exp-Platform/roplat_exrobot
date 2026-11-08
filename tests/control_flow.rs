use std::ops::ControlFlow;

use roplat_behavior::{
    BaseVelocityControl, CartesianPoseControl, ControlExit, ControlWith, JointPositionControl,
    Robot, RobotException,
};
use roplat_exrobot::{ExMobileBase, ExRobot};

#[test]
fn arm_break_stops_without_a_command() {
    let mut robot = ExRobot::<6>::new();
    let mut calls = 0;
    <ExRobot<6> as ControlWith<JointPositionControl<6>>>::control_with_flow(&mut robot, |_, _| {
        calls += 1;
        ControlFlow::Break(ControlExit::Stopped)
    })
    .unwrap();
    assert_eq!(calls, 1);
}

#[test]
fn failed_break_returns_the_controller_error() {
    let mut robot = ExRobot::<6>::new();
    let mut calls = 0;
    let result = <ExRobot<6> as ControlWith<JointPositionControl<6>>>::control_with_flow(
        &mut robot,
        |_, _| {
            calls += 1;
            if calls == 1 {
                ControlFlow::Continue(([0.0; 6], false))
            } else {
                ControlFlow::Break(ControlExit::Failed(RobotException::CommandException(
                    "controller failed".into(),
                )))
            }
        },
    );
    assert!(
        matches!(result, Err(RobotException::CommandException(message)) if message == "controller failed")
    );
    assert_eq!(calls, 2);
}

#[test]
fn failed_break_ends_an_arm_state_session_too() {
    let mut robot = ExRobot::<6>::new();
    let result = <ExRobot<6> as ControlWith<CartesianPoseControl<6>>>::control_with_flow(
        &mut robot,
        |_, _| {
            ControlFlow::Break(ControlExit::Failed(RobotException::CommandException(
                "no pose".into(),
            )))
        },
    );
    assert!(
        matches!(result, Err(RobotException::CommandException(message)) if message == "no pose")
    );
}

#[test]
fn base_break_stops_after_one_valid_cycle() {
    let mut robot = ExMobileBase::new();
    let mut calls = 0;
    <ExMobileBase as ControlWith<BaseVelocityControl>>::control_with_flow(&mut robot, |_, dt| {
        calls += 1;
        // Every cycle reports the nominal period (no cycle is ever missed).
        assert_eq!(dt, ExMobileBase::CONTROL_PERIOD);
        if calls == 1 {
            ControlFlow::Continue(([0.0; 6], false))
        } else {
            ControlFlow::Break(ControlExit::Stopped)
        }
    })
    .unwrap();
    assert_eq!(calls, 2);
}

#[test]
fn legacy_callback_sends_final_command_and_finishes() {
    let mut robot = ExRobot::<6>::new();
    let mut calls = 0;
    <ExRobot<6> as ControlWith<JointPositionControl<6>>>::control_with(&mut robot, |_, _| {
        calls += 1;
        ([1.0; 6], calls == 3)
    })
    .unwrap();
    assert_eq!(calls, 3);
}

#[test]
fn async_flow_callback_can_borrow_session_local_state() {
    let mut robot = ExRobot::<6>::new();
    let mut calls = 0;
    <ExRobot<6> as ControlWith<JointPositionControl<6>>>::control_with_flow_async(
        &mut robot,
        async |_, _| {
            calls += 1;
            ControlFlow::Break(ControlExit::Stopped)
        },
    )
    .unwrap();
    assert_eq!(calls, 1);
}

#[test]
fn arm_cycles_report_the_nominal_period() {
    let mut robot = ExRobot::<6>::new();
    let mut periods = Vec::new();
    <ExRobot<6> as ControlWith<JointPositionControl<6>>>::control_with(&mut robot, |_, dt| {
        periods.push(dt);
        ([0.0; 6], periods.len() == 3)
    })
    .unwrap();
    assert_eq!(periods, vec![ExRobot::<6>::CONTROL_PERIOD; 3]);
}
