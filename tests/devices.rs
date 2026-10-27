//! The reference devices behave as their contracts describe.

use roplat_behavior::{
    ForceTorqueSensor, Gripper, ImuSensor,
    msgs::{FrameId, GripperCommand, Imu, Wrench, clock},
};
use roplat_exrobot::{ExForceTorque, ExGripper, ExImu};

#[test]
fn gripper_moves_grasps_and_rejects_out_of_range_widths() {
    let mut gripper = ExGripper::default();
    assert_eq!(gripper.read_gripper().unwrap().width, 0.08);
    gripper.move_gripper(GripperCommand { width: 0.03, speed: 0.1, force: 0.0 }).unwrap();
    let state = gripper.read_gripper().unwrap();
    assert_eq!((state.width, state.grasped), (0.03, 0));
    gripper.move_gripper(GripperCommand { width: 0.01, speed: 0.1, force: 20.0 }).unwrap();
    assert_eq!(gripper.read_gripper().unwrap().grasped, 1);
    gripper.home_gripper().unwrap();
    assert_eq!(gripper.read_gripper().unwrap().width, 0.08);
    assert!(gripper.move_gripper(GripperCommand { width: 0.2, speed: 0.1, force: 0.0 }).is_err());
}

#[test]
fn force_torque_reads_the_load_and_tares() {
    let load = Wrench::from([0.0, 0.0, 9.81, 0.0, 0.0, 0.0]);
    let mut sensor = ExForceTorque::new(FrameId::new(5), load);
    let first = sensor.read_wrench().unwrap();
    let second = sensor.read_wrench().unwrap();
    assert_eq!(first.value, load);
    assert_eq!((first.header.seq, second.header.seq), (1, 2));
    assert_eq!(first.header.frame, FrameId::new(5));
    assert_eq!(first.header.stamp.clock, clock::MONOTONIC);
    assert!(second.header.stamp.nanos >= first.header.stamp.nanos);

    sensor.bias_wrench().unwrap();
    assert_eq!(sensor.read_wrench().unwrap().value, Wrench::default());
    sensor.set_load(Wrench::from([0.0, 0.0, 14.0, 0.0, 0.0, 0.0]));
    assert!((sensor.read_wrench().unwrap().value.force.z - (14.0 - 9.81)).abs() < 1e-12);
}

#[test]
fn imu_reads_gravity_at_rest() {
    let mut imu = ExImu::new(FrameId::new(9));
    let reading = imu.read_imu().unwrap();
    assert_eq!(
        reading.value.valid_mask(),
        Imu::ORIENTATION | Imu::ANGULAR_VELOCITY | Imu::LINEAR_ACCELERATION
    );
    assert_eq!(reading.value.linear_acceleration().unwrap().z, 9.81);
    assert_eq!(imu.read_imu().unwrap().header.seq, 2);
}
