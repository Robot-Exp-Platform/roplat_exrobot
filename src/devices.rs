//! Reference devices for the end-effector and sensor contracts.
//!
//! Like [`ExRobot`](crate::ExRobot), they connect to nothing: they print each
//! command and synthesize readings, so code written against
//! [`Gripper`], [`ForceTorqueSensor`] and [`ImuSensor`] can run without
//! hardware. Readings are stamped with [`MonotonicClock`] and carry the frame
//! given at construction and a per-device sequence number.

use roplat_behavior::{
    ForceTorqueSensor, Gripper, ImuSensor, RobotException, RobotResult,
    msgs::{
        Clock, ForceTorque, FrameId, GripperCommand, GripperState, Header, Imu, MonotonicClock,
        Quaternion, Stamped, Vector3, Wrench,
    },
};

/// A parallel gripper that reaches every commanded width at once. A
/// positive force reports a grasp; a zero-force move releases it.
pub struct ExGripper {
    width: f64,
    max_width: f64,
    grasped: bool,
}

impl ExGripper {
    /// Opens to `max_width` (m).
    pub fn new(max_width: f64) -> Self {
        Self { width: max_width, max_width, grasped: false }
    }
}

impl Default for ExGripper {
    /// Franka Hand's 0.08 m stroke.
    fn default() -> Self {
        Self::new(0.08)
    }
}

impl Gripper for ExGripper {
    fn read_gripper(&mut self) -> RobotResult<GripperState> {
        Ok(GripperState {
            width: self.width,
            max_width: self.max_width,
            grasped: self.grasped as u32,
            reserved: 0,
        })
    }

    fn move_gripper(&mut self, command: GripperCommand) -> RobotResult<()> {
        println!("ExGripper move_gripper {command:?}");
        if !(0.0..=self.max_width).contains(&command.width) {
            return Err(RobotException::UnprocessableInstructionError(format!(
                "gripper width {} is outside [0, {}]",
                command.width, self.max_width
            )));
        }
        self.width = command.width;
        self.grasped = command.force > 0.0;
        Ok(())
    }

    fn home_gripper(&mut self) -> RobotResult<()> {
        println!("ExGripper home_gripper");
        self.width = self.max_width;
        self.grasped = false;
        Ok(())
    }

    fn stop_gripper(&mut self) -> RobotResult<()> {
        println!("ExGripper stop_gripper");
        self.grasped = false;
        Ok(())
    }
}

/// A six-axis sensor under a constant load, with tare.
pub struct ExForceTorque {
    frame: FrameId,
    seq: u64,
    load: Wrench,
    bias: Wrench,
}

impl ExForceTorque {
    pub fn new(frame: FrameId, load: Wrench) -> Self {
        Self { frame, seq: 0, load, bias: Wrench::default() }
    }

    /// Changes the synthesized load, as if something were picked up.
    pub fn set_load(&mut self, load: Wrench) {
        self.load = load;
    }
}

impl ForceTorqueSensor for ExForceTorque {
    fn read_wrench(&mut self) -> RobotResult<ForceTorque> {
        self.seq += 1;
        let (load, bias) = (self.load.to_array(), self.bias.to_array());
        let value = Wrench::from(std::array::from_fn::<f64, 6, _>(|i| load[i] - bias[i]));
        Ok(ForceTorque::new(
            Header::new(MonotonicClock.now(), self.frame, self.seq),
            value,
        ))
    }

    fn bias_wrench(&mut self) -> RobotResult<()> {
        println!("ExForceTorque bias_wrench");
        self.bias = self.load;
        Ok(())
    }
}

/// An IMU at rest, level: identity orientation, no rotation, and the
/// specific force of gravity (`+9.81` m/s² along z).
pub struct ExImu {
    frame: FrameId,
    seq: u64,
}

impl ExImu {
    pub fn new(frame: FrameId) -> Self {
        Self { frame, seq: 0 }
    }
}

impl ImuSensor for ExImu {
    fn read_imu(&mut self) -> RobotResult<Stamped<Imu>> {
        self.seq += 1;
        let imu = Imu::EMPTY
            .with_orientation(Quaternion::IDENTITY)
            .with_angular_velocity(Vector3::ZERO)
            .with_linear_acceleration(Vector3::new(0.0, 0.0, 9.81));
        Ok(Stamped::new(
            Header::new(MonotonicClock.now(), self.frame, self.seq),
            imu,
        ))
    }
}
