use robot_behavior::{
    ControlType, CxxArmState, LoadState, MotionType, behavior::*, cxx_arm_behavior, cxx_arm_param,
    cxx_arm_preplanned_motion, cxx_arm_preplanned_motion_ext, cxx_arm_preplanned_motion_impl,
    cxx_arm_real_time_control, cxx_arm_real_time_control_ext, cxx_arm_streaming_handle,
    cxx_arm_streaming_motion, cxx_arm_streaming_motion_ext, cxx_robot_behavior,
};

struct ExRobot(crate::ExRobot<6>);
struct ExRobotHandle(crate::ExRobotHandle<6>);

#[cxx::bridge]
mod roplat_exrobot {
    pub struct CxxMotionType {
        pub mode: CxxMotionTypeMode,
        pub values: Vec<f64>,
    }
    pub enum CxxMotionTypeMode {
        Joint,
        JointVel,
        Cartesian,
        CartesianVel,
        Position,
        PositionVel,
        Stop,
    }
    pub struct CxxControlType {
        pub mode: CxxControlTypeMode,
        pub values: Vec<f64>,
    }
    pub enum CxxControlTypeMode {
        Zero,
        Torque,
    }
    pub struct CxxLoadState {
        pub m: f64,
        pub x: [f64; 3],
        pub i: [f64; 9],
    }
    extern "Rust" {
        type ExRobot;

        #[Self = "ExRobot"]
        fn attach() -> Box<ExRobot>;

        #[Self = "ExRobot"]
        fn version() -> String;
        fn init(&mut self) -> Result<()>;
        fn shutdown(&mut self) -> Result<()>;
        fn enable(&mut self) -> Result<()>;
        fn disable(&mut self) -> Result<()>;
        fn reset(&mut self) -> Result<()>;
        fn is_moving(&mut self) -> bool;
        fn stop(&mut self) -> Result<()>;
        fn pause(&mut self) -> Result<()>;
        fn resume(&mut self) -> Result<()>;
        fn emergency_stop(&mut self) -> Result<()>;
        fn clear_emergency_stop(&mut self) -> Result<()>;

        // fn state(&mut self)
        fn set_load(&mut self, load: CxxLoadState) -> Result<()>;
        // fn set_coord
        // fn with_coord(&mut self)
        fn set_speed(&mut self, speed: f64) -> Result<()>;
        fn with_speed(&mut self, speed: f64) -> &mut ExRobot;
        unsafe fn with_velocity<'a>(&'a mut self, joint_vel: &[f64; 6]) -> &'a mut ExRobot;
        unsafe fn with_acceleration<'a>(&'a mut self, joint_acc: &[f64; 6]) -> &'a mut ExRobot;
        unsafe fn with_jerk<'a>(&'a mut self, joint_jerk: &[f64; 6]) -> &'a mut ExRobot;
        fn with_cartesian_velocity(&mut self, cartesian_vel: f64) -> &mut ExRobot;
        fn with_cartesian_acceleration(&mut self, cartesian_acc: f64) -> &mut ExRobot;
        fn with_cartesian_jerk(&mut self, cartesian_jerk: f64) -> &mut ExRobot;
        fn with_rotation_velocity(&mut self, rotation_vel: f64) -> &mut ExRobot;
        fn with_rotation_acceleration(&mut self, rotation_acc: f64) -> &mut ExRobot;
        fn with_rotation_jerk(&mut self, rotation_jerk: f64) -> &mut ExRobot;

        fn move_joint(&mut self, target: &[f64; 6]) -> Result<()>;
        fn move_joint_async(&mut self, target: &[f64; 6]) -> Result<()>;
        fn move_cartesian(&mut self, target: &[f64]) -> Result<()>;
        fn move_cartesian_async(&mut self, target: &[f64]) -> Result<()>;

        fn move_to(&mut self, target: CxxMotionType) -> Result<()>;
        fn move_to_async(&mut self, target: CxxMotionType) -> Result<()>;
        fn move_rel(&mut self, target: CxxMotionType) -> Result<()>;
        fn move_rel_async(&mut self, target: CxxMotionType) -> Result<()>;
        fn move_int(&mut self, target: CxxMotionType) -> Result<()>;
        fn move_int_async(&mut self, target: CxxMotionType) -> Result<()>;
        // fn move_path(&mut self, path: Vec<CxxMotionType>) -> Result<()>;
        // fn move_path_async(&mut self, path: Vec<CxxMotionType>) -> Result<()>;
        // fn move_path_prepare(&mut self, path: Vec<CxxMotionType>) -> Result<()>;
        fn move_path_start(&mut self, start: CxxMotionType) -> Result<()>;

        fn move_joint_rel(&mut self, target: &[f64; 6]) -> Result<()>;
        fn move_joint_rel_async(&mut self, target: &[f64; 6]) -> Result<()>;
        // fn move_joint_path(&mut self, path: Vec<[f64; 6]>) -> Result<()>;
        // fn move_cartesian_rel(&mut self, target: &Pose) -> Result<()>;
        // fn move_cartesian_rel_async(&mut self, target: &Pose) -> Result<()>;
        // fn move_cartesian_int(&mut self, target: &Pose) -> Result<()>;
        // fn move_cartesian_int_async(&mut self, target: &Pose) -> Result<()>;
        // fn move_cartesian_path(&mut self, path: Vec<Pose>) -> Result<()>;/
        fn move_linear_with_euler(&mut self, pose: [f64; 6]) -> Result<()>;
        fn move_linear_with_euler_async(&mut self, pose: [f64; 6]) -> Result<()>;
        fn move_linear_with_euler_rel(&mut self, pose: [f64; 6]) -> Result<()>;
        fn move_linear_with_euler_rel_async(&mut self, pose: [f64; 6]) -> Result<()>;
        fn move_linear_with_euler_int(&mut self, pose: [f64; 6]) -> Result<()>;
        fn move_linear_with_euler_int_async(&mut self, pose: [f64; 6]) -> Result<()>;
        // fn move_linear_with_quat(&mut self, target: &na::Isometry3<f64>) -> Result<()>;
        // fn move_linear_with_quat_async(&mut self, target: &na::Isometry3<f64>) -> Result<()>;
        // fn move_linear_with_quat_rel(&mut self, target: &na::Isometry3<f64>) -> Result<()>;
        // fn move_linear_with_quat_rel_async(&mut self, target: &na::Isometry3<f64>) -> Result<()>;
        // fn move_linear_with_quat_int(&mut self, target: &na::Isometry3<f64>) -> Result<()>;
        // fn move_linear_with_quat_int_async(&mut self, target: &na::Isometry3<f64>) -> Result<()>;
        fn move_linear_with_homo(&mut self, target: [f64; 16]) -> Result<()>;
        fn move_linear_with_homo_async(&mut self, target: [f64; 16]) -> Result<()>;
        fn move_linear_with_homo_rel(&mut self, target: [f64; 16]) -> Result<()>;
        fn move_linear_with_homo_rel_async(&mut self, target: [f64; 16]) -> Result<()>;
        fn move_linear_with_homo_int(&mut self, target: [f64; 16]) -> Result<()>;
        fn move_linear_with_homo_int_async(&mut self, target: [f64; 16]) -> Result<()>;
        fn move_path_prepare_from_file(&mut self, path: &str) -> Result<()>;
        fn move_path_from_file(&mut self, path: &str) -> Result<()>;

        fn start_streaming(&mut self) -> Result<Box<ExRobotHandle>>;
        fn end_streaming(&mut self) -> Result<()>;
        // fn move_to_target(&mut self) -> Arc<Mutex<Option<CxxMotionType>>>;
        // fn control_with_target(&mut self) -> Arc<Mutex<Option<ControlType<6>>>>;
    }
    extern "Rust" {
        type ExRobotHandle;

        fn last_motion(&self) -> CxxMotionType;
        fn move_to(&mut self, target: CxxMotionType) -> Result<()>;
        fn last_control(&self) -> CxxControlType;
        fn control_with(&mut self, control: CxxControlType) -> Result<()>;
    }
}

pub use roplat_exrobot::*;

impl ExRobot {
    pub fn attach() -> Box<Self> {
        Box::new(ExRobot(crate::ExRobot::new()))
    }
}

impl ExRobot {
    cxx_robot_behavior!(ExRobot(crate::ExRobot::<6>));
    cxx_arm_behavior!(ExRobot<{6}>(crate::ExRobot::<6>));
    cxx_arm_param!(ExRobot<{6}>(crate::ExRobot::<6>));
    cxx_arm_preplanned_motion_impl!(ExRobot<{6}>(crate::ExRobot::<6>));
    cxx_arm_preplanned_motion!(ExRobot<{6}>(crate::ExRobot::<6>));
    cxx_arm_preplanned_motion_ext!(ExRobot<{6}>(crate::ExRobot::<6>));
    cxx_arm_streaming_motion!(ExRobot<{6}>(crate::ExRobot::<6>) -> ExRobotHandle);
    cxx_arm_streaming_motion_ext!(ExRobot<{6}>(crate::ExRobot::<6>));
    cxx_arm_real_time_control!(ExRobot<{6}>(crate::ExRobot::<6>));
    cxx_arm_real_time_control_ext!(ExRobot<{6}>(crate::ExRobot::<6>));
}

impl ExRobotHandle {
    cxx_arm_streaming_handle!(ExRobotHandle<{6}>(crate::ExStreamHandle::<6>));
}

impl<const N: usize> From<CxxMotionType> for MotionType<N> {
    fn from(cxx: CxxMotionType) -> Self {
        match cxx.mode {
            CxxMotionTypeMode::Joint => MotionType::Joint(cxx.values.try_into().unwrap()),
            CxxMotionTypeMode::JointVel => MotionType::JointVel(cxx.values.try_into().unwrap()),
            CxxMotionTypeMode::Cartesian => MotionType::Cartesian(cxx.values.into()),
            CxxMotionTypeMode::CartesianVel => {
                MotionType::CartesianVel(cxx.values.try_into().unwrap())
            }
            CxxMotionTypeMode::Position => MotionType::Position(cxx.values.try_into().unwrap()),
            CxxMotionTypeMode::PositionVel => {
                MotionType::PositionVel(cxx.values.try_into().unwrap())
            }
            CxxMotionTypeMode::Stop => MotionType::Stop,
            _ => panic!("Invalid mode for MotionType"),
        }
    }
}

impl<const N: usize> From<MotionType<N>> for CxxMotionType {
    fn from(motion: MotionType<N>) -> Self {
        match motion {
            MotionType::Joint(v) => CxxMotionType {
                mode: CxxMotionTypeMode::Joint,
                values: v.to_vec(),
            },
            MotionType::JointVel(v) => CxxMotionType {
                mode: CxxMotionTypeMode::JointVel,
                values: v.to_vec(),
            },
            MotionType::Cartesian(v) => CxxMotionType {
                mode: CxxMotionTypeMode::Cartesian,
                values: v.into(),
            },
            MotionType::CartesianVel(v) => CxxMotionType {
                mode: CxxMotionTypeMode::CartesianVel,
                values: v.to_vec(),
            },
            MotionType::Position(v) => CxxMotionType {
                mode: CxxMotionTypeMode::Position,
                values: v.to_vec(),
            },
            MotionType::PositionVel(v) => CxxMotionType {
                mode: CxxMotionTypeMode::PositionVel,
                values: v.to_vec(),
            },
            MotionType::Stop => CxxMotionType {
                mode: CxxMotionTypeMode::Position, // Use Position as a placeholder for Stop
                values: vec![],
            },
        }
    }
}

impl<const N: usize> From<CxxControlType> for ControlType<N> {
    fn from(cxx: CxxControlType) -> Self {
        match cxx.mode {
            CxxControlTypeMode::Zero => ControlType::Zero,
            CxxControlTypeMode::Torque => ControlType::Torque(cxx.values.try_into().unwrap()),
            _ => panic!("Invalid mode for ControlType"),
        }
    }
}

impl<const N: usize> From<ControlType<N>> for CxxControlType {
    fn from(control: ControlType<N>) -> Self {
        match control {
            ControlType::Zero => CxxControlType {
                mode: CxxControlTypeMode::Zero,
                values: vec![],
            },
            ControlType::Torque(v) => CxxControlType {
                mode: CxxControlTypeMode::Torque,
                values: v.to_vec(),
            },
        }
    }
}

impl From<CxxLoadState> for LoadState {
    fn from(cxx: CxxLoadState) -> Self {
        LoadState {
            m: cxx.m,
            x: cxx.x,
            i: cxx.i,
        }
    }
}
