use nalgebra as na;
use robot_behavior::{
    ArmState, ControlType, Coord, LoadState, MotionType, Pose, Realtime, RobotResult, behavior::*,
};
use std::{
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

#[derive(Default)]
pub struct ExRobot<const N: usize>;
#[derive(Default)]
pub struct ExRobotHandle<const N: usize>;

impl<const N: usize> ExRobot<N> {
    pub fn new() -> Self {
        ExRobot
    }
}

impl<const N: usize> RobotFile for ExRobot<N> {
    const URDF: &'static str = "";
}

impl<const N: usize> Robot for ExRobot<N> {
    type State = String;

    fn version() -> String {
        env!("CARGO_PKG_VERSION").to_string()
    }

    fn init(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> init");
        Ok(())
    }

    fn shutdown(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> shutdown");
        Ok(())
    }

    fn enable(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> enable");
        Ok(())
    }

    fn disable(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> disable");
        Ok(())
    }

    fn reset(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> reset");
        Ok(())
    }

    fn is_moving(&mut self) -> bool {
        println!("ExRobot<{N}> is_moving");
        false
    }

    fn stop(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> stop");
        Ok(())
    }

    fn pause(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> pause");
        Ok(())
    }

    fn resume(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> resume");
        Ok(())
    }

    fn emergency_stop(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> emergency_stop");
        Ok(())
    }

    fn clear_emergency_stop(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> clear_emergency_stop");
        Ok(())
    }

    fn read_state(&mut self) -> RobotResult<Self::State> {
        println!("ExRobot<{N}> read_state");
        Ok(format!("ExRobot<{N}> State"))
    }
}

impl<const N: usize> Arm<N> for ExRobot<N> {
    fn state(&mut self) -> RobotResult<ArmState<N>> {
        println!("ExRobot<{N}> state");
        Ok(ArmState::default())
    }

    fn set_load(&mut self, load: LoadState) -> RobotResult<()> {
        println!("ExRobot<{N}> set_load: {load:?}");
        Ok(())
    }
    fn set_coord(&mut self, coord: Coord) -> RobotResult<()> {
        println!("ExRobot<{N}> set_coord: {coord:?}");
        Ok(())
    }
    fn with_coord(&mut self, coord: Coord) -> &mut Self {
        println!("\t| ExRobot<{N}> with_coord: {coord:?}");
        self
    }

    fn set_speed(&mut self, speed: f64) -> RobotResult<()> {
        println!("ExRobot<{N}> set_speed: {speed}");
        Ok(())
    }
    fn with_speed(&mut self, speed: f64) -> &mut Self {
        println!("\t| ExRobot<{N}> with_speed: {speed}");
        self
    }

    fn with_velocity(&mut self, joint_vel: &[f64; N]) -> &mut Self {
        println!("\t| ExRobot<{N}> with_velocity: {joint_vel:?}");
        self
    }
    fn with_acceleration(&mut self, joint_acc: &[f64; N]) -> &mut Self {
        println!("\t| ExRobot<{N}> with_acceleration: {joint_acc:?}");
        self
    }
    fn with_jerk(&mut self, joint_jerk: &[f64; N]) -> &mut Self {
        println!("\t| ExRobot<{N}> with_jerk: {joint_jerk:?}");
        self
    }
    fn with_cartesian_velocity(&mut self, cartesian_vel: f64) -> &mut Self {
        println!("\t| ExRobot<{N}> with_cartesian_velocity: {cartesian_vel}");
        self
    }
    fn with_cartesian_acceleration(&mut self, cartesian_acc: f64) -> &mut Self {
        println!("\t| ExRobot<{N}> with_cartesian_acceleration: {cartesian_acc}");
        self
    }
    fn with_cartesian_jerk(&mut self, cartesian_jerk: f64) -> &mut Self {
        println!("\t| ExRobot<{N}> with_cartesian_jerk: {cartesian_jerk}");
        self
    }
    fn with_rotation_velocity(&mut self, rotation_vel: f64) -> &mut Self {
        println!("\t| ExRobot<{N}> with_rotation_velocity: {rotation_vel}");
        self
    }
    fn with_rotation_acceleration(&mut self, rotation_acc: f64) -> &mut Self {
        println!("\t| ExRobot<{N}> with_rotation_acceleration: {rotation_acc}");
        self
    }
    fn with_rotation_jerk(&mut self, rotation_jerk: f64) -> &mut Self {
        println!("\t| ExRobot<{N}> with_rotation_jerk: {rotation_jerk}");
        self
    }
}

impl<const N: usize> ArmParam<N> for ExRobot<N> {
    const DH: [[f64; 4]; N] = [[0.0; 4]; N];
    const JOINT_MIN: [f64; N] = [0.0; N];
    const JOINT_MAX: [f64; N] = [1.0; N];
}

impl<const N: usize> ArmPreplannedMotionImpl<N> for ExRobot<N> {
    fn move_joint(&mut self, target: &[f64; N]) -> RobotResult<()> {
        println!("ExRobot<{N}> move_joint: {target:?}");
        print!("    ");
        self.move_joint_async(target)
    }

    fn move_joint_async(&mut self, target: &[f64; N]) -> RobotResult<()> {
        println!("ExRobot<{N}> move_joint_async: {target:?}");
        Ok(())
    }

    fn move_cartesian(&mut self, target: &Pose) -> RobotResult<()> {
        println!("ExRobot<{N}> move_cartesian: {target:?}");
        print!("    ");
        self.move_cartesian_async(target)
    }

    fn move_cartesian_async(&mut self, target: &Pose) -> RobotResult<()> {
        println!("ExRobot<{N}> move_cartesian_async: {target:?}");
        Ok(())
    }
}

impl<const N: usize> ArmPreplannedMotion<N> for ExRobot<N> {
    fn move_path(&mut self, path: Vec<MotionType<N>>) -> RobotResult<()> {
        println!("ExRobot<{N}> move_path: {path:?}");
        self.move_path_async(path)
    }
    fn move_path_async(&mut self, path: Vec<MotionType<N>>) -> RobotResult<()> {
        println!("ExRobot<{N}> move_path_async: {path:?}");
        Ok(())
    }
    fn move_path_start(&mut self, start: MotionType<N>) -> RobotResult<()> {
        println!("ExRobot<{N}> move_path_start: {start:?}");
        Ok(())
    }
    fn move_path_prepare(&mut self, path: Vec<MotionType<N>>) -> RobotResult<()> {
        println!("ExRobot<{N}> move_path_prepare: {path:?}");
        Ok(())
    }
}

impl<const N: usize> ArmStreamingHandle<N> for ExRobotHandle<N> {
    fn last_motion(&self) -> Option<MotionType<N>> {
        println!("ExRobotHandle<{N}> last_motion");
        Some(MotionType::Joint([0.0; N]))
    }
    fn move_to(&mut self, target: MotionType<N>) -> RobotResult<()> {
        println!("ExRobotHandle<{N}> move_to: {target:?}");
        Ok(())
    }
    fn last_control(&self) -> Option<ControlType<N>> {
        println!("ExRobotHandle<{N}> last_control");
        Some(ControlType::Torque([0.0; N]))
    }
    fn control_with(&mut self, control: ControlType<N>) -> RobotResult<()> {
        println!("ExRobotHandle<{N}> control_with: {control:?}");
        Ok(())
    }
}

impl<const N: usize> ArmStreamingMotion<N> for ExRobot<N> {
    type Handle = ExRobotHandle<N>;
    fn start_streaming(&mut self) -> RobotResult<Self::Handle> {
        println!("ExRobot<{N}> start_streaming");
        Ok(ExRobotHandle::<N>)
    }
    fn end_streaming(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> end_streaming");
        Ok(())
    }

    fn move_to_target(&mut self) -> Arc<Mutex<Option<MotionType<N>>>> {
        println!("ExRobot<{N}> move_to_target");
        Arc::new(Mutex::new(Some(MotionType::Joint([0.0; N]))))
    }
    fn control_with_target(&mut self) -> Arc<Mutex<Option<ControlType<N>>>> {
        println!("ExRobot<{N}> control_with_target");
        Arc::new(Mutex::new(Some(ControlType::Torque([0.0; N]))))
    }
}

impl<const N: usize> ArmStreamingMotionExt<N> for ExRobot<N> {
    fn move_joint_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>> {
        println!("ExRobot<{N}> move_joint_target");
        Arc::new(Mutex::new(Some([0.0; N])))
    }
    fn move_joint_vel_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>> {
        println!("ExRobot<{N}> move_joint_vel_target");
        Arc::new(Mutex::new(Some([0.0; N])))
    }
    fn move_joint_acc_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>> {
        println!("ExRobot<{N}> move_joint_acc_target");
        Arc::new(Mutex::new(Some([0.0; N])))
    }
    fn move_cartesian_target(&mut self) -> Arc<Mutex<Option<Pose>>> {
        println!("ExRobot<{N}> move_cartesian_target");
        Arc::new(Mutex::new(Some(Pose::default())))
    }
    fn move_cartesian_vel_target(&mut self) -> Arc<Mutex<Option<[f64; 6]>>> {
        println!("ExRobot<{N}> move_cartesian_vel_target");
        Arc::new(Mutex::new(Some([0.0; 6])))
    }
    fn move_cartesian_quat_target(&mut self) -> Arc<Mutex<Option<na::Isometry3<f64>>>> {
        println!("ExRobot<{N}> move_cartesian_quat_target");
        Arc::new(Mutex::new(Some(na::Isometry3::identity())))
    }
    fn move_cartesian_homo_target(&mut self) -> Arc<Mutex<Option<[f64; 16]>>> {
        println!("ExRobot<{N}> move_cartesian_homo_target");
        Arc::new(Mutex::new(Some([0.0; 16])))
    }
    fn move_cartesian_euler_target(&mut self) -> Arc<Mutex<Option<[f64; 6]>>> {
        println!("ExRobot<{N}> move_cartesian_euler_target");
        Arc::new(Mutex::new(Some([0.0; 6])))
    }
    fn control_tau_target(&mut self) -> Arc<Mutex<Option<[f64; N]>>> {
        println!("ExRobot<{N}> control_tau_target");
        Arc::new(Mutex::new(Some([0.0; N])))
    }
}

impl<const N: usize> Realtime for ExRobot<N> {}

impl<const N: usize> ArmRealtimeControl<N> for ExRobot<N> {
    fn move_with_closure<FM>(&mut self, mut closure: FM) -> RobotResult<()>
    where
        FM: FnMut(ArmState<N>, std::time::Duration) -> (MotionType<N>, bool) + Send + 'static,
    {
        println!(
            "closure use for default state = {:?}",
            closure(ArmState::default(), Duration::from_secs(0))
        );
        println!("ExRobot<{N}> move_with_closure");
        let mut duration = Duration::from_secs(0);
        thread::spawn(move || {
            loop {
                let (motion, finished) = closure(ArmState::default(), duration);
                println!("\t| {duration:?} | motion: {motion:?}, finished: {finished}");
                if finished {
                    break;
                }
                duration += Duration::from_millis(100);
            }
        });
        Ok(())
    }
    fn control_with_closure<FC>(&mut self, mut closure: FC) -> RobotResult<()>
    where
        FC: FnMut(ArmState<N>, std::time::Duration) -> (ControlType<N>, bool) + Send + 'static,
    {
        println!(
            "closure use for default state = {:?}",
            closure(ArmState::default(), Duration::from_secs(0))
        );
        println!("ExRobot<{N}> control_with_closure");
        let mut duration = Duration::from_secs(0);
        thread::spawn(move || {
            loop {
                let (control, finished) = closure(ArmState::default(), duration);
                println!("\t| {duration:?} | control: {control:?}, finished: {finished}");
                if finished {
                    break;
                }
                duration += Duration::from_millis(100);
            }
        });
        Ok(())
    }
}

impl<const N: usize> ArmRealtimeControlExt<N> for ExRobot<N> {}

#[cfg(test)]
mod test {
    use super::*;
    use nalgebra as na;
    use robot_behavior::{Coord, LoadState, Pose, RobotResult};

    #[test]
    fn robot_behavior() -> RobotResult<()> {
        let mut robot = ExRobot::<6>::new();
        assert_eq!(ExRobot::<6>::version(), env!("CARGO_PKG_VERSION"));
        robot.init()?;
        robot.shutdown()?;
        robot.enable()?;
        robot.disable()?;
        robot.reset()?;
        assert!(!robot.is_moving());
        robot.stop()?;
        robot.pause()?;
        robot.resume()?;
        robot.emergency_stop()?;
        robot.clear_emergency_stop()?;
        robot.read_state()?;
        Ok(())
    }

    #[test]
    fn arm_behavior() -> RobotResult<()> {
        let mut robot = ExRobot::<6>::new();

        robot.set_load(LoadState {
            m: 0.,
            x: [0.; 3],
            i: [0.; 9],
        })?;
        robot.set_coord(Coord::OCS)?;
        robot.with_coord(Coord::OCS);
        robot.set_speed(1.0)?;
        robot.with_speed(1.0);
        robot.with_velocity(&[0.0; 6]);
        robot.with_acceleration(&[0.0; 6]);
        robot.with_jerk(&[0.0; 6]);
        robot.with_cartesian_velocity(1.0);
        robot.with_cartesian_acceleration(1.0);
        robot.with_cartesian_jerk(1.0);
        robot.with_rotation_velocity(1.0);
        robot.with_rotation_acceleration(1.0);
        robot.with_rotation_jerk(1.0);
        Ok(())
    }

    #[test]
    fn arm_param() {
        let identity = na::Isometry3::identity();
        assert_eq!(
            ExRobot::<0>::forward_kinematics(&[0.; 0]),
            Pose::Quat(identity)
        );
        assert_eq!(
            ExRobot::<1>::forward_kinematics(&[0.; 1]),
            Pose::Quat(identity)
        );
        assert_eq!(
            ExRobot::<2>::forward_kinematics(&[0.; 2]),
            Pose::Quat(identity)
        );
        assert_eq!(
            ExRobot::<3>::forward_kinematics(&[0.; 3]),
            Pose::Quat(identity)
        );
        assert_eq!(
            ExRobot::<4>::forward_kinematics(&[0.; 4]),
            Pose::Quat(identity)
        );
        assert_eq!(
            ExRobot::<5>::forward_kinematics(&[0.; 5]),
            Pose::Quat(identity)
        );
        assert_eq!(
            ExRobot::<6>::forward_kinematics(&[0.; 6]),
            Pose::Quat(identity)
        );
        assert_eq!(
            ExRobot::<7>::forward_kinematics(&[0.; 7]),
            Pose::Quat(identity)
        );
    }

    #[test]
    fn arm_preplanned_motion() -> RobotResult<()> {
        let mut robot = ExRobot::<6>::new();
        robot.move_joint(&[0.0; 6])?;
        robot.move_joint_async(&[0.0; 6])?;
        robot.move_cartesian(&Pose::default())?;
        robot.move_cartesian_async(&Pose::default())?;
        robot.move_to(MotionType::Joint([0.0; 6]))?;
        robot.move_to_async(MotionType::Joint([0.0; 6]))?;
        robot.move_rel(MotionType::Joint([0.0; 6]))?;
        robot.move_rel_async(MotionType::Joint([0.0; 6]))?;
        robot.move_int(MotionType::Joint([0.0; 6]))?;
        robot.move_int_async(MotionType::Joint([0.0; 6]))?;
        robot.move_path(vec![MotionType::Joint([0.0; 6]); 6])?;
        robot.move_path_async(vec![MotionType::Joint([0.0; 6]); 6])?;
        Ok(())
    }

    #[test]
    fn py_arm_realtime_control() -> RobotResult<()> {
        let mut robot = ExRobot::<6>::new();
        let closure = |_, t: Duration| (MotionType::Joint([0.0; 6]), t.as_secs() > 1);
        robot.move_with_closure(closure)?;
        thread::sleep(Duration::from_secs(2));

        let closure = |_, t: Duration| (ControlType::Torque([0.0; 6]), t.as_secs() > 1);
        robot.control_with_closure(closure)?;
        thread::sleep(Duration::from_secs(2));
        Ok(())
    }
}
