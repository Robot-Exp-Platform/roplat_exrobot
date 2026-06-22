use robot_behavior::{
    Arm, ArmState, ArmTorqueControl, BalanceControl, BasePoseSpace, BaseState, BaseVelocityControl,
    BaseVelocitySpace, CartesianPoseControl, CartesianVelocityControl, CenterOfMassSpace,
    ControlWith, EndPoint, FlangeSpace, FootSpace, GaitCommand, GaitSpace, HandSpace, Humanoid,
    HumanoidState, JointPositionControl, JointSpace, JointState, JointVelocityControl, Joints,
    LoadState, MobileBase, MobileBaseState, MoveTo, MoveTraj, Pose, Quadruped, QuadrupedState,
    Robot, RobotDescription, RobotResult, TorqueControl, WholeBodyJointSpace, WholeBodyTorqueSpace,
    WholeBodyVelocitySpace,
};
use std::{thread, time::Duration};

#[derive(Default)]
pub struct ExRobot<const N: usize>;

#[derive(Default)]
pub struct ExRobotHandle<const N: usize>;

#[derive(Default)]
pub struct ExMobileBase;

#[derive(Default)]
pub struct ExQuadruped<const N: usize>;

#[derive(Default)]
pub struct ExHumanoid<const N: usize>;

impl<const N: usize> ExRobot<N> {
    pub fn new() -> Self {
        ExRobot
    }
}

impl ExMobileBase {
    pub fn new() -> Self {
        ExMobileBase
    }
}

impl<const N: usize> ExQuadruped<N> {
    pub fn new() -> Self {
        ExQuadruped
    }
}

impl<const N: usize> ExHumanoid<N> {
    pub fn new() -> Self {
        ExHumanoid
    }
}

fn hold_joint_position<const N: usize>(state: &JointState<N>) -> [f64; N] {
    state
        .cmd
        .q
        .or(state.des.q)
        .or(state.meas.q)
        .unwrap_or([0.0; N])
}

fn hold_joint_velocity<const N: usize>(_state: &JointState<N>) -> [f64; N] {
    [0.0; N]
}

fn hold_joint_torque<const N: usize>(state: &JointState<N>) -> [f64; N] {
    state
        .cmd
        .tau
        .or(state.des.tau)
        .or(state.meas.tau)
        .unwrap_or([0.0; N])
}

fn hold_arm_pose<const N: usize>(state: &ArmState<N>) -> Pose {
    state
        .flange
        .cmd
        .pose
        .or(state.flange.des.pose)
        .or(state.flange.meas.pose)
        .unwrap_or_default()
}

fn hold_base_velocity(_state: &BaseState) -> [f64; 6] {
    [0.0; 6]
}

impl<const N: usize> RobotDescription for ExRobot<N> {
    const URDF: Option<&'static str> = Some("");
}

impl RobotDescription for ExMobileBase {
    const URDF: Option<&'static str> = Some("");
}

impl<const N: usize> RobotDescription for ExQuadruped<N> {
    const URDF: Option<&'static str> = Some("");
}

impl<const N: usize> RobotDescription for ExHumanoid<N> {
    const URDF: Option<&'static str> = Some("");
}

impl<const N: usize> Robot for ExRobot<N> {
    type State = String;
    const CONTROL_PERIOD: f64 = 1.0;

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

    fn is_moving(&mut self) -> RobotResult<bool> {
        println!("ExRobot<{N}> is_moving");
        Ok(false)
    }

    fn waiting_for_finish(&mut self) -> RobotResult<()> {
        println!("ExRobot<{N}> waiting_for_finish");
        Ok(())
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

impl<const N: usize> Joints<N> for ExRobot<N> {
    const JOINT_MIN: [f64; N] = [0.0; N];
    const JOINT_MAX: [f64; N] = [1.0; N];
}

impl<const N: usize> EndPoint for ExRobot<N> {
    const CARTESIAN_VEL_BOUND: f64 = 1.0;
    const CARTESIAN_ACC_BOUND: f64 = 1.0;
    const ROTATION_VEL_BOUND: f64 = 1.0;
    const ROTATION_ACC_BOUND: f64 = 1.0;
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

    fn get_joint(&self) -> [f64; N] {
        [0.; N]
    }

    fn get_endpoint(&self) -> Pose {
        Pose::default()
    }

    fn with_joint_vel(self, joint_vel: [f64; N]) -> Self {
        println!("ExRobot<{N}> with_joint_vel: {joint_vel:?}");
        self
    }

    fn with_joint_acc(self, joint_acc: [f64; N]) -> Self {
        println!("ExRobot<{N}> with_joint_acc: {joint_acc:?}");
        self
    }

    fn with_joint_jerk(self, joint_jerk: [f64; N]) -> Self {
        println!("ExRobot<{N}> with_joint_jerk: {joint_jerk:?}");
        self
    }

    fn with_torque(self, torque: [f64; N]) -> Self {
        println!("ExRobot<{N}> with_torque: {torque:?}");
        self
    }

    fn with_torque_dot(self, torque_dot: [f64; N]) -> Self {
        println!("ExRobot<{N}> with_torque_dot: {torque_dot:?}");
        self
    }

    fn with_cartesian_vel(self, cartesian_vel: f64) -> Self {
        println!("ExRobot<{N}> with_cartesian_vel: {cartesian_vel}");
        self
    }

    fn with_cartesian_acc(self, cartesian_acc: f64) -> Self {
        println!("ExRobot<{N}> with_cartesian_acc: {cartesian_acc}");
        self
    }

    fn with_cartesian_jerk(self, cartesian_jerk: f64) -> Self {
        println!("ExRobot<{N}> with_cartesian_jerk: {cartesian_jerk}");
        self
    }

    fn with_rotation_vel(self, rotation_vel: f64) -> Self {
        println!("ExRobot<{N}> with_rotation_vel: {rotation_vel}");
        self
    }

    fn with_rotation_acc(self, rotation_acc: f64) -> Self {
        println!("ExRobot<{N}> with_rotation_acc: {rotation_acc}");
        self
    }

    fn with_rotation_jerk(self, rotation_jerk: f64) -> Self {
        println!("ExRobot<{N}> with_rotation_jerk: {rotation_jerk}");
        self
    }
}

impl<const N: usize> MoveTo<JointSpace<N>> for ExRobot<N> {
    fn move_to(&mut self, target: [f64; N]) -> RobotResult<()> {
        println!("ExRobot<{N}> move_to joint: {target:?}");
        Ok(())
    }
}

impl<const N: usize> MoveTo<FlangeSpace> for ExRobot<N> {
    fn move_to(&mut self, target: Pose) -> RobotResult<()> {
        println!("ExRobot<{N}> move_to flange: {target:?}");
        Ok(())
    }
}

impl<const N: usize> MoveTraj<JointSpace<N>> for ExRobot<N> {
    fn move_traj(&mut self, traj: Vec<[f64; N]>) -> RobotResult<()> {
        println!("ExRobot<{N}> move_traj joint: {traj:?}");
        Ok(())
    }

    fn move_path<F>(&mut self, _path: F) -> RobotResult<()>
    where
        F: Fn(f64) -> Option<[f64; N]>,
    {
        println!("ExRobot<{N}> move_path joint");
        Ok(())
    }

    fn move_waypoints(&mut self, waypoints: Vec<[f64; N]>) -> RobotResult<()> {
        println!("ExRobot<{N}> move_waypoints joint: {waypoints:?}");
        Ok(())
    }
}

impl<const N: usize> MoveTraj<FlangeSpace> for ExRobot<N> {
    fn move_traj(&mut self, traj: Vec<Pose>) -> RobotResult<()> {
        println!("ExRobot<{N}> move_traj flange: {traj:?}");
        Ok(())
    }

    fn move_path<F>(&mut self, _path: F) -> RobotResult<()>
    where
        F: Fn(f64) -> Option<Pose>,
    {
        println!("ExRobot<{N}> move_path flange");
        Ok(())
    }

    fn move_waypoints(&mut self, waypoints: Vec<Pose>) -> RobotResult<()> {
        println!("ExRobot<{N}> move_waypoints flange: {waypoints:?}");
        Ok(())
    }
}

fn spawn_realtime_loop<const N: usize, C, F>(mut closure: F, label: &'static str)
where
    C: std::fmt::Debug + Send + 'static,
    F: FnMut(ArmState<N>, Duration) -> (C, bool) + Send + 'static,
{
    println!("ExRobot<{N}> {label}");
    thread::spawn(move || {
        let mut duration = Duration::from_secs(0);
        loop {
            let (command, finished) = closure(ArmState::default(), duration);
            println!("\t| {duration:?} | command: {command:?}, finished: {finished}");
            if finished {
                break;
            }
            duration += Duration::from_millis(100);
        }
    });
}

fn spawn_state_realtime_loop<O, C, F>(mut closure: F, label: &'static str, obs: O)
where
    O: Clone + Send + 'static,
    C: std::fmt::Debug + Send + 'static,
    F: FnMut(O, Duration) -> (C, bool) + Send + 'static,
{
    println!("{label}");
    thread::spawn(move || {
        let mut duration = Duration::from_secs(0);
        loop {
            let (command, finished) = closure(obs.clone(), duration);
            println!("\t| {duration:?} | command: {command:?}, finished: {finished}");
            if finished {
                break;
            }
            duration += Duration::from_millis(100);
        }
    });
}

impl<const N: usize> ControlWith<TorqueControl<N>> for ExRobot<N> {
    fn hold_command(state: &JointState<N>) -> [f64; N] {
        hold_joint_torque(state)
    }

    fn control_with<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static,
    {
        spawn_state_realtime_loop(closure, "torque_control", JointState::default());
        Ok(())
    }
}

impl<const N: usize> ControlWith<ArmTorqueControl<N>> for ExRobot<N> {
    fn hold_command(state: &ArmState<N>) -> [f64; N] {
        hold_joint_torque(&state.joint)
    }

    fn control_with<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(ArmState<N>, Duration) -> ([f64; N], bool) + Send + 'static,
    {
        spawn_realtime_loop(closure, "arm_torque_control");
        Ok(())
    }
}

impl<const N: usize> ControlWith<JointPositionControl<N>> for ExRobot<N> {
    fn hold_command(state: &JointState<N>) -> [f64; N] {
        hold_joint_position(state)
    }

    fn control_with<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static,
    {
        spawn_state_realtime_loop(closure, "joint_position_control", JointState::default());
        Ok(())
    }
}

impl<const N: usize> ControlWith<JointVelocityControl<N>> for ExRobot<N> {
    fn hold_command(state: &JointState<N>) -> [f64; N] {
        hold_joint_velocity(state)
    }

    fn control_with<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static,
    {
        spawn_state_realtime_loop(closure, "joint_velocity_control", JointState::default());
        Ok(())
    }
}

impl<const N: usize> ControlWith<CartesianVelocityControl<N>> for ExRobot<N> {
    fn hold_command(_state: &ArmState<N>) -> [f64; 6] {
        [0.; 6]
    }

    fn control_with<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(ArmState<N>, Duration) -> ([f64; 6], bool) + Send + 'static,
    {
        spawn_realtime_loop(closure, "cartesian_velocity_control");
        Ok(())
    }
}

impl<const N: usize> ControlWith<CartesianPoseControl<N>> for ExRobot<N> {
    fn hold_command(state: &ArmState<N>) -> Pose {
        hold_arm_pose(state)
    }

    fn control_with<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(ArmState<N>, Duration) -> (Pose, bool) + Send + 'static,
    {
        spawn_realtime_loop(closure, "cartesian_pose_control");
        Ok(())
    }
}

impl Robot for ExMobileBase {
    type State = MobileBaseState;
    const CONTROL_PERIOD: f64 = 0.01;

    fn version() -> String {
        format!("ExMobileBase v{}", env!("CARGO_PKG_VERSION"))
    }

    fn read_state(&mut self) -> RobotResult<Self::State> {
        println!("ExMobileBase read_state");
        Ok(MobileBaseState::default())
    }
}

impl MobileBase for ExMobileBase {
    fn base_state(&mut self) -> RobotResult<BaseState> {
        println!("ExMobileBase base_state");
        Ok(BaseState::default())
    }
}

impl MoveTo<BasePoseSpace> for ExMobileBase {
    fn move_to(&mut self, target: Pose) -> RobotResult<()> {
        println!("ExMobileBase move_to base pose: {target:?}");
        Ok(())
    }
}

impl MoveTraj<BasePoseSpace> for ExMobileBase {
    fn move_traj(&mut self, traj: Vec<Pose>) -> RobotResult<()> {
        println!("ExMobileBase move_traj base pose: {traj:?}");
        Ok(())
    }

    fn move_path<F>(&mut self, _path: F) -> RobotResult<()>
    where
        F: Fn(f64) -> Option<Pose>,
    {
        println!("ExMobileBase move_path base pose");
        Ok(())
    }

    fn move_waypoints(&mut self, waypoints: Vec<Pose>) -> RobotResult<()> {
        println!("ExMobileBase move_waypoints base pose: {waypoints:?}");
        Ok(())
    }
}

impl MoveTo<BaseVelocitySpace> for ExMobileBase {
    fn move_to(&mut self, target: [f64; 6]) -> RobotResult<()> {
        println!("ExMobileBase move_to base velocity: {target:?}");
        Ok(())
    }
}

impl MoveTraj<BaseVelocitySpace> for ExMobileBase {
    fn move_traj(&mut self, traj: Vec<[f64; 6]>) -> RobotResult<()> {
        println!("ExMobileBase move_traj base velocity: {traj:?}");
        Ok(())
    }

    fn move_path<F>(&mut self, _path: F) -> RobotResult<()>
    where
        F: Fn(f64) -> Option<[f64; 6]>,
    {
        println!("ExMobileBase move_path base velocity");
        Ok(())
    }

    fn move_waypoints(&mut self, waypoints: Vec<[f64; 6]>) -> RobotResult<()> {
        println!("ExMobileBase move_waypoints base velocity: {waypoints:?}");
        Ok(())
    }
}

impl ControlWith<BaseVelocityControl> for ExMobileBase {
    fn hold_command(state: &BaseState) -> [f64; 6] {
        hold_base_velocity(state)
    }

    fn control_with<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(BaseState, Duration) -> ([f64; 6], bool) + Send + 'static,
    {
        spawn_state_realtime_loop(
            closure,
            "ExMobileBase base_velocity_control",
            BaseState::default(),
        );
        Ok(())
    }
}

impl ControlWith<BalanceControl> for ExMobileBase {
    fn hold_command(state: &BaseState) -> [f64; 6] {
        hold_base_velocity(state)
    }

    fn control_with<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(BaseState, Duration) -> ([f64; 6], bool) + Send + 'static,
    {
        spawn_state_realtime_loop(
            closure,
            "ExMobileBase balance_control",
            BaseState::default(),
        );
        Ok(())
    }
}

impl<const N: usize> Robot for ExQuadruped<N> {
    type State = QuadrupedState<N>;
    const CONTROL_PERIOD: f64 = 0.002;

    fn version() -> String {
        format!("ExQuadruped<{N}> v{}", env!("CARGO_PKG_VERSION"))
    }

    fn read_state(&mut self) -> RobotResult<Self::State> {
        println!("ExQuadruped<{N}> read_state");
        Ok(QuadrupedState::default())
    }
}

impl<const N: usize> Joints<N> for ExQuadruped<N> {
    const JOINT_MIN: [f64; N] = [-1.0; N];
    const JOINT_MAX: [f64; N] = [1.0; N];
}

impl<const N: usize> Quadruped<N> for ExQuadruped<N> {
    fn state(&mut self) -> RobotResult<QuadrupedState<N>> {
        println!("ExQuadruped<{N}> state");
        Ok(QuadrupedState::default())
    }
}

impl<const N: usize> MoveTo<GaitSpace> for ExQuadruped<N> {
    fn move_to(&mut self, target: GaitCommand) -> RobotResult<()> {
        println!("ExQuadruped<{N}> move_to gait: {target:?}");
        Ok(())
    }
}

impl<const N: usize> MoveTo<WholeBodyJointSpace<N>> for ExQuadruped<N> {
    fn move_to(&mut self, target: [f64; N]) -> RobotResult<()> {
        println!("ExQuadruped<{N}> move_to whole-body joint: {target:?}");
        Ok(())
    }
}

impl<const N: usize> MoveTo<WholeBodyVelocitySpace<N>> for ExQuadruped<N> {
    fn move_to(&mut self, target: [f64; N]) -> RobotResult<()> {
        println!("ExQuadruped<{N}> move_to whole-body velocity: {target:?}");
        Ok(())
    }
}

impl<const N: usize> MoveTo<WholeBodyTorqueSpace<N>> for ExQuadruped<N> {
    fn move_to(&mut self, target: [f64; N]) -> RobotResult<()> {
        println!("ExQuadruped<{N}> move_to whole-body torque: {target:?}");
        Ok(())
    }
}

impl<const N: usize, const LEG: usize> MoveTo<FootSpace<LEG>> for ExQuadruped<N> {
    fn move_to(&mut self, target: Pose) -> RobotResult<()> {
        println!("ExQuadruped<{N}> move_to foot {LEG}: {target:?}");
        Ok(())
    }
}

impl<const N: usize> MoveTraj<WholeBodyJointSpace<N>> for ExQuadruped<N> {
    fn move_traj(&mut self, traj: Vec<[f64; N]>) -> RobotResult<()> {
        println!("ExQuadruped<{N}> move_traj whole-body joint: {traj:?}");
        Ok(())
    }

    fn move_path<F>(&mut self, _path: F) -> RobotResult<()>
    where
        F: Fn(f64) -> Option<[f64; N]>,
    {
        println!("ExQuadruped<{N}> move_path whole-body joint");
        Ok(())
    }

    fn move_waypoints(&mut self, waypoints: Vec<[f64; N]>) -> RobotResult<()> {
        println!("ExQuadruped<{N}> move_waypoints whole-body joint: {waypoints:?}");
        Ok(())
    }
}

impl<const N: usize> ControlWith<TorqueControl<N>> for ExQuadruped<N> {
    fn hold_command(state: &JointState<N>) -> [f64; N] {
        hold_joint_torque(state)
    }

    fn control_with<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static,
    {
        spawn_state_realtime_loop(
            closure,
            "ExQuadruped whole_body_torque_control",
            JointState::default(),
        );
        Ok(())
    }
}

impl<const N: usize> ControlWith<JointPositionControl<N>> for ExQuadruped<N> {
    fn hold_command(state: &JointState<N>) -> [f64; N] {
        hold_joint_position(state)
    }

    fn control_with<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static,
    {
        spawn_state_realtime_loop(
            closure,
            "ExQuadruped whole_body_position_control",
            JointState::default(),
        );
        Ok(())
    }
}

impl<const N: usize> ControlWith<JointVelocityControl<N>> for ExQuadruped<N> {
    fn hold_command(state: &JointState<N>) -> [f64; N] {
        hold_joint_velocity(state)
    }

    fn control_with<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static,
    {
        spawn_state_realtime_loop(
            closure,
            "ExQuadruped whole_body_velocity_control",
            JointState::default(),
        );
        Ok(())
    }
}

impl<const N: usize> ControlWith<BaseVelocityControl> for ExQuadruped<N> {
    fn hold_command(state: &BaseState) -> [f64; 6] {
        hold_base_velocity(state)
    }

    fn control_with<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(BaseState, Duration) -> ([f64; 6], bool) + Send + 'static,
    {
        spawn_state_realtime_loop(
            closure,
            "ExQuadruped base_velocity_control",
            BaseState::default(),
        );
        Ok(())
    }
}

impl<const N: usize> ControlWith<BalanceControl> for ExQuadruped<N> {
    fn hold_command(state: &BaseState) -> [f64; 6] {
        hold_base_velocity(state)
    }

    fn control_with<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(BaseState, Duration) -> ([f64; 6], bool) + Send + 'static,
    {
        spawn_state_realtime_loop(closure, "ExQuadruped balance_control", BaseState::default());
        Ok(())
    }
}

impl<const N: usize> Robot for ExHumanoid<N> {
    type State = HumanoidState<N>;
    const CONTROL_PERIOD: f64 = 0.002;

    fn version() -> String {
        format!("ExHumanoid<{N}> v{}", env!("CARGO_PKG_VERSION"))
    }

    fn read_state(&mut self) -> RobotResult<Self::State> {
        println!("ExHumanoid<{N}> read_state");
        Ok(HumanoidState::default())
    }
}

impl<const N: usize> Joints<N> for ExHumanoid<N> {
    const JOINT_MIN: [f64; N] = [-1.0; N];
    const JOINT_MAX: [f64; N] = [1.0; N];
}

impl<const N: usize> Humanoid<N> for ExHumanoid<N> {
    fn state(&mut self) -> RobotResult<HumanoidState<N>> {
        println!("ExHumanoid<{N}> state");
        Ok(HumanoidState::default())
    }
}

impl<const N: usize> MoveTo<WholeBodyJointSpace<N>> for ExHumanoid<N> {
    fn move_to(&mut self, target: [f64; N]) -> RobotResult<()> {
        println!("ExHumanoid<{N}> move_to whole-body joint: {target:?}");
        Ok(())
    }
}

impl<const N: usize> MoveTo<WholeBodyVelocitySpace<N>> for ExHumanoid<N> {
    fn move_to(&mut self, target: [f64; N]) -> RobotResult<()> {
        println!("ExHumanoid<{N}> move_to whole-body velocity: {target:?}");
        Ok(())
    }
}

impl<const N: usize> MoveTo<WholeBodyTorqueSpace<N>> for ExHumanoid<N> {
    fn move_to(&mut self, target: [f64; N]) -> RobotResult<()> {
        println!("ExHumanoid<{N}> move_to whole-body torque: {target:?}");
        Ok(())
    }
}

impl<const N: usize> MoveTo<CenterOfMassSpace> for ExHumanoid<N> {
    fn move_to(&mut self, target: [f64; 3]) -> RobotResult<()> {
        println!("ExHumanoid<{N}> move_to center of mass: {target:?}");
        Ok(())
    }
}

impl<const N: usize, const HAND: usize> MoveTo<HandSpace<HAND>> for ExHumanoid<N> {
    fn move_to(&mut self, target: Pose) -> RobotResult<()> {
        println!("ExHumanoid<{N}> move_to hand {HAND}: {target:?}");
        Ok(())
    }
}

impl<const N: usize, const LEG: usize> MoveTo<FootSpace<LEG>> for ExHumanoid<N> {
    fn move_to(&mut self, target: Pose) -> RobotResult<()> {
        println!("ExHumanoid<{N}> move_to foot {LEG}: {target:?}");
        Ok(())
    }
}

impl<const N: usize> MoveTraj<WholeBodyJointSpace<N>> for ExHumanoid<N> {
    fn move_traj(&mut self, traj: Vec<[f64; N]>) -> RobotResult<()> {
        println!("ExHumanoid<{N}> move_traj whole-body joint: {traj:?}");
        Ok(())
    }

    fn move_path<F>(&mut self, _path: F) -> RobotResult<()>
    where
        F: Fn(f64) -> Option<[f64; N]>,
    {
        println!("ExHumanoid<{N}> move_path whole-body joint");
        Ok(())
    }

    fn move_waypoints(&mut self, waypoints: Vec<[f64; N]>) -> RobotResult<()> {
        println!("ExHumanoid<{N}> move_waypoints whole-body joint: {waypoints:?}");
        Ok(())
    }
}

impl<const N: usize> ControlWith<TorqueControl<N>> for ExHumanoid<N> {
    fn hold_command(state: &JointState<N>) -> [f64; N] {
        hold_joint_torque(state)
    }

    fn control_with<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static,
    {
        spawn_state_realtime_loop(
            closure,
            "ExHumanoid whole_body_torque_control",
            JointState::default(),
        );
        Ok(())
    }
}

impl<const N: usize> ControlWith<JointPositionControl<N>> for ExHumanoid<N> {
    fn hold_command(state: &JointState<N>) -> [f64; N] {
        hold_joint_position(state)
    }

    fn control_with<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static,
    {
        spawn_state_realtime_loop(
            closure,
            "ExHumanoid whole_body_position_control",
            JointState::default(),
        );
        Ok(())
    }
}

impl<const N: usize> ControlWith<JointVelocityControl<N>> for ExHumanoid<N> {
    fn hold_command(state: &JointState<N>) -> [f64; N] {
        hold_joint_velocity(state)
    }

    fn control_with<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<N>, Duration) -> ([f64; N], bool) + Send + 'static,
    {
        spawn_state_realtime_loop(
            closure,
            "ExHumanoid whole_body_velocity_control",
            JointState::default(),
        );
        Ok(())
    }
}

impl<const N: usize> ControlWith<BalanceControl> for ExHumanoid<N> {
    fn hold_command(state: &BaseState) -> [f64; 6] {
        hold_base_velocity(state)
    }

    fn control_with<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(BaseState, Duration) -> ([f64; 6], bool) + Send + 'static,
    {
        spawn_state_realtime_loop(closure, "ExHumanoid balance_control", BaseState::default());
        Ok(())
    }
}
