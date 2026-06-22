use std::time::Duration;

use robot_behavior::{RobotResult, behavior::*};
use roplat_exrobot::{ExHumanoid, ExMobileBase, ExQuadruped, ExRobot};

fn main() -> RobotResult<()> {
    arm_capabilities()?;
    mobile_base_capabilities()?;
    quadruped_capabilities()?;
    humanoid_capabilities()?;
    Ok(())
}

fn arm_capabilities() -> RobotResult<()> {
    let mut arm = ExRobot::<6>::new();
    arm.init()?;
    arm.enable()?;
    arm.move_to::<JointSpace<6>>([0.1; 6])?;
    arm.move_to::<FlangeSpace>(Pose::Position([0.2, 0.0, 0.3]))?;
    arm.move_traj::<JointSpace<6>>(vec![[0.0; 6], [0.2; 6]])?;
    arm.control_with::<JointPositionControl<6>, _>(|_, dt| {
        ([dt.as_secs_f64(); 6], dt > Duration::from_millis(200))
    })?;
    arm.control_with::<TorqueControl<6>, _>(|_, dt| ([0.0; 6], dt > Duration::from_millis(200)))?;
    arm.control_with::<CartesianVelocityControl<6>, _>(|_, dt| {
        ([0.0; 6], dt > Duration::from_millis(200))
    })?;
    arm.control_with::<CartesianPoseControl<6>, _>(|_, dt| {
        (
            Pose::Position([0.0, 0.0, 0.1]),
            dt > Duration::from_millis(200),
        )
    })?;
    Ok(())
}

fn mobile_base_capabilities() -> RobotResult<()> {
    let mut base = ExMobileBase::new();
    base.move_to::<BasePoseSpace>(Pose::Position([1.0, 0.0, 0.0]))?;
    base.move_to::<BaseVelocitySpace>([0.2, 0.0, 0.0, 0.0, 0.0, 0.1])?;
    base.control_with::<BaseVelocityControl, _>(|_, dt| {
        ([0.0; 6], dt > Duration::from_millis(200))
    })?;
    Ok(())
}

fn quadruped_capabilities() -> RobotResult<()> {
    let mut dog = ExQuadruped::<12>::new();
    dog.move_to::<GaitSpace>(GaitCommand::Stand)?;
    dog.move_to::<WholeBodyJointSpace<12>>([0.1; 12])?;
    dog.move_to::<FootSpace<0>>(Pose::Position([0.2, 0.1, -0.3]))?;
    dog.control_with::<TorqueControl<12>, _>(|_, dt| ([0.0; 12], dt > Duration::from_millis(200)))?;
    Ok(())
}

fn humanoid_capabilities() -> RobotResult<()> {
    let mut humanoid = ExHumanoid::<30>::new();
    humanoid.move_to::<WholeBodyJointSpace<30>>([0.0; 30])?;
    humanoid.move_to::<CenterOfMassSpace>([0.0, 0.0, 0.9])?;
    humanoid.move_to::<HandSpace<0>>(Pose::Position([0.4, 0.2, 1.0]))?;
    humanoid.move_to::<FootSpace<0>>(Pose::Position([0.1, 0.1, 0.0]))?;
    humanoid
        .control_with::<BalanceControl, _>(|_, dt| ([0.0; 6], dt > Duration::from_millis(200)))?;
    Ok(())
}
