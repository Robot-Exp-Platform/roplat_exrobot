from robot_behavior import (
    Arm,
    ArmState,
    ArmTorqueControl,
    CartesianPoseControl,
    CartesianVelocityControl,
    FlangeMotion,
    FlangeTrajectory,
    JointMotion,
    JointPositionControl,
    JointSample,
    JointState,
    JointTorqueControl,
    JointVelocityControl,
    LoadState,
    MotionType,
    Pose,
    SpatialSample,
    SpatialState,
    Vec,
)


class ExRobot(
    Arm,
    JointMotion,
    FlangeMotion,
    FlangeTrajectory,
    JointPositionControl,
    JointVelocityControl,
    JointTorqueControl,
    ArmTorqueControl,
    CartesianVelocityControl,
    CartesianPoseControl,
):
    def __init__(self) -> None: ...
