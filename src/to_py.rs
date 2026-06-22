use crate::exrobot::ExRobot;
use pyo3::{pyclass, pymethods};
use robot_behavior::{
    py_arm, py_arm_torque_control, py_cartesian_pose_control, py_cartesian_velocity_control,
    py_flange_motion, py_joint_motion, py_joint_position_control, py_joint_torque_control,
    py_joint_velocity_control, py_robot,
};

type ExRobot6 = ExRobot<6>;
#[pyclass(name = "ExRobot")]
pub struct PyExRobot(ExRobot6);

#[pymethods]
impl PyExRobot {
    #[new]
    fn new() -> Self {
        PyExRobot(ExRobot::new())
    }
}

py_robot!(PyExRobot(ExRobot6));
py_arm!(PyExRobot<{6}>(ExRobot6));
py_joint_motion!(PyExRobot<{6}>(ExRobot6));
py_flange_motion!(PyExRobot(ExRobot6));
py_joint_position_control!(PyExRobot<{6}>(ExRobot6));
py_joint_velocity_control!(PyExRobot<{6}>(ExRobot6));
py_cartesian_velocity_control!(PyExRobot<{6}>(ExRobot6));
py_cartesian_pose_control!(PyExRobot<{6}>(ExRobot6));
py_joint_torque_control!(PyExRobot<{6}>(ExRobot6));
py_arm_torque_control!(PyExRobot<{6}>(ExRobot6));
