use crate::exrobot::{ExRobot, ExStreamHandle};
use pyo3::{pyclass, pymethods, types::PyAnyMethods};
use robot_behavior::{
    behavior::*, py_arm_behavior, py_arm_param, py_arm_preplanned_motion,
    py_arm_preplanned_motion_ext, py_arm_preplanned_motion_impl, py_arm_real_time_control,
    py_arm_real_time_control_ext, py_arm_streaming_handle, py_arm_streaming_motion,
    py_arm_streaming_motion_ext, py_robot_behavior,
};

type ExRobot6 = ExRobot<6>;
type ExStreamHandle6 = ExStreamHandle<6>;

#[pyclass(name = "ExRobot")]
pub struct PyExRobot(ExRobot6);
#[pyclass(name = "ExStreamHandle")]
pub struct PyExStreamHandle(ExStreamHandle6);

#[pymethods]
impl PyExRobot {
    #[new]
    fn new() -> Self {
        PyExRobot(ExRobot::new())
    }
}

impl From<ExStreamHandle6> for PyExStreamHandle {
    fn from(handle: ExStreamHandle6) -> Self {
        PyExStreamHandle(handle)
    }
}

py_robot_behavior!(PyExRobot(ExRobot6));
py_arm_behavior!(PyExRobot<{6}>(ExRobot6));
py_arm_param!(PyExRobot<{6}>(ExRobot6));
py_arm_preplanned_motion!(PyExRobot<{6}>(ExRobot6));
py_arm_preplanned_motion_impl!(PyExRobot<{6}>(ExRobot6));
py_arm_preplanned_motion_ext!(PyExRobot<{6}>(ExRobot6));
py_arm_streaming_handle!(PyExStreamHandle<{6}>(ExStreamHandle6));
py_arm_streaming_motion!(PyExRobot<{6}>(ExRobot6) -> PyExStreamHandle);
py_arm_streaming_motion_ext!(PyExRobot<{6}>(ExRobot6));
py_arm_real_time_control!(PyExRobot<{6}>(ExRobot6));
py_arm_real_time_control_ext!(PyExRobot<{6}>(ExRobot6));
