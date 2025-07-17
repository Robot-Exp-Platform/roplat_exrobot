pub mod exrobot;
#[cfg(feature = "to_py")]
pub mod to_py;

pub use exrobot::{ExRobot, ExStreamHandle};

#[pyo3::pymodule]
mod roplat_exrobot {
    #[pymodule_export]
    use super::to_py::{PyExRobot, PyExStreamHandle};
    #[pymodule_export]
    use robot_behavior::{LoadState, PyArmState, PyControlType, PyMotionType, PyPose};
}
