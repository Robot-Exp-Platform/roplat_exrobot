pub mod devices;
pub mod exrobot;
#[cfg(feature = "to_cxx")]
pub mod to_cxx;
#[cfg(feature = "to_py")]
pub mod to_py;

pub use devices::{ExForceTorque, ExGripper, ExImu};
pub use exrobot::{ExHumanoid, ExMobileBase, ExQuadruped, ExRobot, ExRobotHandle};

#[cfg(feature = "to_py")]
#[pyo3::pymodule]
mod roplat_exrobot {
    #[pymodule_export]
    use super::to_py::PyExRobot;
    #[pymodule_export]
    use roplat_behavior_ffi::{
        PyArmState, PyJointSample, PyJointState, PyLoadState, PyMotionType, PyPose,
        PySpatialSample, PySpatialState,
    };
}
