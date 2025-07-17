use robot_behavior::{RobotResult, behavior::*};
use roplat_exrobot::ExRobot;

fn main() -> RobotResult<()> {
    println!("{}", ExRobot::<6>::version());
    Ok(())
}
