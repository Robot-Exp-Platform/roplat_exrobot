use robot_behavior::behavior::*;
use roplat_exrobot::ExRobot;

fn main() {
    println!("{}", ExRobot::<6>::version());
}
