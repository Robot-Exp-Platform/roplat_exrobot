# roplat_exrobot

Console-only reference robots for learning the **`robot_behavior` capability
interfaces**. Use them to try typed motion calls, write generic application code,
or understand what a real driver needs to implement before connecting hardware.

The objects print operations and return illustrative state. They do not connect
to a robot, integrate dynamics, or simulate a commanded trajectory. The name
does not imply a dependency on the roplat execution framework: this crate's
primary abstraction is a set of `robot_behavior` trait implementations.

## Design: capabilities describe what a robot can do

`Robot` provides common lifecycle and state access. More specific traits describe
the available motion spaces and control callbacks. A space such as
`JointSpace<6>` gives `move_to` a six-element joint target; a control marker such
as `JointPositionControl<6>` gives a callback its observation and command types.
The joint count is part of the Rust type, so a six-joint arm is `ExRobot<6>`.

| Example object | Interfaces illustrated |
|---|---|
| `ExRobot<N>` | Arm state, joint/flange moves and trajectories, position/velocity/torque callbacks |
| `ExMobileBase` | Base pose/velocity moves and base control callbacks |
| `ExQuadruped<N>` | Gait and whole-body/foot targets, joint and base callbacks |
| `ExHumanoid<N>` | Whole-body, center-of-mass, hand/foot targets and balance callbacks |

Only the traits implemented by an object are available. This makes the examples
useful for learning capability-based APIs without treating every kind of robot
as an arm.

## Install

The current published version is `0.2.0`. Add these dependencies to a Rust 2024
project; `robot_behavior` is explicit because the application imports its traits:

```toml
[dependencies]
roplat_exrobot = "0.2.0"
robot_behavior = "0.6.1"
```

Use **Rust nightly**: the crate and its behavior dependency use
`generic_const_exprs`. The default Rust API needs no robot SDK, device, or network
connection. A normal platform C++ build toolchain is needed by the dependency
builds; Windows users need MSVC Build Tools and the Windows SDK.

## First example: a finite control session

Put this complete program in `src/main.rs`, then run `cargo +nightly run`.
All operations below are console demonstrations.

```rust
use robot_behavior::{behavior::*, RobotResult};
use roplat_exrobot::ExRobot;

fn main() -> RobotResult<()> {
    let mut robot = ExRobot::<6>::new();
    robot.init()?;
    robot.enable()?;
    println!("{}", robot.read_state()?);

    robot.move_to::<JointSpace<6>>([0.1; 6])?;
    let mut cycles = 0;
    robot.control_with::<JointPositionControl<6>, _>(|_state, elapsed| {
        cycles += 1;
        println!("cycle {cycles}, synthetic time {elapsed:?}");
        ([0.2; 6], cycles == 3)
    })?;

    robot.disable()?;
    robot.shutdown()?;
    Ok(())
}
```

The output shows lifecycle calls, one joint target, and three callback commands.
The tuple means `(command, done)`: the final command is handled before the session
finishes. Callback time starts at zero and advances by a synthetic 100 ms per
iteration; the example loop does not sleep or enforce a physical control rate.
The state supplied to callbacks is a default value and is not updated by commands.

## Object lifetime and session completion

`new()` constructs a local value without attaching to anything. `ExRobot`'s
`init`, `enable`, `disable`, and `shutdown` methods print their operation; they
do not enforce a hardware lifecycle. There is no device connection to recover
or automatically shut down when the value is dropped.

For a callback that should exit **without a command on that iteration**, use
`ControlWith::control_with_flow` and return `ControlFlow::Break(())`.
`ControlFlow::Continue((command, true))` instead completes after its final
command. The async callback variants still occupy the calling control session;
they do not launch an independently running controller. A callback must eventually
finish or break, otherwise these illustrative loops continue indefinitely.

`ExRobot::read_state()` returns a descriptive string. The separate arm
`state()` method returns a default `ArmState<N>`. Mobile-base, quadruped, and
humanoid `read_state()` methods return their corresponding typed default states.
These values help exercise interface shape, not observe simulated motion.

## Next steps

Read [capability_showcase.rs](examples/capability_showcase.rs) for complete arm,
mobile-base, quadruped, and humanoid examples. Use
[the trait implementations](src/exrobot.rs) as a reference when designing a
driver, then choose a real backend for device communication or a simulator for
physical state evolution. The example `stop` and `emergency_stop` methods only
print; their names do not provide a physical watchdog or emergency stop.

Optional features demonstrate language bindings:

| Feature | Purpose and additional requirements |
|---|---|
| Default: none | Rust reference objects |
| `to_py` | PyO3 bindings for the six-joint `ExRobot`; Python and an extension-module packaging tool such as maturin |
| `to_cxx` | CXX bridge for the six-joint `ExRobot`; a supported C++ toolchain |
| `ffi` | Shared binding marker; does not export a language API by itself |

See [the Python binding](src/to_py.rs), [the C++ bridge](src/to_cxx.rs), and
[the C++ wrapper](roplat_exrobot.hpp). The additional example robot families are
Rust APIs; these bindings currently wrap `ExRobot<6>`.

## Common questions

- **Methods such as `move_to` are not found:** import
  `robot_behavior::behavior::*`; the public convenience methods come from traits.
- **Stable Rust rejects the build:** select the nightly toolchain rather than
  suppressing feature checks.
- **Why does a successful motion call leave state unchanged?** Commands are
  printed, not simulated. Use a dynamics backend when motion feedback matters.
- **Why does building this repository ask for SSH access?** The source manifest
  pins `robot_behavior` to a GitHub SSH revision. Configure your existing SSH
  key/agent and use `CARGO_NET_GIT_FETCH_WITH_CLI=true` as needed. The published
  crates.io package uses a registry dependency instead; neither route needs a
  sibling checkout. When using the Git version in an application, import the
  behavior traits from that same pinned Git source to avoid two distinct copies
  of the trait definitions.

## More information and license

- [Version example](examples/version.rs)
- [Capability examples](examples/capability_showcase.rs)
- [robot_behavior](https://github.com/Robot-Exp-Platform/robot_behavior)

Licensed under [Apache-2.0](LICENSE).
