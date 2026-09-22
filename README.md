
## Control-session exit contract

Example robots implement `ControlWith::control_with_flow`: return
`ControlFlow::Continue((command, done))` for a valid command and
`ControlFlow::Break(())` to finish without producing a command for that cycle.
The existing `control_with` and `control_with_async` tuple callbacks remain
convenience wrappers. The async callback API still blocks for the control session;
it does not spawn a background task. These are console examples, not hardware
watchdog or emergency-stop implementations.

Run the finite contract tests with `cargo test -p roplat_exrobot --test control_flow`.
