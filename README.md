
## Control-session exit contract

Example robots implement `ControlWith::control_with_flow`: return
`ControlFlow::Continue((command, done))` for a valid command and
`ControlFlow::Break(())` to finish without producing a command for that cycle.
The existing `control_with` and `control_with_async` tuple callbacks remain
convenience wrappers. The async callback API still blocks for the control session;
it does not spawn a background task. These are console examples, not hardware
watchdog or emergency-stop implementations.

Run the finite contract tests with `cargo test -p roplat_exrobot --test control_flow`.

## Independent source checkout

This internal development baseline pins `robot_behavior` to a specific GitHub
commit because its current 0.6 API has not been published on crates.io. The
manifest is complete and does not inherit dependencies from a parent drives
workspace. Use an authorized SSH key/agent and
`CARGO_NET_GIT_FETCH_WITH_CLI=true` when building from a standalone checkout;
no sibling `robot_behavior` or `roplat` directory is required. Native driver
builds do not enable the optional roplat framework adapter.
