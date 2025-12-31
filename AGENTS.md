# Agent Guidelines

- When adding a new feature, place the implementation in a subfolder of `src/`, creating the subfolder if necessary, instead of the ros-based `nodes/` folder
- Avoid simply appending large amounts of code into existing files (often node files); prefer organizing new functionality into appropriately named subdirectories.
- Using ROS dependencies outside of the `nodes/` folder is barely acceptable but should be minimized when possible.
- **Keep new classes generic and reusable**: put algorithm/processing code in `src/<domain>/...` with **no ROS types/logging**; expose a small, ROS-agnostic API (e.g., `Update(sample, dt)`, `GetBias()`, `Reset()`). Only the ROS `nodes/` layer should translate to/from messages, manage parameters/QoS, and do `RCLCPP_*` logging. Also if theres ros stuff needed then `ros/` is appropriate.
- **Keep new classes generic**: prefer small, domain-agnostic utility classes (math/filters/estimators/parsers) that can be reused across drivers and nodes; keep ROS glue thin and localized.
- **Separate “core” from “integration”**: put algorithms/state machines/data transforms in `src/<area>/...` with no ROS includes when possible; have `nodes/` only adapt parameters/topics/timers to the core API.
- **Stable, message-driven APIs**: design core classes around plain inputs/outputs (structs, spans, arrays) that map cleanly to messages; avoid parameter-heavy constructors or ROS parameter lookups inside the core.
- **Document public structs/interfaces**: any public `struct`/`enum` must have brief field docs (meaning + units + expected range + how it’s computed/used), so downstream users can treat it as a contract.
- **Minimize hidden coupling**: avoid implicit dependencies on global state/time/ROS clocks; pass time stamps and configuration explicitly.
- **Document mathematical parameters**: any constant/denominator/derived factor in algorithms must be documented with its meaning or derivation. Math explanations can be dense, so try to use comment style and whitespace to optimize readability.
