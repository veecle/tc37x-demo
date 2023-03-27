## Demo code for CAN over AURIX TC375

This demo provides an initial showcase on how to use Rust on AURIX™. The demo is 100% written in Rust, and provides 
1. Board clock configuration and distribution
2. CAN Driver configuration & basic usage scenarios
3. Basic stack-traces
4. [defmt] logging through [tricore-probe]

**Please, be aware that the code presented here is very experimental and only meant to showcase; by no means one should consider this finished or correct**

## Usage
To initialize: `git clone --recurse-submodules https://github.com/veecle/tc37x-demo` 

To compile the demo [HighTec Rust](https://hightec-rt.com/en/products/rust)'s compiler should be installed. 
Please register and follow their installation instruction.

Following, the demo can be compiled via `cd app && cargo build` and subsequently flashed via `cargo run` over `tricore-probe`.

We use [defmt] as our logging framework: by setting the `DEFMT_LOG={TRACE, DEBUG, INFO, WARN, ERROR}` the probe will do logging (for more details refer to the official project).  

#### Example code

The `main.rs` comes with two examples:
- `can_with_pins`: (**disabled** by default) assumes two connected devices (a joystick and a motor controller) that talks over CAN. This is unlikely to work given the strict setup; however, the code can be modified to accommodate other scenarios.
- `with_with_loopback`: (**enabled** by default) this makes CAN work in loopback. This runs by default and should give an idea on how sending/receiving work. 

#### VSCode
In order to use rust-analyzer in VSCode, the `tricore.core-workspace.RLM_LICENSE` should be updated to point to the license path.

#### License
Licensed under <a href="LICENSE">Apache License, Version 2.0</a>

[defmt]: https://github.com/knurling-rs/defmt
[tricore-probe]: https://github.com/veecle/tricore-probe