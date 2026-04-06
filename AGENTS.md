# Repository Guidelines

## Project Structure & Module Organization
This repository is a C/C++ embedded components library. Top-level folders map to functional areas:
- `bsp/`: hardware abstraction wrappers such as `can_driver` and `gpio_driver`
- `libs/`: reusable libraries, grouped by domain (`control`, `math`, `utils`, `traits`, `concurrency`)
- `protocol/`: communication helpers such as `UartRxSync`
- `services/`: higher-level runtime services like `watchdog`
- `utils/`: shared helpers used across modules

Each module usually has its own `CMakeLists.txt` and `cpkg.toml`. Prefer keeping new code inside the nearest existing module rather than adding ad hoc files at the root.

## Build, Test, and Development Commands
The root `CMakeLists.txt` sets C++17 and adds all module subdirectories.
- `cmake -S . -B build`: configure the project
- `cmake --build build`: compile all enabled modules
- `cmake --build build --target <target>`: build one library or example target if a module exposes one

There is currently no repository-wide test runner or `ctest` setup. If you add tests, wire them into CMake and document the new command here.

## Coding Style & Naming Conventions
Follow the existing embedded style:
- Use 4-space indentation in C/C++ code.
- Keep filenames and module names lowercase with underscores, for example `pid_motor`, `ring_buffer`, `gpio_driver`.
- Use `PascalCase` for types and `snake_case` for functions, variables, and files.
- Prefer header comments that explain usage and constraints, especially for ISR-safe or allocation-free code.

## Testing Guidelines
Automated tests are not currently defined. For changes in low-level drivers or control code, validate behavior by compiling the affected module and checking the generated API surface. If you add tests, name them after the module under test, such as `pid_motor_test.cpp`.

## Commit & Pull Request Guidelines
Recent commits use scoped prefixes like `feat(traits): ...`, `fix(can): ...`, and `docs(README): ...`. Follow the same pattern: `<type>(<scope>): <summary>`.

Pull requests should describe the change, list impacted modules, and mention validation performed. Include logs or screenshots only when the change affects generated output, hardware behavior, or documentation clarity.

## Agent Notes
When editing, keep changes localized to the relevant module and update the matching `CMakeLists.txt` and `cpkg.toml` if you add or rename files.
