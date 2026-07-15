# CCS Agent Guidelines

CCStudio is installed at `D:/TI/CCS`.

Before any CCS or Texas Instruments task, read:

```text
D:/TI/CCS/ccs/theia/resources/ai/CCS.md
```

Use these paths:

- MSPM0 SDK: `D:/TI/SDK/mspm0_sdk_2_10_00_04`
- SysConfig 1.28.0: `D:/TI/CCS/ccs/utils/sysconfig_1.28.0`
- Projects: `D:/TI/Projects/elec-competition/MSPM0G3507/Projects`

Keep CCS project names and paths in ASCII. Do not commit generated `Debug/` or
`Release/` directories.

## Complete Project Requirement

Do not create source-only skeletons for new MSPM0/CCS firmware projects.
Every new project intended for board work must be a complete CCS project before
it is handed to the user.

A complete CCS MSPM0 project must include:

- ASCII-only project name and path.
- CCS project metadata created or maintained by CCS Project tools:
  `.project`, `.cproject`, `.ccsproject`, and `targetConfigs/*.ccxml` when the
  project uses CCS debug/flash.
- A real `.syscfg` created or maintained by SysConfig for the selected MSPM0
  device, package, SDK, clocks, pins, peripheral instances, and interrupts.
- Generated `ti_msp_dl_config.c/h` produced by SysConfig during generation or
  build. Inspect generated files for macro names, but never hand-edit them.
- A minimal safe application entry point such as `empty.c` or `main.c` that
  calls the generated `SYSCFG_DL_init()` and can be compiled, flashed, and
  tested independently before higher-level logic is enabled.
- Required include paths, source entries, linked resources, linker settings,
  and product references configured through CCS/SysConfig tools, not by manual
  XML editing.
- A documented bring-up test order, starting with LED/GPIO and adding
  peripherals one at a time.

Before claiming a new project is ready, verify and report:

1. Source-level checks.
2. SysConfig generation/check result and warnings.
3. CCS Project build result using CCS tooling.
4. Flash/download result, only when the user has approved flashing.
5. Real hardware behavior, only when it was actually tested.

If CCS/SysConfig tools are unavailable, stop and say the project cannot yet be
completed as a full CCS project. Do not present a source folder, pseudo project,
or hand-written metadata as a complete CCS project.

## Reusable Module Policy

This repository is a shared firmware codebase. Before writing a new driver,
control helper, protocol parser, or hardware abstraction from scratch, inspect
the existing `Modules/` tree and reuse or improve the closest matching module
when practical.

- Prefer reusable modules under `Modules/Drivers`, `Modules/Control`,
  `Modules/BSP`, and other shared module folders over per-project copies.
- Project code should contain board/application glue only: pin bindings,
  SysConfig-generated macro usage, scheduling, route logic, and test entry
  points.
- If an existing module is incomplete or not matched to the real hardware,
  update that module in place when it is intended to be reusable, then connect
  it to the project through a small board adaptation file. This file should only
  bind the reusable module to this board's SysConfig-generated pins,
  peripherals, and timing.
- Do not duplicate a module into a project just to make small edits. If a
  temporary project-local bridge is needed while CCS linked resources are being
  configured, document it and keep the reusable module as the single source of
  truth.
- Check the module README/header comments and current users before changing a
  shared API. Preserve backwards compatibility when reasonable; otherwise
  update all affected projects/tests in the same task.
- `Modules/Drivers/F32C_MOTOR` is a protected completed gimbal motor driver.
  Do not modify it, rename it, move it, or add it to unrelated project builds
  unless the user explicitly approves that specific change.
- Legacy vendor or STM32-specific code under `Modules/` must not be compiled
  into MSPM0 projects until it has been ported to MSPM0 DriverLib/SysConfig and
  tested.
- For CCS projects, add shared modules to the build using CCS Project tools
  or linked resources. Do not hand-edit `.project`, `.cproject`,
  `.ccsproject`, `.syscfg`, generated `ti_msp_dl_config.c/h`, or `Debug/` /
  `Release/` output files.
- Hardware behavior must be validated step by step. Source checks, SysConfig
  checks, CCS build success, flash success, and real-board behavior are
  separate results and must be reported separately.
