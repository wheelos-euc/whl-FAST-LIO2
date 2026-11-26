# Repository map

Use the canonical [documentation index](../../docs/README.md) for maintained
project facts.

- `modules/fast_lio2/`: Apollo Cyber runtime, sensor adapters, core, config,
  and launch assets.
- `fast_lio2/`: framework-independent contracts, evaluation, and map-product
  interfaces.
- `tools/`: Bazel wrapper, host replay asset generator, and scorecard tooling.
- `tests/`: contract smoke tests and host-run generator tests.
- `docs/`: maintained architecture, algorithm, and sensor documentation.
- `.agents/skills/`: repeatable task procedures.
- `.agents/notes/`: plans and temporary investigation notes.
- [WLR-720 user-supplied extrinsic](wlr_720.md): task-specific sensor geometry.
- [FAST-LIO2 implementation map](fast_lio2_implementation.md): two-tree
  boundaries and historical upstream-comparison findings.

`data/` and `artifacts/` are ignored locations for private inputs and generated
outputs. Do not copy their contents into tracked documentation.
