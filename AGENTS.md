# Agent entry point

Read `README.md` for build and runtime context, then `docs/README.md` for the
current project documentation map.

## Documentation boundaries

- `docs/` contains maintained project knowledge and contracts.
- `.agents/skills/` contains reusable procedures; follow the matching skill
  when performing that task.
- `.agents/knowledge/` contains concise agent-oriented repository context and
  links to the canonical documentation. Do not fork authoritative contracts
  here.
- `.agents/notes/` contains plans and temporary investigation notes, not
  verified project policy.
- `docs/archive/` preserves historical reviews. Check dates and current source
  before relying on claims in archived files.

## Repository-specific guidance

- Treat `modules/fast_lio2/` as the Apollo Cyber runtime and the
  framework-independent `fast_lio2/` tree as contracts, pipeline interfaces,
  evaluation, and map-product support. Confirm current code before describing
  implementation status.
- Keep sensor timestamp, units, frame, extrinsic direction, and per-point time
  semantics explicit. See
  `docs/sensors/fast_lio2_sensor_contract.md`.
- Distinguish a short replay smoke test from trajectory/map quality
  validation; report only what the collected evidence supports.
- Do not add records, customer configurations, runtime logs, or generated
  mapping results to tracked source or documentation. The repository ignores
  `data/` and `artifacts/` for private inputs and generated outputs.
- Preserve unrelated working-tree changes. Do not stage or commit unless the
  user explicitly asks.
