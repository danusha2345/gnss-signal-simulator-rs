# Repository Guidelines

## Project Structure & Module Organization
- `src/` — core library and app modules (`lib.rs`, `main.rs`, GNSS bits/time/coords, `ifdatagen.rs`).
- `src/bin/` — auxiliary binaries: `spectrum_analyzer`, `nav_test`, benches.
- `presets/` — JSON configs for scenarios; edit and pass to `cargo run`.
- `Rinex_Data/`, `EphData/` — ephemeris/almanac inputs (read-only in Git).
- `generated_files/` — large outputs (ignored); do not commit.
- Logs, PDFs and debug artifacts live at repo root; keep out of PRs.

## Build, Test, and Development Commands
- `cargo check` — fast type/check cycle.
- `cargo build --release [--features gpu,avx512]` — optimized build. GPU requires CUDA 12.5 (cudarc).
- `cargo run -- presets/gps_l1ca.json` — run main generator with a preset.
- `cargo run --bin spectrum_analyzer -- <file> <iq4|iq8>` — analyze IF samples.
- `cargo test` — run unit tests (`#[cfg(test)]` in modules).
- `cargo fmt --all` / `cargo clippy -- -D warnings` — format and lint.
- Example: `cargo run --release -- presets/gps_bds_gal_l1.json` (Rayon-параллелизм всегда включён).

## Coding Style & Naming Conventions
- Rust 2021; rustfmt default (4 spaces). Run `cargo fmt` before commit.
- Names: modules/files `snake_case`; types `CamelCase`; consts `SCREAMING_SNAKE_CASE`; functions `snake_case`.
- Prefer `Result<>` over `panic!`; validate indices (see `satellite_param.rs`).
- Keep functions small, benchmark hot paths, document with `///`.

## Testing Guidelines
- Unit tests colocated in modules (e.g., `gnsstime.rs`); name tests `test_*`.
- Add integration tests under `tests/` if needed.
- For performance-sensitive changes: include timing notes or a small bench and expected ranges.
- Run `cargo test` and attach relevant logs for RINEX/IF scenarios.

## Commit & Pull Request Guidelines
- Use Conventional Commits: `feat:`, `fix:`, `perf:`, `refactor:`, `docs:`, `test:`, `chore:`, `debug:`, `config:` (see Git history).
- PRs must include: purpose, minimal diff, how to run (preset path), expected output/metrics, and any compatibility notes (GPU/AVX512).
- CI hygiene: pass `fmt`, `clippy -D warnings`, and tests. No large binaries or secrets in Git.

## Security & Configuration Tips
- Do not commit credentials or large data. Store secrets in local env, not `.env` in Git; rotate if exposed.
- Use relative paths in presets; keep `Rinex_Data/` files read-only in PRs.
