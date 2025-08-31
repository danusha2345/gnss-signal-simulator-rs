# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

- `cargo build` - Build the project
- `cargo run` - Build and run the main executable
- `cargo test` - Run unit tests
- `cargo check` - Check code without building

## Architecture Overview

This is a GNSS (Global Navigation Satellite System) signal generation library written in Rust. The codebase implements IF (Intermediate Frequency) signal generation for multiple GNSS systems including GPS, GLONASS, Galileo, and BeiDou.

### Core Structure

- **Entry point**: `src/main.rs` demonstrates usage of almanac conversion, BCNav1Bit frame generation, and parameter setting
- **Library interface**: `src/lib.rs` exposes all public modules and types
- **Signal generation**: `src/ifdatagen.rs` contains the main IF data generation logic

### Key Modules

- **Navigation data generation**: Multiple `*navbit.rs` modules handle different GNSS navigation message formats (BCNav, CNNav, FNav, etc.)
- **Time systems**: `src/gnsstime.rs` handles conversions between different GNSS time systems
- **Coordinate systems**: `src/coordinate.rs` provides conversions between LLA and ECEF coordinate systems
- **Satellite data**: `src/satellite_*.rs` modules handle satellite parameters and signals
- **Almanac processing**: `src/almanac.rs` handles almanac data parsing and type detection
- **Mathematical utilities**: `src/fastmath.rs` and `src/complex_number.rs` provide optimized math operations

### Data Types

Core types are defined in `src/types.rs` including:

- `GnssTime`, `UtcTime`, `GlonassTime` for time representation
- `AlmanacType`, `GnssSystem` enums for system identification
- Various ephemeris and almanac structures

### Configuration and Constants

- `src/constants.rs` contains mathematical and GNSS system constants (WGS84, PZ90, etc.)
- JSON parsing capabilities through `src/json_*.rs` modules for configuration input

The project uses minimal dependencies (only `rand` crate) and implements most functionality from scratch for GNSS signal processing.

## Multi-System GNSS Generation

The project now supports simultaneous multi-system signal generation with mathematically correct ephemeris synchronization:

### Supported GNSS Systems
- **GPS L1CA**: Legacy civilian signal (1575.42 MHz)
- **BeiDou B1C**: Modern civilian signal (1575.42 MHz) 
- **Multi-system**: Simultaneous GPS + BeiDou generation

### Unified Epoch Algorithm
A critical feature ensuring navigational correctness by forcing all satellites to use ephemeris data from the same time epoch (toe). This prevents mathematical inconsistencies that would occur if different satellites used ephemeris from different time periods.

### Available Presets
- `presets/GPS_L1_only.json` - GPS L1CA only generation
- `presets/GPS_L1CA_BDS_B1C_dual.json` - Simultaneous GPS + BeiDou generation
- Other multi-system configurations available

### Performance Features
- **Parallel Mode**: Enable with `GNSS_PARALLEL_MODE=true` environment variable
- **High Performance**: Optimized Rayon-based parallel processing 
- **Unified Output**: All systems generated into single IF data file
