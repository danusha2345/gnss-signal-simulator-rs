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
- **GPS ephemeris**: `GpsEphemeris` structure for GPS navigation data
- **BeiDou ephemeris**: `BeiDouEphemeris` specialized structure with RINEX 3.04 compliant fields
- **GLONASS ephemeris**: `GlonassEphemeris` for Russian GNSS system
- Various almanac structures for different GNSS systems

### Configuration and Constants

- `src/constants.rs` contains mathematical and GNSS system constants (WGS84, PZ90, etc.)
- JSON parsing capabilities through `src/json_*.rs` modules for configuration input

The project uses minimal dependencies (only `rand` crate) and implements most functionality from scratch for GNSS signal processing.

## Multi-System GNSS Generation

The project now supports simultaneous multi-system signal generation with mathematically correct ephemeris synchronization:

### Supported GNSS Systems

- **GPS L1CA**: Legacy civilian signal (1575.42 MHz)
- **BeiDou B1C**: Modern civilian signal (1575.42 MHz)  
- **Galileo E1**: European civilian signal (1575.42 MHz)
- **Multi-system**: Simultaneous GPS + BeiDou + Galileo generation

### Unified Epoch Algorithm

A critical feature ensuring navigational correctness by forcing all satellites to use ephemeris data from the same time epoch (toe). This prevents mathematical inconsistencies that would occur if different satellites used ephemeris from different time periods.

### RINEX 3.04 BeiDou Support

- **Dedicated BeiDou Structure**: Specialized `BeiDouEphemeris` structure for proper RINEX 3.04 parsing
- **BeiDou-specific fields**: AODE, AODC, TGD1, TGD2, satellite type identification
- **Full constellation**: Support for GEO, IGSO, MEO satellites (SVID 1-63)
- **CGCS2000 coordinate system**: Native BeiDou coordinate reference frame
- **Backward compatibility**: Conversion methods for legacy GPS-based code

### Galileo E1 Support

- **Full RINEX 3.04 Galileo parsing**: Native support for Galileo ephemeris data
- **GST time system**: Galileo System Time support with GPS time synchronization
- **I/NAV navigation message**: Proper Galileo E1 signal generation
- **European constellation**: Complete support for Galileo satellite constellation

### Time System Corrections

**CRITICAL BUG FIXES (August 2025):**

- **UTC to GPS Time Conversion**: Fixed incorrect GLONASS time epoch reference in `src/gnsstime.rs:97`
  - **Issue**: `utc_to_gps_time` was using `utc_to_glonass_time` with 1992 epoch instead of correct 1996 epoch
  - **Fix**: Restored original `utc_to_glonass_time` usage for proper GPS week calculation
  - **Impact**: Corrected 4-year time offset causing all satellites to appear below horizon

- **BeiDou Time System**: Proper BDT to GPS time conversion with 1356-week offset
  - **BDT Epoch**: January 1, 2006 vs GPS epoch January 6, 1980
  - **Correction**: Added `BDT_TO_GPS_WEEK_OFFSET = 1356` weeks for temporal filtering

- **Temporal Filtering**: 3-hour time window filtering for ephemeris selection
  - **GPS**: Uses GPS time directly from RINEX (Week 2369 for June 2025)  
  - **BeiDou**: BDT time corrected to GPS time for unified filtering
  - **Galileo**: GST time synchronized with GPS time for consistency

### Available Presets

- `presets/GPS_L1_only.json` - GPS L1CA only generation
- `presets/GPS_BDS_GAL_triple_system.json` - Triple system: GPS + BeiDou + Galileo generation
- Other multi-system configurations available

### Current Status

**Ephemeris Processing**: ✅ Working correctly
- GPS: 30 satellites loaded from 7 epochs
- BeiDou: 295 satellites loaded from 6 epochs  
- Galileo: 1534 satellites loaded from 36 epochs

**Time Synchronization**: ✅ Fixed
- Correct UTC to GPS time conversion (2025-06-05 → GPS Week 2369)
- Proper BeiDou BDT time correction
- Unified temporal filtering across all GNSS systems

**Satellite Visibility**: ✅ RESOLVED (September 2025)
- **GPS**: 8 visible satellites with correct elevation angles
- **BeiDou**: 25 visible satellites 
- **Galileo**: 4 visible satellites
- **Total**: 37 visible satellites across all systems

### Major Bug Fixes (September 2025)

**CRITICAL FIXES for Satellite Visibility:**

1. **LOS Vector Calculation Bug** in `src/coordinate.rs:geometry_distance_array()`:
   - **Issue**: LOS vector calculated using Earth-rotation-compensated distance instead of geometric distance
   - **Fix**: Separated geometric distance calculation from Earth rotation compensation
   - **Impact**: Fixed all satellite position and elevation calculations

2. **Missing GPS omega_t Parameter** in `src/json_interpreter.rs:parse_gps_ephemeris()`:
   - **Issue**: Critical orbital parameters `omega_t` and `omega_delta` never initialized (remained 0.0)
   - **Fix**: Added proper initialization: `eph.omega_t = eph.omega0; eph.omega_delta = eph.omega_dot;`
   - **Impact**: Corrected all GPS satellite orbital positions

3. **Epoch Selection Algorithm** in ephemeris filtering:
   - **Issue**: Algorithm prioritized past epochs over closest time difference
   - **Fix**: Changed to select epoch with minimum absolute time difference
   - **Impact**: All GNSS systems now use temporally optimal ephemeris data

4. **Doppler Frequency Calculation** in `src/ifdatagen.rs`:
   - **Issue**: Incorrect velocity source - used separate array instead of `KinematicInfo` velocity fields
   - **Fix**: Use `sat_pos_vel.vx/vy/vz` from `gps_sat_pos_speed_eph()` function
   - **Impact**: Realistic Doppler values (±5000 Hz range) instead of incorrect small values

5. **Galileo RINEX Parsing Bug** in `src/json_interpreter.rs:parse_galileo_ephemeris()`:
   - **Issue**: Used GPS parser (`parse_gps_ephemeris`) which reads 7 data lines, but Galileo has only 6 lines in RINEX 3.04
   - **Fix**: Created dedicated Galileo parser reading correct 6 data lines with Galileo-specific parameters
   - **Impact**: Fixed Galileo ephemeris parsing, proper BGD/SISA/IODnav parameter handling

### Detailed Satellite Output Tables

Added comprehensive satellite visibility tables matching C version format:
- **PRN/SV numbers**: Satellite identification
- **Elevation/Azimuth**: Accurate angular positions
- **Doppler frequencies**: Realistic values in ±5000 Hz range  
- **Range distances**: Satellite-receiver distances in meters

### Performance Features

- **Parallel Mode**: Enable with `GNSS_PARALLEL_MODE=true` environment variable
- **High Performance**: Optimized Rayon-based parallel processing
- **Unified Output**: All systems generated into single IF data file

### Known Issues

~~1. **Satellite Visibility Calculation**: Despite correct ephemeris loading and time synchronization, all satellites appear below horizon (0 visible satellites for all systems)~~
~~2. **Position Calculation**: Need to investigate satellite position computation algorithms in elevation/azimuth calculation~~

**All major visibility issues have been resolved as of September 2025. System now correctly calculates satellite positions and visibility.**
