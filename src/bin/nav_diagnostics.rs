//! GNSS Navigation Message Diagnostics Tool
//!
//! Loads RINEX data, encodes/decodes navigation messages, and verifies
//! round-trip consistency for GPS LNAV, Galileo I-NAV, and BeiDou B-CNAV.
//!
//! Usage: cargo run --bin nav_diagnostics -- --rinex Rinex_Data/rinex_v3_20251560000.rnx

use std::collections::HashSet;
use std::env;

use gnss_rust::bcnavbit::BCNavBit;
use gnss_rust::coordinate::gps_sat_pos_speed_eph;
use gnss_rust::inavbit::INavBit;
use gnss_rust::lnavbit::LNavBit;
use gnss_rust::nav_decode::*;
use gnss_rust::types::*;

const GPS_GM: f64 = 3.986005e14;

fn main() {
    let args: Vec<String> = env::args().collect();

    let rinex_path = match parse_args(&args) {
        Some(path) => path,
        None => {
            eprintln!("Usage: {} --rinex <path_to_rinex_file>", args[0]);
            std::process::exit(1);
        }
    };

    if !std::path::Path::new(&rinex_path).exists() {
        eprintln!("Error: RINEX file not found: {}", rinex_path);
        std::process::exit(1);
    }

    let mut nav_data = gnss_rust::json_interpreter::CNavData::new();
    // Use read_nav_file_limited to load all ephemerides without epoch filtering
    gnss_rust::json_interpreter::read_nav_file_limited(&mut nav_data, &rinex_path, 10000);

    let rinex_filename = std::path::Path::new(&rinex_path)
        .file_name()
        .map(|n| n.to_string_lossy().to_string())
        .unwrap_or_else(|| rinex_path.clone());

    println!("=== GNSS Nav Message Diagnostics ===");
    println!(
        "RINEX: {} (loaded GPS: {}, BDS: {}, GAL: {})\n",
        rinex_filename,
        nav_data.gps_ephemeris.len(),
        nav_data.beidou_ephemeris.len(),
        nav_data.galileo_ephemeris.len()
    );

    let gps_unique = deduplicate_gps(&nav_data.gps_ephemeris);
    let gal_unique = deduplicate_gps(&nav_data.galileo_ephemeris);
    let bds_unique = deduplicate_beidou(&nav_data.beidou_ephemeris);

    run_gps_lnav_roundtrip(&gps_unique);
    run_gps_lnav_parity(&gps_unique);
    run_galileo_inav_roundtrip(&gal_unique);
    run_beidou_bcnav_roundtrip(&bds_unique);
}

fn parse_args(args: &[String]) -> Option<String> {
    let mut i = 1;
    while i < args.len() {
        if args[i] == "--rinex" && i + 1 < args.len() {
            return Some(args[i + 1].clone());
        }
        i += 1;
    }
    None
}

fn deduplicate_gps(ephs: &[GpsEphemeris]) -> Vec<GpsEphemeris> {
    let mut seen = HashSet::new();
    let mut result = Vec::new();
    for eph in ephs {
        if eph.valid != 0 && seen.insert(eph.svid) {
            result.push(*eph);
        }
    }
    result.sort_by_key(|e| e.svid);
    result
}

fn deduplicate_beidou(ephs: &[BeiDouEphemeris]) -> Vec<BeiDouEphemeris> {
    let mut seen = HashSet::new();
    let mut result = Vec::new();
    for eph in ephs {
        if eph.valid != 0 && seen.insert(eph.svid) {
            result.push(*eph);
        }
    }
    result.sort_by_key(|e| e.svid);
    result
}

// ============================================================
// GPS LNAV Round-Trip (uses nav_decode::decode_lnav_stream123)
// ============================================================

fn run_gps_lnav_roundtrip(gps_ephs: &[GpsEphemeris]) {
    println!("--- GPS LNAV Round-Trip ---");
    if gps_ephs.is_empty() {
        println!("  No GPS ephemeris data loaded.\n");
        return;
    }

    println!(
        "{:<4} | {:<4} | {:<6} | {:<14} | {:<12} | {:<17} | {:>10}",
        "SV", "IODE", "toe", "M0 err (rad)", "ecc err", "sqrtA err (m^½)", "Orbit Δ (m)"
    );

    let mut all_ok = true;

    for eph in gps_ephs {
        let svid = eph.svid as i32;
        if !(1..=32).contains(&svid) {
            continue;
        }

        let mut lnav = LNavBit::new();
        lnav.set_ephemeris(svid, eph);

        let stream = &lnav.gps_stream123[(svid - 1) as usize];
        let decoded = decode_lnav_stream123(stream);

        let diffs = decoded.compare_with_original(eph);
        let failed: Vec<_> = diffs.iter().filter(|d| !d.ok).collect();
        if !failed.is_empty() {
            all_ok = false;
            for d in &failed {
                println!(
                    "  ⚠ G{:02} {}: orig={:.6e} decoded={:.6e}",
                    eph.svid, d.name, d.original, d.decoded
                );
            }
        }

        let m0_err = (eph.M0 - decoded.M0).abs();
        let ecc_err = (eph.ecc - decoded.ecc).abs();
        let sqrta_err = (eph.sqrtA - decoded.sqrtA).abs();

        let orbit_delta = compute_orbit_delta_lnav(eph, &decoded, eph.toe as f64 + 1800.0);

        println!(
            "G{:02} | {:<4} | {:<6} | {:<14.2e} | {:<12.2e} | {:<17.2e} | {:>10.2}",
            eph.svid, eph.iode, eph.toe, m0_err, ecc_err, sqrta_err, orbit_delta
        );
    }

    println!(
        "[All parameters within quantization tolerance: {}]\n",
        if all_ok { "YES" } else { "NO" }
    );
}

// ============================================================
// GPS LNAV Parity (uses nav_decode::verify_lnav_parity)
// ============================================================

fn run_gps_lnav_parity(gps_ephs: &[GpsEphemeris]) {
    println!("--- GPS LNAV Parity ---");
    if gps_ephs.is_empty() {
        println!("  No GPS ephemeris data loaded.\n");
        return;
    }

    for eph in gps_ephs {
        let svid = eph.svid as i32;
        if !(1..=32).contains(&svid) {
            continue;
        }
        let mut lnav = LNavBit::new();
        lnav.set_ephemeris(svid, eph);

        // Test all 3 subframes
        for sf in 0..3 {
            let time = GnssTime {
                Week: 2369,
                MilliSeconds: sf * 6000,
                SubMilliSeconds: 0.0,
            };
            let mut nav_bits = [0i32; 300];
            lnav.get_frame_data(time, svid, 0, &mut nav_bits);

            let results = verify_lnav_parity(&nav_bits);
            let ok_count = results.iter().filter(|(_, ok)| *ok).count();
            let tow = (time.MilliSeconds / 6000) + 1;
            let sf_num = (time.MilliSeconds / 6000) % 5 + 1;
            let status = if ok_count == 10 { "✓" } else { "✗" };
            println!(
                "G{:02} TOW={:>5} SF{}: {}/10 words OK {}",
                eph.svid, tow, sf_num, ok_count, status
            );
        }
    }
    println!();
}

// ============================================================
// Galileo I-NAV Round-Trip (uses nav_decode::decode_inav_eph_words)
// ============================================================

fn run_galileo_inav_roundtrip(gal_ephs: &[GpsEphemeris]) {
    println!("--- Galileo I-NAV Round-Trip ---");
    if gal_ephs.is_empty() {
        println!("  No Galileo ephemeris data loaded.\n");
        return;
    }

    println!(
        "{:<4} | {:<4} | {:<6} | {:<14} | {:<12} | {:>10}",
        "SV", "IOD", "toe", "M0 err (rad)", "ecc err", "Orbit Δ (m)"
    );

    let mut all_ok = true;

    for eph in gal_ephs {
        let svid = eph.svid as i32;
        if !(1..=36).contains(&svid) {
            continue;
        }

        let mut inav = INavBit::new();
        inav.set_ephemeris(svid, eph);

        let ephdata = &inav.gal_eph_data[(svid - 1) as usize];
        let decoded = decode_inav_eph_words(ephdata);

        let diffs = decoded.compare_with_original(eph);
        let failed: Vec<_> = diffs.iter().filter(|d| !d.ok).collect();
        if !failed.is_empty() {
            all_ok = false;
            for d in &failed {
                println!(
                    "  ⚠ E{:02} {}: orig={:.6e} decoded={:.6e}",
                    eph.svid, d.name, d.original, d.decoded
                );
            }
        }

        let m0_err = (eph.M0 - decoded.M0).abs();
        let ecc_err = (eph.ecc - decoded.ecc).abs();

        let orbit_delta = compute_orbit_delta_inav(eph, &decoded, eph.toe as f64 + 1800.0);

        println!(
            "E{:02} | {:<4} | {:<6} | {:<14.2e} | {:<12.2e} | {:>10.2}",
            eph.svid, decoded.iod_nav, decoded.toe, m0_err, ecc_err, orbit_delta
        );
    }

    println!(
        "[toe quantization: 60s, max error: 30s. All params OK: {}]\n",
        if all_ok { "YES" } else { "NO" }
    );
}

// ============================================================
// BeiDou B-CNAV Round-Trip (uses nav_decode::decode_bcnav_ephemeris)
// ============================================================

fn run_beidou_bcnav_roundtrip(bds_ephs: &[BeiDouEphemeris]) {
    println!("--- BeiDou B-CNAV Round-Trip ---");
    if bds_ephs.is_empty() {
        println!("  No BeiDou ephemeris data loaded.\n");
        return;
    }

    println!(
        "{:<4} | {:<10} | {:<12} | {:<5} | {:<14} | {:>10}",
        "SV", "toe(orig)", "toe(encoded)", "Δtoe", "M0 err", "Orbit Δ (m)"
    );

    let mut toe_offset_detected = false;

    for bds_eph in bds_ephs {
        let svid = bds_eph.svid as i32;
        if !(1..=63).contains(&svid) {
            continue;
        }

        let gps_eph = bds_eph.to_gps_ephemeris();

        if gps_eph.toe % 300 != 0 {
            continue;
        }

        let mut bcnav = BCNavBit::new();
        let result = bcnav.set_ephemeris(svid, &gps_eph);
        if result == 0 {
            continue;
        }

        let idx = (svid - 1) as usize;
        let decoded = decode_bcnav_ephemeris(
            &bcnav.ephemeris1[idx],
            &bcnav.ephemeris2[idx],
            &bcnav.clock_param[idx],
        );

        let toe_diff = decoded.toe - gps_eph.toe;
        if toe_diff != 0 {
            toe_offset_detected = true;
        }

        let m0_err = (gps_eph.M0 - decoded.M0).abs();

        // Build decoded eph for orbit comparison
        let ref_axis = if gps_eph.flag == 3 { 27906100.0 } else { 42162200.0 };
        let dec_axis = decoded.delta_A + ref_axis;
        let mut dec_eph = GpsEphemeris::default();
        dec_eph.valid = 1;
        dec_eph.M0 = decoded.M0;
        dec_eph.delta_n = decoded.delta_n;
        dec_eph.ecc = decoded.ecc;
        dec_eph.sqrtA = dec_axis.sqrt();
        dec_eph.omega0 = decoded.omega0;
        dec_eph.i0 = decoded.i0;
        dec_eph.w = decoded.w;
        dec_eph.omega_dot = decoded.omega_dot;
        dec_eph.idot = decoded.idot;
        dec_eph.cuc = decoded.cuc;
        dec_eph.cus = decoded.cus;
        dec_eph.crc = decoded.crc;
        dec_eph.crs = decoded.crs;
        dec_eph.cic = decoded.cic;
        dec_eph.cis = decoded.cis;
        dec_eph.toe = decoded.toe;
        dec_eph.toc = decoded.toc;
        prepare_eph_derived(&mut dec_eph);

        let mut orig = gps_eph;
        prepare_eph_derived(&mut orig);

        let mut po = KinematicInfo::default();
        let mut pd = KinematicInfo::default();
        let t = gps_eph.toe as f64 + 1800.0;
        gps_sat_pos_speed_eph(GnssSystem::GpsSystem, t, &mut orig, &mut po, None);
        gps_sat_pos_speed_eph(GnssSystem::GpsSystem, t, &mut dec_eph, &mut pd, None);
        let dx = po.x - pd.x;
        let dy = po.y - pd.y;
        let dz = po.z - pd.z;
        let orbit_delta = (dx * dx + dy * dy + dz * dz).sqrt();

        println!(
            "C{:02} | {:<10} | {:<12} | {:<5} | {:<14.2e} | {:>10.2}",
            bds_eph.svid, gps_eph.toe, decoded.toe, toe_diff, m0_err, orbit_delta
        );
    }

    if toe_offset_detected {
        println!("⚠ WARNING: BeiDou toe offset detected between nav message and orbit propagator");
    }
    println!();
}

// ============================================================
// Helpers
// ============================================================

fn prepare_eph_derived(eph: &mut GpsEphemeris) {
    eph.axis = eph.sqrtA * eph.sqrtA;
    eph.root_ecc = (1.0 - eph.ecc * eph.ecc).sqrt();
    if eph.axis > 0.0 {
        eph.n = (GPS_GM / (eph.axis * eph.axis * eph.axis)).sqrt() + eph.delta_n;
    }
    eph.omega_t = eph.omega0;
    eph.omega_delta = eph.omega_dot;
}

fn compute_orbit_delta_lnav(
    original: &GpsEphemeris,
    decoded: &DecodedLnavEph,
    transmit_time: f64,
) -> f64 {
    let mut orig_eph = *original;
    prepare_eph_derived(&mut orig_eph);
    let mut orig_pos = KinematicInfo::default();
    gps_sat_pos_speed_eph(GnssSystem::GpsSystem, transmit_time, &mut orig_eph, &mut orig_pos, None);

    let mut dec_eph = GpsEphemeris::default();
    dec_eph.valid = 1;
    dec_eph.M0 = decoded.M0;
    dec_eph.delta_n = decoded.delta_n;
    dec_eph.ecc = decoded.ecc;
    dec_eph.sqrtA = decoded.sqrtA;
    dec_eph.omega0 = decoded.omega0;
    dec_eph.i0 = decoded.i0;
    dec_eph.w = decoded.w;
    dec_eph.omega_dot = decoded.omega_dot;
    dec_eph.idot = decoded.idot;
    dec_eph.cuc = decoded.cuc;
    dec_eph.cus = decoded.cus;
    dec_eph.crc = decoded.crc;
    dec_eph.crs = decoded.crs;
    dec_eph.cic = decoded.cic;
    dec_eph.cis = decoded.cis;
    dec_eph.toe = decoded.toe;
    dec_eph.toc = decoded.toc;
    prepare_eph_derived(&mut dec_eph);

    let mut dec_pos = KinematicInfo::default();
    gps_sat_pos_speed_eph(GnssSystem::GpsSystem, transmit_time, &mut dec_eph, &mut dec_pos, None);

    let dx = orig_pos.x - dec_pos.x;
    let dy = orig_pos.y - dec_pos.y;
    let dz = orig_pos.z - dec_pos.z;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn compute_orbit_delta_inav(
    original: &GpsEphemeris,
    decoded: &DecodedInavEph,
    transmit_time: f64,
) -> f64 {
    let mut orig_eph = *original;
    prepare_eph_derived(&mut orig_eph);
    let mut orig_pos = KinematicInfo::default();
    gps_sat_pos_speed_eph(GnssSystem::GpsSystem, transmit_time, &mut orig_eph, &mut orig_pos, None);

    let mut dec_eph = GpsEphemeris::default();
    dec_eph.valid = 1;
    dec_eph.M0 = decoded.M0;
    dec_eph.delta_n = decoded.delta_n;
    dec_eph.ecc = decoded.ecc;
    dec_eph.sqrtA = decoded.sqrtA;
    dec_eph.omega0 = decoded.omega0;
    dec_eph.i0 = decoded.i0;
    dec_eph.w = decoded.w;
    dec_eph.omega_dot = decoded.omega_dot;
    dec_eph.idot = decoded.idot;
    dec_eph.cuc = decoded.cuc;
    dec_eph.cus = decoded.cus;
    dec_eph.crc = decoded.crc;
    dec_eph.crs = decoded.crs;
    dec_eph.cic = decoded.cic;
    dec_eph.cis = decoded.cis;
    dec_eph.toe = decoded.toe;
    dec_eph.toc = decoded.toc;
    prepare_eph_derived(&mut dec_eph);

    let mut dec_pos = KinematicInfo::default();
    gps_sat_pos_speed_eph(GnssSystem::GpsSystem, transmit_time, &mut dec_eph, &mut dec_pos, None);

    let dx = orig_pos.x - dec_pos.x;
    let dy = orig_pos.y - dec_pos.y;
    let dz = orig_pos.z - dec_pos.z;
    (dx * dx + dy * dy + dz * dz).sqrt()
}
