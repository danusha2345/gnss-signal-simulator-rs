//! Tests for upstream fixes and new functionality.
//!
//! Covers: IonoKlobuchar8 trait, ephemeris transition, L1C swap,
//! BDS GEO svid>=59, AlignToe300s, BCNav2 message types 31-40,
//! FindEphemeris <= comparison.

use gnss_rust::bcnav2bit::BCNav2Bit;
use gnss_rust::coordinate::gps_sat_pos_speed_eph;
use gnss_rust::delay_model::{IonoDelay, IonoKlobuchar8};
use gnss_rust::navbit::NavBit;
use gnss_rust::powercontrol::ElevationAdjust;
use gnss_rust::satellite_param::SatelliteParamCalculator;
use gnss_rust::types::*;

// ---------------------------------------------------------------------------
// Test 1: IonoKlobuchar8 trait
// ---------------------------------------------------------------------------
#[test]
fn test_iono_klobuchar8_delay() {
    let model = IonoKlobuchar8::new();
    // Default zero params should return non-negative delay
    let delay = model.get_delay(0.0, 0.5, 1.0, 0.5, 0.0);
    assert!(delay >= 0.0, "Ionosphere delay should be non-negative");

    // With some non-zero params, delay should be positive
    let mut param = IonoParam {
        a0: 1.0e-8,
        ..Default::default()
    };

    param.a1 = 1.0e-8;
    let mut model2 = IonoKlobuchar8::new();
    model2.set_iono_param(&param);
    let delay2 = model2.get_delay(50000.0, 0.6, 1.5, 0.3, 0.5);
    assert!(
        delay2 > 0.0,
        "Ionosphere delay with non-zero params should be positive"
    );
}

// ---------------------------------------------------------------------------
// Test 2: SatelliteParamCalculator ephemeris transition
// ---------------------------------------------------------------------------
#[test]
fn test_satellite_param_calculator_eph_transition() {
    let model = Box::new(IonoKlobuchar8::new());
    let mut calc = SatelliteParamCalculator::new(model);

    let mut eph1 = GpsEphemeris {
        svid: 1,
        ..Default::default()
    };

    eph1.toe = 100000;
    eph1.iode = 1;
    eph1.valid = 1;
    eph1.sqrtA = 5153.0; // typical GPS satellite
    eph1.axis = 5153.0 * 5153.0;

    calc.initialize(
        GnssSystem::GpsSystem,
        &eph1,
        45.0,
        ElevationAdjust::ElevationAdjustNone,
    );
    assert_eq!(calc.eph_transition, 0);

    // Update with different ephemeris
    let mut eph2 = eph1;
    eph2.toe = 107200; // 2 hours later
    eph2.iode = 2;
    calc.update_ephemeris_gps(&eph2);
    assert_eq!(
        calc.eph_transition, 600000,
        "Transition period should be 600000ms"
    );
}

// ---------------------------------------------------------------------------
// Test 3: L1C swap verification
// ---------------------------------------------------------------------------
#[test]
fn test_l1c_generates_distinct_data_and_pilot() {
    use gnss_rust::prngenerate::PrnGenerate;
    let prn = PrnGenerate::new(GnssSystem::GpsSystem, gnss_rust::SIGNAL_INDEX_L1C as i32, 1);
    let data = prn.get_data_prn().unwrap();
    let pilot = prn.get_pilot_prn().unwrap();
    assert_eq!(data.len(), 10230);
    assert_eq!(pilot.len(), 10230);
    assert_ne!(data, pilot);
}

// ---------------------------------------------------------------------------
// Test 4: BDS GEO svid >= 59
// ---------------------------------------------------------------------------
#[test]
fn test_bds_geo_svid_59_and_above() {
    let mut eph = GpsEphemeris {
        svid: 59,
        ..Default::default()
    };

    eph.valid = 1;
    eph.sqrtA = 6493.0; // GEO orbit
    eph.axis = 6493.0 * 6493.0;
    eph.i0 = 0.0; // GEO has ~0 inclination
    eph.ecc = 0.001;
    eph.n = 1.0e-4;
    eph.toe = 100000; // must be close to transmit_time to avoid expiry
    eph.root_ecc = (1.0 - eph.ecc * eph.ecc).sqrt();

    let mut pos = KinematicInfo::default();
    // This should trigger the GEO rotation branch (svid >= 59)
    let result = gps_sat_pos_speed_eph(GnssSystem::BdsSystem, 100000.0, &mut eph, &mut pos, None);
    assert!(result, "BDS GEO with svid=59 should succeed");
}

// ---------------------------------------------------------------------------
// Test 5: AlignToe300s
// ---------------------------------------------------------------------------
#[test]
fn test_align_toe_300s() {
    let mut eph = GpsEphemeris {
        toe: 100150,
        ..Default::default()
    };
    // should round to 100200  (100150 + 150 = 100300) / 300 * 300 = 100200? No: (100150+150)/300 = 100300/300 = 334.33 -> 334*300 = 100200. Actually integer division: 100300/300 = 334, 334*300 = 100200.
    eph.toc = 100150;
    eph.top = 100150;
    eph.sqrtA = 5153.0;
    eph.axis = 5153.0 * 5153.0;
    eph.n = 1.458e-4;

    let aligned = NavBit::align_toe_300s(&eph);
    assert_eq!(aligned.toe, 100200);
    assert_eq!(aligned.toc, 100200);
    assert_eq!(aligned.top, 100200);

    // Test boundary: toe = 604700 should round up
    // (604700 + 150) / 300 * 300 = 604850 / 300 * 300 = 2016 * 300 = 604800
    // 604800 >= 604800, so wraps to 0 with week+1
    let mut eph2 = GpsEphemeris {
        toe: 604700,
        ..Default::default()
    };

    eph2.week = 100;
    eph2.sqrtA = 5153.0;
    eph2.axis = 5153.0 * 5153.0;
    eph2.n = 1.458e-4;

    let aligned2 = NavBit::align_toe_300s(&eph2);
    assert_eq!(aligned2.toe, 0);
    assert_eq!(aligned2.week, 101);
}

// ---------------------------------------------------------------------------
// Test 6: BCNav2 message types 31 to 40
// ---------------------------------------------------------------------------
#[test]
fn test_bcnav2_message_types_31_to_40() {
    let mut nav = BCNav2Bit::new();
    let mut bits = vec![0i32; 600]; // B2a frame is 600 bits

    // MESSAGE_ORDER: [10,11,30,40, 10,11,31,40, 10,11,32,40, 10,11,33,40, 10,11,34,40]
    // sow = MilliSeconds / 3000;  index = sow % 20

    // Type 31: index 6, sow = 6, MilliSeconds = 6*3000 = 18000
    let time_31 = GnssTime {
        Week: 1360,
        MilliSeconds: 18000,
        SubMilliSeconds: 0.0,
    };
    let result = nav.get_frame_data(time_31, 1, 0, &mut bits);
    assert!(result > 0, "BCNav2 type 31 should generate successfully");

    // Type 32: index 10, sow = 10, MilliSeconds = 10*3000 = 30000
    let time_32 = GnssTime {
        Week: 1360,
        MilliSeconds: 30000,
        SubMilliSeconds: 0.0,
    };
    let result = nav.get_frame_data(time_32, 1, 0, &mut bits);
    assert!(result > 0, "BCNav2 type 32 should generate successfully");

    // Type 33: index 14, sow = 14, MilliSeconds = 14*3000 = 42000
    let time_33 = GnssTime {
        Week: 1360,
        MilliSeconds: 42000,
        SubMilliSeconds: 0.0,
    };
    let result = nav.get_frame_data(time_33, 1, 0, &mut bits);
    assert!(result > 0, "BCNav2 type 33 should generate successfully");

    // Type 40: index 3, sow = 3, MilliSeconds = 3*3000 = 9000
    let time_40 = GnssTime {
        Week: 1360,
        MilliSeconds: 9000,
        SubMilliSeconds: 0.0,
    };
    let result = nav.get_frame_data(time_40, 1, 0, &mut bits);
    assert!(result > 0, "BCNav2 type 40 should generate successfully");
}

// ---------------------------------------------------------------------------
// Test 7: FindEphemeris uses <= comparison
// ---------------------------------------------------------------------------
#[test]
fn test_find_ephemeris_equal_time_diff() {
    let mut nav = gnss_rust::ifdatagen::NavData::new();
    let a = GpsEphemeris {
        svid: 1,
        valid: 1,
        week: 2000,
        toe: 100,
        iode: 1,
        ..Default::default()
    };
    let b = GpsEphemeris {
        toe: 120,
        iode: 2,
        ..a
    };
    nav.gps_ephemeris = vec![Some(a), Some(b)];
    let t = GnssTime {
        Week: 2000,
        MilliSeconds: 110000,
        SubMilliSeconds: 0.0,
    };
    assert_eq!(
        nav.find_ephemeris(GnssSystem::GpsSystem, t, 1, 0, 0)
            .unwrap()
            .iode,
        2
    );
}
