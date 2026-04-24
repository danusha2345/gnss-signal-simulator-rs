use gnss_rust::json_parser::JsonStream;
use gnss_rust::lnavbit::LNavBit;
use gnss_rust::nav_decode::{
    decode_lnav_stream123, extract_bits, rescale_int, rescale_uint, sign_extend, sign_magnitude_33,
    verify_inav_crc24q, verify_lnav_parity,
};
use gnss_rust::pvt::solve_pvt_wls;
use gnss_rust::types::{GnssTime, GpsEphemeris, KinematicInfo};
use std::ffi::CStr;

fn t(ms: i32) -> GnssTime {
    GnssTime {
        Week: 2369,
        MilliSeconds: ms,
        SubMilliSeconds: 0.0,
    }
}

fn sample_gps_eph(svid: u8) -> GpsEphemeris {
    GpsEphemeris {
        ura: 2,
        iodc: 45,
        iode: 45,
        svid,
        source: 0,
        valid: 1,
        flag: 0b1111_1111,
        health: 0,
        toe: 345_600,
        toc: 345_600,
        top: 345_600,
        week: 2369,
        M0: 0.1,
        delta_n: 4.5e-9,
        delta_n_dot: 0.0,
        ecc: 0.01,
        sqrtA: 5153.795_890_81,
        axis_dot: 0.0,
        omega0: 1.0,
        i0: 0.94,
        w: 0.5,
        omega_dot: -8.0e-9,
        idot: 0.0,
        cuc: 1.0e-6,
        cus: -1.2e-6,
        crc: 200.0,
        crs: -100.0,
        cic: 5.0e-8,
        cis: -5.0e-8,
        af0: 1.0e-4,
        af1: -1.0e-12,
        af2: 0.0,
        tgd: -1.1e-8,
        tgd2: 2.2e-9,
        tgd_ext: [0.0, 0.0, 2.0e-9, 3.0e-9, 0.0],
        axis: 26_560_000.0,
        n: 0.0,
        root_ecc: 0.0,
        omega_t: 0.0,
        omega_delta: 0.0,
        Ek: 0.0,
        Ek_dot: 0.0,
    }
}

#[test]
fn nav_decode_bit_primitives_and_crc_have_stable_baseline() {
    assert_eq!(extract_bits(0b101101, 1, 3), 0b110);
    assert_eq!(sign_extend(0b1111, 4), -1);
    assert_eq!(sign_extend(0b0111, 4), 7);
    assert_eq!(sign_extend(0x8000_0000, 32), i32::MIN);
    assert_eq!(rescale_int(-2, -3), -0.25);
    assert_eq!(rescale_uint(3, 2), 12.0);
    assert_eq!(sign_magnitude_33(1, 0xffff_fffe), -2.0);

    assert!(verify_inav_crc24q(&[], 0, 0));
    assert!(!verify_inav_crc24q(&[], 0, 1));
}

#[test]
fn lnav_stream_roundtrips_through_decoder_within_quantization_tolerance() {
    let eph = sample_gps_eph(1);
    let mut nav = LNavBit::new();
    assert_ne!(nav.set_ephemeris(1, &eph), 0);

    let decoded = decode_lnav_stream123(&nav.gps_stream123[0]);
    let diffs = decoded.compare_with_original(&eph);
    let failures: Vec<_> = diffs
        .iter()
        .filter(|diff| !diff.ok)
        .map(|diff| diff.name)
        .collect();

    assert!(
        failures.is_empty(),
        "LNAV decode diffs failed: {failures:?}"
    );
}

#[test]
fn lnav_parity_verifier_accepts_zero_words_and_rejects_flipped_parity() {
    let zero_bits = [0i32; 300];
    assert!(verify_lnav_parity(&zero_bits).iter().all(|(_, ok)| *ok));

    let mut bad_bits = zero_bits;
    bad_bits[29] = 1;
    let results = verify_lnav_parity(&bad_bits);
    assert!(!results[0].1);
}

#[test]
fn json_stream_parses_object_tree_and_exposes_top_level_keys() {
    let mut stream = JsonStream::new();
    assert_eq!(
        stream.parse_string(r#"{"name":"baseline","count":3,"nested":{"flag":true}}"#),
        0
    );

    let root = stream.get_root_object();
    assert!(!root.is_null());

    unsafe {
        let first_child = JsonStream::get_first_object(root);
        let mut current = if first_child.is_null() {
            root
        } else {
            first_child
        };
        let mut keys = Vec::new();
        while !current.is_null() {
            keys.push(
                CStr::from_ptr((*current).key.as_ptr())
                    .to_string_lossy()
                    .into_owned(),
            );
            current = JsonStream::get_next_object(current);
        }

        assert!(keys.iter().any(|key| key == "name"));
        assert!(keys.iter().any(|key| key == "count"));
        assert!(keys.iter().any(|key| key == "nested"));
    }
}

#[test]
fn pvt_solver_requires_at_least_four_satellite_measurements() {
    let empty_gps: [Option<GpsEphemeris>; 4] = [None, None, None, None];
    let result = solve_pvt_wls(
        t(381_948_000),
        t(381_934_000),
        t(381_948_000),
        &empty_gps,
        &empty_gps,
        &empty_gps,
        KinematicInfo::default(),
    );

    assert!(result.is_none());
}
