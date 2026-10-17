//! Независимые parity equations: GLONASS ICD 4.7 / GNSS-SDR index lists.
use gnss_rust::*;
const EQUATIONS: [&[usize]; 7] = [
    &[
        9, 10, 12, 13, 15, 17, 19, 20, 22, 24, 26, 28, 30, 32, 34, 35, 37, 39, 41, 43, 45, 47, 49,
        51, 53, 55, 57, 59, 61, 63, 65, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84,
    ],
    &[
        9, 11, 12, 14, 15, 18, 19, 21, 22, 25, 26, 29, 30, 33, 34, 36, 37, 40, 41, 44, 45, 48, 49,
        52, 53, 56, 57, 60, 61, 64, 65, 67, 68, 71, 72, 75, 76, 79, 80, 83, 84,
    ],
    &[
        10, 11, 12, 16, 17, 18, 19, 23, 24, 25, 26, 31, 32, 33, 34, 38, 39, 40, 41, 46, 47, 48, 49,
        54, 55, 56, 57, 62, 63, 64, 65, 69, 70, 71, 72, 77, 78, 79, 80, 85,
    ],
    &[
        13, 14, 15, 16, 17, 18, 19, 27, 28, 29, 30, 31, 32, 33, 34, 42, 43, 44, 45, 46, 47, 48, 49,
        58, 59, 60, 61, 62, 63, 64, 65, 73, 74, 75, 76, 77, 78, 79, 80,
    ],
    &[
        20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 50, 51, 52, 53, 54, 55, 56, 57,
        58, 59, 60, 61, 62, 63, 64, 65, 81, 82, 83, 84, 85,
    ],
    &[
        35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57,
        58, 59, 60, 61, 62, 63, 64, 65,
    ],
    &[
        66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85,
    ],
];
fn valid(bits: &[i32]) -> bool {
    EQUATIONS
        .iter()
        .enumerate()
        .all(|(i, row)| row.iter().fold(bits[84 - i], |v, &k| v ^ bits[85 - k]) == 0)
        && bits[..85].iter().fold(0, |v, &b| v ^ b) == 0
}
fn start() -> GnssTime {
    utc_to_gps_time(
        UtcTime {
            Year: 2025,
            Month: 6,
            Day: 5,
            Hour: 10,
            Minute: 5,
            Second: 30.0,
        },
        true,
    )
}
fn nav() -> gnss_rust::gnavbit::GNavBit {
    let mut n = gnss_rust::gnavbit::GNavBit::new();
    n.set_glonass_ephemeris(
        1,
        &GlonassEphemeris {
            valid: 1,
            flag: 1,
            n: 1,
            slot: 1,
            x: 2.1e7,
            y: -1.2e7,
            z: 1.1e7,
            vx: -1200.0,
            vy: 2300.0,
            vz: 800.0,
            day: 10749,
            tb: 46800,
            ..Default::default()
        },
    );
    n
}
fn field(b: &[i32], start: usize, n: usize) -> u32 {
    b[start - 1..start - 1 + n]
        .iter()
        .fold(0, |x, &b| (x << 1) | b as u32)
}
#[test]
fn raw_strings_have_exact_length_hamming_and_current_time() {
    let n = nav();
    for i in 0..75 {
        let mut b = [-1; 100];
        assert_eq!(
            n.get_frame_data(start().add_milliseconds(i as f64 * 2000.0), 1, 0, &mut b),
            0
        );
        assert_eq!(b[0], 0);
        assert!(b[85..].iter().all(|&x| x == -1));
        assert!(valid(&b));
        assert_eq!(field(&b, 2, 4), i % 15 + 1);
        let mut bad = b;
        bad[20] ^= 1;
        assert!(!valid(&bad));
        if i == 0 {
            assert_eq!(field(&b, 10, 5), 13);
            assert_eq!(field(&b, 15, 6), 5);
            assert_eq!(field(&b, 21, 1), 1);
        }
        if i == 15 {
            assert_eq!(field(&b, 15, 6), 6);
            assert_eq!(field(&b, 21, 1), 0);
        }
        if i == 4 {
            assert_eq!(field(&b, 50, 5), 8);
            assert_eq!(field(&b, 6, 11), 522);
        }
    }
}
#[test]
fn transmitted_string_decodes_relative_and_manchester_then_time_mark() {
    let n = nav();
    let mut raw = [0; 85];
    n.get_frame_data(start(), 1, 0, &mut raw);
    let mut signal = gnss_rust::satellite_signal::SatelliteSignal::new();
    assert!(signal.set_signal_attribute(
        GnssSystem::GlonassSystem,
        SIGNAL_INDEX_G1 as i32,
        Some(gnss_rust::nav_data::NavData::GNav(n)),
        1
    ));
    let mut relative = Vec::new();
    let mut data = ComplexNumber::new();
    let mut pilot = ComplexNumber::new();
    for bit in 0..85 {
        signal.get_satellite_signal(
            start().add_milliseconds((bit * 20 + 1) as f64),
            &mut data,
            &mut pilot,
        );
        let first = data.real;
        signal.get_satellite_signal(
            start().add_milliseconds((bit * 20 + 11) as f64),
            &mut data,
            &mut pilot,
        );
        assert_eq!(first, -data.real);
        relative.push(i32::from(first > 0.0));
    }
    let mut decoded = vec![0];
    for i in 1..85 {
        decoded.push(relative[i] ^ relative[i - 1]);
    }
    assert_eq!(decoded, raw);
    assert!(valid(&decoded));
    let mut marker = 0;
    for i in 0..30 {
        signal.get_satellite_signal(
            start().add_milliseconds((1701 + i * 10) as f64),
            &mut data,
            &mut pilot,
        );
        marker = (marker << 1) | u32::from(data.real < 0.0);
    }
    assert_eq!(marker, 0x3e375096);
}
