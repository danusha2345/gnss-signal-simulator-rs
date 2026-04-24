//! Integration checks for navigation message formation across all systems.
//!
//! Goal: verify that for each supported nav message type, after loading
//! ephemerides/almanac/iono/UTC, the module produces a non-empty bitstream
//! of expected size (or at least a substantial number of non-zero bits),
//! indicating a fully formed navigation message.
//!
//! These are sanity/coverage tests, not bit-exact ICD conformance tests.

use gnss_rust::bcnav1bit::BCNav1Bit;
use gnss_rust::bcnav2bit::BCNav2Bit;
use gnss_rust::bcnav3bit::BCNav3Bit;
use gnss_rust::cnav2bit::CNav2Bit;
use gnss_rust::cnavbit::CNavBit;
use gnss_rust::d1d2navbit::D1D2NavBit;
use gnss_rust::fnavbit::FNavBit;
use gnss_rust::gnavbit::GNavBit;
use gnss_rust::inavbit::INavBit;
use gnss_rust::l5cnavbit::L5CNavBit;
use gnss_rust::lnavbit::LNavBit;
use gnss_rust::types::*;

fn sample_gps_eph(svid: u8) -> GpsEphemeris {
    // Reasonable synthetic ephemeris that satisfies module checks
    GpsEphemeris {
        ura: 2,
        iodc: 45,
        iode: 45,
        svid,
        source: 0,
        valid: 1,
        flag: 0b1111_1111,
        health: 0,
        toe: 345_600, // 4th day
        toc: 345_600,
        top: 345_600,
        week: 2369,
        M0: 0.1,
        delta_n: 4.5e-9,
        delta_n_dot: 0.0,
        ecc: 0.01,
        sqrtA: 5153.795_890_81, // ~26_560 km^0.5
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

fn sample_alm(len: usize) -> Vec<GpsAlmanac> {
    (0..len)
        .map(|i| GpsAlmanac {
            valid: 1,
            flag: 0,
            health: 0,
            svid: (i + 1) as u8,
            toa: 1200 << 12, // scaled as used by modules
            week: 2369,
            M0: 0.0,
            ecc: 0.01,
            sqrtA: 5153.795,
            omega0: 1.0,
            i0: 0.94,
            w: 0.5,
            omega_dot: -8.0e-9,
            af0: 0.0,
            af1: 0.0,
        })
        .collect()
}

fn sample_iono() -> IonoParam {
    IonoParam {
        a0: 1.2e-8,
        a1: -1.5e-8,
        a2: 0.0,
        a3: 0.0,
        b0: 1.2e5,
        b1: 9.6e4,
        b2: 7.2e4,
        b3: 4.8e4,
        flag: 1,
    }
}

fn sample_nequick() -> IonoNequick {
    IonoNequick {
        ai0: 120.0,
        ai1: 0.0,
        ai2: 0.0,
        flag: 1,
    }
}

fn sample_utc() -> UtcParam {
    UtcParam {
        A0: 1.0e-6,
        A1: -1.0e-12,
        A2: 0.0,
        WN: 2369,
        WNLSF: 2370,
        tot: 16, // 2^12 scaled in some modules; presence is enough
        TLS: 18,
        TLSF: 18,
        DN: 4,
        flag: 3, // both UTC and leap second valid
    }
}

fn t(ms: i32) -> GnssTime {
    GnssTime {
        Week: 2369,
        MilliSeconds: ms,
        SubMilliSeconds: 0.0,
    }
}

fn non_zero_ratio(bits: &[i32]) -> f32 {
    let cnt = bits
        .iter()
        .take(bits.len().min(2000))
        .filter(|&&b| b != 0)
        .count();
    cnt as f32 / bits.len().min(2000) as f32
}

fn assert_nav_bits(name: &str, bits: &[i32], min_non_zero_ratio: f32) {
    assert!(
        bits.iter().all(|&bit| bit == 0 || bit == 1),
        "{name} must contain only binary nav bits"
    );
    assert!(
        non_zero_ratio(bits) > min_non_zero_ratio,
        "{name} is too sparse"
    );
}

fn hamming_distance(left: &[i32], right: &[i32]) -> usize {
    assert_eq!(left.len(), right.len());
    left.iter()
        .zip(right.iter())
        .filter(|(left, right)| left != right)
        .count()
}

#[test]
fn gps_lnav_forms_frames() {
    let mut nav = LNavBit::new();
    let eph = sample_gps_eph(1);
    assert_ne!(nav.set_ephemeris(1, &eph), 0);

    // Provide supporting data
    let alm: [GpsAlmanac; 32] = sample_alm(32).try_into().unwrap();
    nav.set_almanac(&alm);
    nav.set_iono_utc(&sample_iono(), &sample_utc());

    // Cover 5 subframes (6s each)
    let mut previous_bits: Option<[i32; 300]> = None;
    for i in 0..5 {
        let mut bits = [0i32; 300];
        let rc = nav.get_frame_data(t(1000 + i * 6000), 1, 0, &mut bits);
        assert_eq!(rc, 0);
        assert_nav_bits(&format!("LNAV frame {}", i + 1), &bits, 0.05);
        if let Some(previous) = previous_bits {
            assert!(
                hamming_distance(&previous, &bits) > 20,
                "LNAV frame {} should differ from previous subframe",
                i + 1
            );
        }
        previous_bits = Some(bits);
    }
}

#[test]
fn gps_cnav_and_l5cnav_form_frames() {
    let eph = sample_gps_eph(3);
    let alm = sample_alm(32);
    let iono = sample_iono();
    let utc = sample_utc();

    // CNAV (L2C)
    let mut cnav = CNavBit::new();
    assert_ne!(cnav.set_ephemeris(3, &eph), 0);
    cnav.set_almanac(&alm);
    cnav.set_iono_utc(&iono, &utc);
    let mut bits_cnav = vec![0i32; 600];
    let rc = cnav.get_frame_data(t(12_000), 3, 0, &mut bits_cnav);
    assert_eq!(rc, 0);
    assert_nav_bits("CNAV frame", &bits_cnav, 0.05);

    // L5 CNAV
    let mut l5 = L5CNavBit::new();
    assert_ne!(l5.set_ephemeris(4, &eph), 0);
    let alm_arr: [GpsAlmanac; 32] = alm[..32].try_into().unwrap();
    l5.set_almanac(&alm_arr);
    l5.set_iono_utc(&iono, &utc);
    let mut bits_l5 = [0i32; 600];
    let rc = l5.get_frame_data(t(6_000), 4, 1, &mut bits_l5);
    assert_eq!(rc, 0);
    assert_nav_bits("L5 CNAV frame", &bits_l5, 0.05);
}

#[test]
fn gps_cnav2_l1c_forms_frames() {
    let eph = sample_gps_eph(6);
    let alm = sample_alm(32);
    let mut l1c = CNav2Bit::new();
    assert_ne!(l1c.set_ephemeris(6, &eph), 0);
    l1c.set_almanac(&alm);
    l1c.set_iono_utc(&sample_iono(), &sample_utc());

    // Allocate generously because L1C uses LDPC blocks and interleaving
    let mut bits = vec![0i32; 2000];
    // Выберем время, дающее субкадр 2 (более плотное наполнение)
    let rc = l1c.get_frame_data(t(36_000), 6, 0, &mut bits);
    assert_eq!(rc, 0);
    assert_nav_bits("CNAV2 frame", &bits[..1200], 0.01);
}

#[test]
fn glonass_gnav_forms_strings() {
    let eph = sample_glo_eph(1); // native GLONASS ephemeris
    let mut gnav = GNavBit::new();
    assert_ne!(gnav.set_glonass_ephemeris(1, &eph), 0);
    gnav.set_almanac(&sample_alm(24));
    gnav.set_iono_utc(None, Some(&sample_utc()));

    let mut bits = vec![0i32; 100];
    let rc = gnav.get_frame_data(t(30_000), 1, 0, &mut bits);
    assert_eq!(rc, 0);
    assert_nav_bits("GLONASS GNAV string", &bits, 0.05);
}

#[test]
fn galileo_inav_fnav_form_frames() {
    let eph = sample_gps_eph(10);
    let alm = sample_alm(36);

    // I/NAV (E1)
    let mut inav = INavBit::new();
    assert_ne!(inav.set_ephemeris(10, &eph), 0);
    inav.set_almanac(&alm);
    inav.set_iono_utc(&sample_nequick(), &sample_utc());
    let mut bits_inav = vec![0i32; 600];
    let rc = inav.get_frame_data(t(2_000), 10, 1, &mut bits_inav);
    assert_eq!(rc, 0);
    assert_nav_bits("Galileo INAV frame", &bits_inav[..500], 0.05);

    // F/NAV (E5a)
    let mut fnav = FNavBit::new();
    assert_ne!(fnav.set_ephemeris(11, &eph), 0);
    fnav.set_almanac(&alm);
    fnav.set_iono_utc(&sample_iono(), &sample_utc());
    let mut bits_fnav = vec![0i32; 600];
    let rc = fnav.get_frame_data(t(1_000), 11, 0, &mut bits_fnav);
    assert_eq!(rc, 0);
    assert_nav_bits("Galileo FNAV frame", &bits_fnav, 0.05);
}

#[test]
fn beidou_d1d2_and_bcnav_form_frames() {
    let eph_b1 = sample_gps_eph(8); // BDS-2 MEO/IGSO via conversion paths
    let eph_geo = sample_gps_eph(2); // for GEO pages
    let alm = sample_alm(63);
    let iono = sample_iono();
    let utc = sample_utc();

    // D1/D2 (B1I/B2I/B3I legacy)
    let mut d12 = D1D2NavBit::new();
    assert_ne!(d12.set_ephemeris(6, &eph_b1), 0); // MEO/IGSO range
    assert_ne!(d12.set_ephemeris(2, &eph_geo), 0); // GEO range (subframe D2)
    d12.set_almanac(&alm);
    d12.set_iono_utc(&iono, &utc);
    let mut bits_d12 = vec![0i32; 300];
    let rc = d12.get_frame_data(t(6_000), 6, 0, &mut bits_d12);
    assert_eq!(rc, 0);
    assert_nav_bits("BeiDou D1/D2 frame", &bits_d12, 0.05);

    // BCNav1 (B1C)
    let mut b1c = BCNav1Bit::new();
    assert!(b1c.set_ephemeris(38, &eph_b1));
    assert!(b1c.set_almanac(&alm));
    b1c.set_iono_utc(Some(&iono), Some(&utc));
    let mut bits_b1c = vec![0i32; 2048];
    let rc = b1c.get_frame_data(t(18_000), 38, 0, &mut bits_b1c);
    assert_eq!(rc, 0);
    assert_nav_bits("BeiDou BCNav1 frame", &bits_b1c[..1500], 0.02);

    // BCNav2 (B2a)
    let mut b2a = BCNav2Bit::new();
    let _ = b2a.set_ephemeris(20, &eph_b1);
    b2a.set_almanac(&alm);
    b2a.set_iono_utc(Some(&iono), Some(&utc));
    let mut bits_b2a = vec![0i32; 1500];
    let rc = b2a.get_frame_data(t(9_000), 20, 0, &mut bits_b2a);
    assert!(rc >= 0);
    assert_nav_bits("BeiDou BCNav2 frame", &bits_b2a[..1200], 0.02);

    // BCNav3 (B2b/B3I)
    let mut b2b = BCNav3Bit::new();
    assert!(b2b.set_ephemeris(22, &eph_b1));
    assert!(b2b.set_almanac(&alm));
    assert!(b2b.set_iono_utc(Some(&iono), Some(&utc)));
    let mut bits_b2b = vec![0i32; 2048];
    let rc = b2b.get_frame_data(t(21_000), 22, 0, &mut bits_b2b);
    assert_eq!(rc, 0);
    assert_nav_bits("BeiDou BCNav3 frame", &bits_b2b[..1500], 0.02);
}
fn sample_glo_eph(slot: u8) -> GlonassEphemeris {
    GlonassEphemeris {
        valid: 1,
        flag: 1,
        freq: 0,
        slot,
        P: 0,
        M: 0,
        Ft: 0,
        n: 0,
        Bn: 0,
        En: 0,
        tb: 900,
        day: 100,
        tk: 1200,
        gamma: 1e-9,
        tn: 0.0,
        dtn: 0.0,
        x: 1.0e6,
        y: -1.2e6,
        z: 2.0e6,
        vx: 0.0,
        vy: 0.0,
        vz: 0.0,
        ax: 0.0,
        ay: 0.0,
        az: 0.0,
        tc: 0.0,
        PosVelT: KinematicInfo::default(),
    }
}
