use gnss_rust::constants::{LIGHT_SPEED, SIGNAL_INDEX_L1CA};
use gnss_rust::coordinate::{
    ecef_to_lla, geometry_distance, gps_iono_delay, gps_sat_pos_speed_eph, lla_to_ecef,
    sat_el_az_from_positions, tropo_delay,
};
use gnss_rust::json_interpreter::{read_nav_file_limited, CNavData};
use gnss_rust::satellite_param::{get_doppler, get_satellite_param, get_travel_time};
use gnss_rust::types::{
    GnssSystem, GnssTime, IonoParam, KinematicInfo, LlaPosition, SatelliteParam,
};

const RINEX_PATH: &str = "Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx";
const TARGET_GPS_TIME: GnssTime = GnssTime {
    Week: 2369,
    MilliSeconds: 381_948_000,
    SubMilliSeconds: 0.0,
};
const TARGET_SVID: u8 = 23;

fn load_nav_data() -> CNavData {
    let mut nav_data = CNavData::new();
    read_nav_file_limited(&mut nav_data, RINEX_PATH, 10_000);
    nav_data
}

fn receiver_lla() -> LlaPosition {
    LlaPosition {
        lat: 55.7539_f64.to_radians(),
        lon: 37.6208_f64.to_radians(),
        alt: 150.0,
    }
}

#[test]
fn lla_ecef_roundtrip_preserves_receiver_position() {
    let lla = receiver_lla();
    let ecef = lla_to_ecef(&lla);
    let roundtrip = ecef_to_lla(&ecef);

    assert!((roundtrip.lat - lla.lat).abs() < 1e-12);
    assert!((roundtrip.lon - lla.lon).abs() < 1e-12);
    assert!((roundtrip.alt - lla.alt).abs() < 1e-6);
}

#[test]
fn geometry_distance_returns_unit_los_vector() {
    let receiver = KinematicInfo {
        x: 1_000.0,
        y: 2_000.0,
        z: 3_000.0,
        ..KinematicInfo::default()
    };
    let satellite = KinematicInfo {
        x: 21_000_000.0,
        y: -13_000_000.0,
        z: 18_000_000.0,
        ..KinematicInfo::default()
    };

    let mut los = [0.0; 3];
    let distance = geometry_distance(&receiver, &satellite, Some(&mut los));
    let los_norm = (los[0] * los[0] + los[1] * los[1] + los[2] * los[2]).sqrt();

    assert!(distance > 20_000_000.0);
    assert!(distance < 35_000_000.0);
    assert!((los_norm - 1.0).abs() < 1e-12);
}

#[test]
fn gps_ephemeris_propagates_to_reasonable_position_and_velocity() {
    let nav_data = load_nav_data();
    let target_sow = TARGET_GPS_TIME.MilliSeconds as f64 / 1000.0;
    let eph = nav_data
        .gps_ephemeris
        .iter()
        .filter(|eph| eph.svid == TARGET_SVID && eph.valid != 0)
        .min_by(|a, b| {
            (a.toe as f64 - target_sow)
                .abs()
                .partial_cmp(&(b.toe as f64 - target_sow).abs())
                .unwrap()
        })
        .expect("target GPS ephemeris should exist");

    let mut sat_pos = KinematicInfo::default();
    let mut eph_for_propagation = *eph;
    gps_sat_pos_speed_eph(
        GnssSystem::GpsSystem,
        target_sow,
        &mut eph_for_propagation,
        &mut sat_pos,
        None,
    );

    let radius = (sat_pos.x * sat_pos.x + sat_pos.y * sat_pos.y + sat_pos.z * sat_pos.z).sqrt();
    let speed =
        (sat_pos.vx * sat_pos.vx + sat_pos.vy * sat_pos.vy + sat_pos.vz * sat_pos.vz).sqrt();

    assert!(radius > 25_000_000.0, "unexpected orbit radius: {radius}");
    assert!(radius < 28_000_000.0, "unexpected orbit radius: {radius}");
    assert!(speed > 2_000.0, "unexpected satellite speed: {speed}");
    assert!(speed < 5_000.0, "unexpected satellite speed: {speed}");
}

#[test]
fn satellite_param_doppler_delay_and_angles_are_physical() {
    let nav_data = load_nav_data();
    let target_sow = TARGET_GPS_TIME.MilliSeconds as f64 / 1000.0;
    let eph = nav_data
        .gps_ephemeris
        .iter()
        .filter(|eph| eph.svid == TARGET_SVID && eph.valid != 0)
        .min_by(|a, b| {
            (a.toe as f64 - target_sow)
                .abs()
                .partial_cmp(&(b.toe as f64 - target_sow).abs())
                .unwrap()
        })
        .expect("target GPS ephemeris should exist");

    let receiver_lla = receiver_lla();
    let receiver_ecef = lla_to_ecef(&receiver_lla);
    let iono = if let (Some(alpha), Some(beta)) =
        (nav_data.get_gps_iono_alpha(), nav_data.get_gps_iono_beta())
    {
        IonoParam {
            a0: alpha[0],
            a1: alpha[1],
            a2: alpha[2],
            a3: alpha[3],
            b0: beta[0],
            b1: beta[1],
            b2: beta[2],
            b3: beta[3],
            flag: 1,
        }
    } else {
        IonoParam::default()
    };

    let mut sat_param = SatelliteParam::default();
    get_satellite_param(
        &receiver_ecef,
        &receiver_lla,
        &TARGET_GPS_TIME,
        GnssSystem::GpsSystem,
        eph,
        &iono,
        &mut sat_param,
    );

    let doppler = get_doppler(&sat_param, SIGNAL_INDEX_L1CA);
    let travel_time = get_travel_time(&sat_param, SIGNAL_INDEX_L1CA);
    let pseudorange = travel_time * LIGHT_SPEED;

    assert_eq!(sat_param.svid, TARGET_SVID as i32);
    assert!(sat_param.Elevation > 0.0);
    assert!((0.0..std::f64::consts::TAU).contains(&sat_param.Azimuth));
    assert!(
        doppler.abs() < 6_000.0,
        "unexpected L1CA doppler: {doppler}"
    );
    assert!(
        (60_000.0..90_000.0).contains(&(travel_time * 1e6)),
        "unexpected travel time: {travel_time}"
    );
    assert!(
        (18_000_000.0..27_000_000.0).contains(&pseudorange),
        "unexpected pseudorange: {pseudorange}"
    );
    assert!((0.0..100.0).contains(&sat_param.IonoDelay));

    let mut elevation = 0.0;
    let mut azimuth = 0.0;
    sat_el_az_from_positions(
        &receiver_ecef,
        &sat_param.PosVel,
        &mut elevation,
        &mut azimuth,
    );
    assert!((elevation - sat_param.Elevation).abs() < 0.02);
    assert!((0.0..std::f64::consts::TAU).contains(&azimuth));
}

#[test]
fn ionosphere_and_troposphere_delay_models_return_bounded_values() {
    let nav_data = load_nav_data();
    let receiver_lla = receiver_lla();
    let iono = nav_data
        .get_gps_iono_alpha()
        .zip(nav_data.get_gps_iono_beta())
        .map(|(alpha, beta)| IonoParam {
            a0: alpha[0],
            a1: alpha[1],
            a2: alpha[2],
            a3: alpha[3],
            b0: beta[0],
            b1: beta[1],
            b2: beta[2],
            b3: beta[3],
            flag: 1,
        })
        .expect("GPS ionosphere parameters should be present");

    let elevation = 45.0_f64.to_radians();
    let azimuth = 120.0_f64.to_radians();
    let iono_delay = gps_iono_delay(
        &iono,
        TARGET_GPS_TIME.MilliSeconds as f64 / 1000.0,
        receiver_lla.lat,
        receiver_lla.lon,
        elevation,
        azimuth,
    );
    let tropo = tropo_delay(receiver_lla.lat, receiver_lla.alt, elevation);

    assert!((0.0..100.0).contains(&iono_delay));
    assert!((1.0..10.0).contains(&tropo));
    assert_eq!(tropo_delay(receiver_lla.lat, receiver_lla.alt, -0.1), 0.0);
}
