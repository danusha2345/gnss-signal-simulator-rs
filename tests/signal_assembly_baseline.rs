use gnss_rust::complex_number::ComplexNumber;
use gnss_rust::constants::{
    SIGNAL_INDEX_B1C, SIGNAL_INDEX_E1, SIGNAL_INDEX_G3, SIGNAL_INDEX_L1CA, SIGNAL_INDEX_L5,
};
use gnss_rust::nav_data::NavData;
use gnss_rust::sat_if_signal::SatIfSignal;
use gnss_rust::satellite_signal::SatelliteSignal;
use gnss_rust::types::{GnssSystem, GnssTime, SatelliteParam};

fn t(ms: i32) -> GnssTime {
    GnssTime {
        Week: 2369,
        MilliSeconds: ms,
        SubMilliSeconds: 0.0,
    }
}

fn assert_close(actual: f64, expected: f64, name: &str) {
    assert!(
        (actual - expected).abs() < 1e-12,
        "{name}: expected {expected}, got {actual}"
    );
}

fn assert_finite_signal(signal: &ComplexNumber, name: &str) {
    assert!(signal.real.is_finite(), "{name} real part is not finite");
    assert!(signal.imag.is_finite(), "{name} imag part is not finite");
}

fn sample_param(system: GnssSystem, svid: i32) -> SatelliteParam {
    SatelliteParam {
        system,
        svid,
        CN0: 4_500,
        TravelTime: 0.074,
        ..SatelliteParam::default()
    }
}

#[test]
fn satellite_signal_component_mapping_matches_supported_modulations() {
    let cases = [
        (
            "GPS L1CA",
            GnssSystem::GpsSystem,
            SIGNAL_INDEX_L1CA,
            1,
            1.0,
            0.0,
            0.0,
            0.0,
        ),
        (
            "GPS L5",
            GnssSystem::GpsSystem,
            SIGNAL_INDEX_L5,
            4,
            0.0,
            std::f64::consts::FRAC_1_SQRT_2,
            std::f64::consts::FRAC_1_SQRT_2,
            0.0,
        ),
        (
            "BDS B1C",
            GnssSystem::BdsSystem,
            SIGNAL_INDEX_B1C,
            19,
            0.0,
            0.5,
            0.811_844_540_519_421_5,
            0.0,
        ),
        (
            "Galileo E1",
            GnssSystem::GalileoSystem,
            SIGNAL_INDEX_E1,
            8,
            std::f64::consts::FRAC_1_SQRT_2,
            0.0,
            std::f64::consts::FRAC_1_SQRT_2,
            0.0,
        ),
        (
            "GLONASS G3",
            GnssSystem::GlonassSystem,
            SIGNAL_INDEX_G3,
            5,
            std::f64::consts::FRAC_1_SQRT_2,
            0.0,
            0.0,
            0.0,
        ),
    ];

    for (
        name,
        system,
        signal_index,
        svid,
        expected_data_real,
        expected_data_imag,
        expected_pilot_real,
        expected_pilot_imag,
    ) in cases
    {
        let mut signal = SatelliteSignal::new();
        assert!(signal.set_signal_attribute(system, signal_index as i32, None, svid));

        let mut data_signal = ComplexNumber::new();
        let mut pilot_signal = ComplexNumber::new();
        assert!(signal.get_satellite_signal(t(20_000), &mut data_signal, &mut pilot_signal));

        assert_finite_signal(&data_signal, name);
        assert_finite_signal(&pilot_signal, name);
        assert_close(data_signal.real.abs(), expected_data_real, name);
        assert_close(data_signal.imag.abs(), expected_data_imag, name);
        assert_close(pilot_signal.real.abs(), expected_pilot_real, name);
        assert_close(pilot_signal.imag.abs(), expected_pilot_imag, name);
    }
}

#[test]
fn satellite_signal_rejects_mismatched_navigation_data_type() {
    let wrong_nav_data = NavData::new_gps(1).expect("GPS LNAV data should be constructible");
    let mut signal = SatelliteSignal::new();

    assert!(!signal.set_signal_attribute(
        GnssSystem::GalileoSystem,
        SIGNAL_INDEX_E1 as i32,
        Some(wrong_nav_data),
        1,
    ));
}

#[test]
fn sat_if_signal_generates_finite_nonzero_one_millisecond_block() {
    let mut channel = SatIfSignal::new(
        32,
        125_000,
        GnssSystem::GpsSystem,
        SIGNAL_INDEX_L1CA as i32,
        1,
    );
    let cur_time = t(381_948_000);
    let sat_param = sample_param(GnssSystem::GpsSystem, 1);

    channel.init_state(cur_time, &sat_param, None);
    channel.get_if_sample(cur_time);

    assert_eq!(channel.sample_array.len(), 32);
    assert!(channel
        .sample_array
        .iter()
        .all(|sample| sample.real.is_finite() && sample.imag.is_finite()));

    let total_power: f64 = channel
        .sample_array
        .iter()
        .map(|sample| sample.real * sample.real + sample.imag * sample.imag)
        .sum();
    assert!(
        total_power > 0.0,
        "generated sample block should have power"
    );
    assert!(
        total_power < 128.0,
        "generated sample block power is too large"
    );
}
