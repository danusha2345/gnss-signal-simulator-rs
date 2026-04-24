use gnss_rust::complex_number::ComplexNumber;
use gnss_rust::constants::{
    FREQ_GLO_G1, SIGNAL_INDEX_B1C, SIGNAL_INDEX_E1, SIGNAL_INDEX_G1, SIGNAL_INDEX_G3,
    SIGNAL_INDEX_L1CA, SIGNAL_INDEX_L5,
};
use gnss_rust::nav_data::NavData;
use gnss_rust::sat_if_signal::{PrnCache, SatIfSignal};
use gnss_rust::satellite_signal::SatelliteSignal;
use gnss_rust::types::{GnssSystem, GnssTime, SatelliteParam};
use gnss_rust::SafeAvx512Processor;

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

fn signal_power(samples: &[ComplexNumber]) -> f64 {
    samples
        .iter()
        .map(|sample| sample.real * sample.real + sample.imag * sample.imag)
        .sum()
}

fn assert_finite_nonzero_samples(samples: &[ComplexNumber], name: &str) -> f64 {
    assert!(
        !samples.is_empty(),
        "{name} sample block should not be empty"
    );
    assert!(
        samples
            .iter()
            .all(|sample| sample.real.is_finite() && sample.imag.is_finite()),
        "{name} sample block should be finite"
    );

    let total_power = signal_power(samples);
    assert!(total_power > 0.0, "{name} sample block should have power");
    total_power
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
fn prn_cache_maps_binary_chips_to_bipolar_values_and_expires_by_millisecond() {
    let mut cache = PrnCache::new();

    assert!(cache.needs_update(42));
    cache.update_cache(&[0, 1, 1, 0], 42);

    assert!(!cache.needs_update(42));
    assert!(cache.needs_update(43));
    assert_eq!(cache.get_prn_bit(0), 1.0);
    assert_eq!(cache.get_prn_bit(1), -1.0);
    assert_eq!(cache.get_prn_bit(2), -1.0);
    assert_eq!(cache.get_prn_bit(3), 1.0);
    assert_eq!(cache.get_prn_bit(4), 1.0);
}

#[test]
fn cached_if_generation_handles_modern_boc_and_glonass_signals() {
    let cases = [
        (
            "BDS B1C",
            GnssSystem::BdsSystem,
            SIGNAL_INDEX_B1C,
            19,
            0,
            137_000,
        ),
        (
            "Galileo E1",
            GnssSystem::GalileoSystem,
            SIGNAL_INDEX_E1,
            8,
            0,
            137_000,
        ),
        (
            "GLONASS G3",
            GnssSystem::GlonassSystem,
            SIGNAL_INDEX_G3,
            5,
            0,
            137_000,
        ),
        (
            "GLONASS G1 FDMA",
            GnssSystem::GlonassSystem,
            SIGNAL_INDEX_G1,
            7,
            1,
            0,
        ),
    ];
    let cur_time = t(381_948_321);

    for (name, system, signal_index, svid, freq_id, if_freq) in cases {
        let mut sat_param = sample_param(system, svid);
        sat_param.FreqID = freq_id;
        let mut channel = SatIfSignal::new(96, if_freq, system, signal_index as i32, svid as u8);

        channel.init_state(cur_time, &sat_param, None);
        if system == GnssSystem::GlonassSystem && signal_index == SIGNAL_INDEX_G1 {
            channel.push_sat_param_for_ms(&sat_param, FREQ_GLO_G1, &cur_time);
        }
        channel.get_if_sample_cached(cur_time);

        assert_finite_nonzero_samples(&channel.sample_array, name);
        let real_energy: f64 = channel
            .sample_array
            .iter()
            .map(|sample| sample.real * sample.real)
            .sum();
        let imag_energy: f64 = channel
            .sample_array
            .iter()
            .map(|sample| sample.imag * sample.imag)
            .sum();
        assert!(
            real_energy > 0.0,
            "{name} real component should have energy"
        );
        assert!(
            imag_energy > 0.0,
            "{name} imag component should have energy"
        );
    }
}

#[test]
fn block_generation_matches_repeated_cached_millisecond_generation() {
    let samples_per_ms = 64;
    let duration_ms = 3;
    let start_time = t(381_948_020);
    let sat_param = sample_param(GnssSystem::GpsSystem, 1);

    let mut block_channel = SatIfSignal::new(
        samples_per_ms,
        125_000,
        GnssSystem::GpsSystem,
        SIGNAL_INDEX_L1CA as i32,
        1,
    );
    block_channel.init_state(start_time, &sat_param, None);
    block_channel.generate_block_signal_parallel(
        start_time,
        duration_ms,
        samples_per_ms as usize,
        samples_per_ms as f64 * 1000.0,
    );

    let block_data = block_channel
        .block_data
        .as_ref()
        .expect("block generation should allocate output");
    assert_eq!(
        block_data.len(),
        samples_per_ms as usize * duration_ms as usize
    );
    assert_finite_nonzero_samples(block_data, "block generator");

    let mut step_channel = SatIfSignal::new(
        samples_per_ms,
        125_000,
        GnssSystem::GpsSystem,
        SIGNAL_INDEX_L1CA as i32,
        1,
    );
    step_channel.init_state(start_time, &sat_param, None);

    for ms_offset in 0..duration_ms {
        let cur_time = t(start_time.MilliSeconds + ms_offset);
        step_channel.get_if_sample_cached(cur_time);
        let start = ms_offset as usize * samples_per_ms as usize;
        let end = start + samples_per_ms as usize;

        for (block_sample, step_sample) in block_data[start..end]
            .iter()
            .zip(step_channel.sample_array.iter())
        {
            assert_close(block_sample.real, step_sample.real, "block real");
            assert_close(block_sample.imag, step_sample.imag, "block imag");
        }
    }
}

#[test]
fn avx512_l1ca_path_matches_cached_generation_across_code_wrap() {
    let processor = SafeAvx512Processor::new();
    if !processor.is_available() {
        return;
    }

    let samples_per_ms = 64;
    let cur_time = t(381_948_321);
    let mut sat_param = sample_param(GnssSystem::GpsSystem, 1);
    // Leaves transmit sub-millisecond time close to 1.0, so the first ms crosses
    // the GPS L1 C/A 1023-chip boundary and exercises wrap handling.
    sat_param.TravelTime = 0.000_001;

    let mut cached_channel = SatIfSignal::new(
        samples_per_ms,
        125_000,
        GnssSystem::GpsSystem,
        SIGNAL_INDEX_L1CA as i32,
        1,
    );
    let mut accelerated_channel = SatIfSignal::new(
        samples_per_ms,
        125_000,
        GnssSystem::GpsSystem,
        SIGNAL_INDEX_L1CA as i32,
        1,
    );

    cached_channel.init_state(cur_time, &sat_param, None);
    accelerated_channel.init_state(cur_time, &sat_param, None);

    cached_channel.get_if_sample_cached(cur_time);
    accelerated_channel.get_if_sample_avx512_accelerated(cur_time, &processor);

    for (cached_sample, accelerated_sample) in cached_channel
        .sample_array
        .iter()
        .zip(accelerated_channel.sample_array.iter())
    {
        assert_close(accelerated_sample.real, cached_sample.real, "avx512 real");
        assert_close(accelerated_sample.imag, cached_sample.imag, "avx512 imag");
    }
}

#[test]
fn glonass_fdma_frequency_slot_changes_cached_carrier_samples() {
    let cur_time = t(381_948_321);
    let mut slot_zero_param = sample_param(GnssSystem::GlonassSystem, 7);
    slot_zero_param.FreqID = 0;
    let mut slot_one_param = slot_zero_param;
    slot_one_param.FreqID = 1;

    let mut slot_zero =
        SatIfSignal::new(96, 0, GnssSystem::GlonassSystem, SIGNAL_INDEX_G1 as i32, 7);
    let mut slot_one =
        SatIfSignal::new(96, 0, GnssSystem::GlonassSystem, SIGNAL_INDEX_G1 as i32, 7);

    slot_zero.init_state(cur_time, &slot_zero_param, None);
    slot_one.init_state(cur_time, &slot_one_param, None);
    slot_zero.push_sat_param_for_ms(&slot_zero_param, FREQ_GLO_G1, &cur_time);
    slot_one.push_sat_param_for_ms(&slot_one_param, FREQ_GLO_G1, &cur_time);
    slot_zero.get_if_sample_cached(cur_time);
    slot_one.get_if_sample_cached(cur_time);

    assert_finite_nonzero_samples(&slot_zero.sample_array, "GLONASS slot zero");
    assert_finite_nonzero_samples(&slot_one.sample_array, "GLONASS slot one");

    let total_delta: f64 = slot_zero
        .sample_array
        .iter()
        .zip(slot_one.sample_array.iter())
        .map(|(left, right)| (left.real - right.real).abs() + (left.imag - right.imag).abs())
        .sum();
    assert!(
        total_delta > 1.0e-6,
        "GLONASS FDMA slot must affect carrier samples"
    );
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
