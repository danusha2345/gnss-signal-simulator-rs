use gnss_rust::gnsstime::{
    bds_time_to_utc, galileo_time_to_utc, gps_time_to_utc, utc_to_bds_time, utc_to_galileo_time,
    utc_to_glonass_time_corrected, utc_to_gps_time,
};
use gnss_rust::types::{GnssTime, UtcTime};

fn utc_2025_june_5() -> UtcTime {
    UtcTime {
        Year: 2025,
        Month: 6,
        Day: 5,
        Hour: 10,
        Minute: 5,
        Second: 30.25,
    }
}

#[test]
fn gps_bds_and_galileo_offsets_match_reference_epoch() {
    let utc = utc_2025_june_5();

    // Second = 30.25 → 30250 ms exactly: the 0.25 s lives in MilliSeconds, so the
    // sub-millisecond remainder is 0.0 (previously SubMilliSeconds wrongly held 0.25 s,
    // double-counting the fraction — see utc_to_gps_time fix / audit H6).
    let gps = utc_to_gps_time(utc, true);
    assert_eq!(gps.Week, 2369);
    assert_eq!(gps.MilliSeconds, 381_948_250);
    assert!(gps.SubMilliSeconds.abs() < f64::EPSILON);

    let bds = utc_to_bds_time(utc);
    assert_eq!(bds.Week, 1013);
    assert_eq!(bds.MilliSeconds, 381_934_250);
    assert!(bds.SubMilliSeconds.abs() < f64::EPSILON);
    assert_eq!(gps.Week - bds.Week, 1356);
    assert_eq!(gps.MilliSeconds - bds.MilliSeconds, 14_000);

    let galileo = utc_to_galileo_time(utc);
    assert_eq!(galileo.Week, 1345);
    assert_eq!(galileo.MilliSeconds, 381_948_250);
    assert!(galileo.SubMilliSeconds.abs() < f64::EPSILON);
    assert_eq!(gps.Week - galileo.Week, 1024);
    assert_eq!(gps.MilliSeconds, galileo.MilliSeconds);
}

#[test]
fn gps_epoch_without_leap_seconds_starts_at_zero() {
    let gps_epoch = UtcTime {
        Year: 1980,
        Month: 1,
        Day: 6,
        Hour: 0,
        Minute: 0,
        Second: 0.0,
    };

    let gps = utc_to_gps_time(gps_epoch, false);
    assert_eq!(gps.Week, 0);
    assert_eq!(gps.MilliSeconds, 0);
    assert_eq!(gps.SubMilliSeconds, 0.0);
}

#[test]
fn utc_to_gps_keeps_sub_millisecond_remainder() {
    // Regression for audit H6: SubMilliSeconds must be the fraction of ONE millisecond
    // [0,1), not the fraction of a second. 30.2505 s = 30250.5 ms → 30250 ms goes into
    // MilliSeconds and 0.5 into SubMilliSeconds, so (MilliSeconds + SubMilliSeconds)/1000
    // reconstructs the exact time-of-week with no double counting of the sub-second part.
    let utc = UtcTime {
        Year: 2025,
        Month: 6,
        Day: 5,
        Hour: 10,
        Minute: 5,
        Second: 30.2505,
    };
    let gps = utc_to_gps_time(utc, true);
    assert_eq!(gps.MilliSeconds, 381_948_250);
    assert!(
        (gps.SubMilliSeconds - 0.5).abs() < 1e-9,
        "SubMilliSeconds={}",
        gps.SubMilliSeconds
    );
    let tow = (gps.MilliSeconds as f64 + gps.SubMilliSeconds) / 1000.0;
    assert!((tow - 381_948.250_5).abs() < 1e-6, "tow={tow}");
}

#[test]
fn gnss_time_add_milliseconds_wraps_week_boundaries() {
    let end_of_week = GnssTime {
        Week: 2369,
        MilliSeconds: 604_799_900,
        SubMilliSeconds: 0.5,
    };
    let next_week = end_of_week.add_milliseconds(250.0);
    assert_eq!(next_week.Week, 2370);
    assert_eq!(next_week.MilliSeconds, 150);
    assert_eq!(next_week.SubMilliSeconds, 0.5);

    let start_of_week = GnssTime {
        Week: 2370,
        MilliSeconds: 100,
        SubMilliSeconds: 0.25,
    };
    let previous_week = start_of_week.add_milliseconds(-250.0);
    assert_eq!(previous_week.Week, 2369);
    assert_eq!(previous_week.MilliSeconds, 604_799_850);
    assert_eq!(previous_week.SubMilliSeconds, 0.25);
}

#[test]
fn glonass_corrected_time_applies_three_hour_offset() {
    let glonass = utc_to_glonass_time_corrected(utc_2025_june_5());

    assert_eq!(glonass.LeapYear, 7);
    assert_eq!(glonass.Day, 10_749);
    assert_eq!(glonass.MilliSeconds, 47_130_250);
    assert_eq!(glonass.SubMilliSeconds, 0.0);
}

#[test]
fn gnss_time_roundtrips_preserve_calendar_date() {
    let utc = utc_2025_june_5();

    let gps_utc = gps_time_to_utc(utc_to_gps_time(utc, true), true);
    assert_eq!((gps_utc.Year, gps_utc.Month, gps_utc.Day), (2025, 6, 5));
    assert_eq!((gps_utc.Hour, gps_utc.Minute), (10, 5));
    assert!((gps_utc.Second - 30.25).abs() < 0.001);

    let bds_utc = bds_time_to_utc(utc_to_bds_time(utc));
    assert_eq!((bds_utc.Year, bds_utc.Month, bds_utc.Day), (2025, 6, 5));
    assert_eq!((bds_utc.Hour, bds_utc.Minute), (10, 5));

    let galileo_utc = galileo_time_to_utc(utc_to_galileo_time(utc));
    assert_eq!(
        (galileo_utc.Year, galileo_utc.Month, galileo_utc.Day),
        (2025, 6, 5)
    );
    assert_eq!((galileo_utc.Hour, galileo_utc.Minute), (10, 5));
}

#[test]
fn fractional_time_arithmetic_and_transmit_week_rollover() {
    let time = GnssTime {
        Week: 2369,
        MilliSeconds: 0,
        SubMilliSeconds: 0.25,
    };
    let previous = time.add_milliseconds(-0.5);
    assert_eq!(previous.Week, 2368);
    assert_eq!(previous.MilliSeconds, 604799999);
    assert_eq!(previous.SubMilliSeconds, 0.75);
    let previous = gnss_rust::satellite_param::get_transmit_time(&time, 0.075);
    assert_eq!(previous.Week, 2368);
    assert_eq!(previous.MilliSeconds, 604799925);
    assert_eq!(previous.SubMilliSeconds, 0.25);
    let utc = UtcTime {
        Year: 2025,
        Month: 12,
        Day: 31,
        Hour: 23,
        Minute: 30,
        Second: 0.0,
    };
    let glo = gnss_rust::gnsstime::utc_to_glonass_time_corrected(utc);
    assert_eq!(glo.MilliSeconds, 9000000);
    assert_eq!((glo.Day - 1) % 1461 + 1, 732);
}
