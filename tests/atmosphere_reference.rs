//! Значения получены отдельной компиляцией RTKLIB rtklib_2.4.3 rtkcmn.c::ionmodel.
//! https://github.com/tomojitakasu/RTKLIB/blob/rtklib_2.4.3/src/rtkcmn.c
use gnss_rust::{coordinate::gps_iono_delay, types::IonoParam};
#[test]
fn klobuchar_agrees_with_external_reference() {
    let p = IonoParam {
        a0: 1.118e-8,
        a1: -7.451e-9,
        a2: -5.961e-8,
        a3: 1.192e-7,
        b0: 116700.0,
        b1: -229400.0,
        b2: -131100.0,
        b3: 1049000.0,
        flag: 1,
    };
    let expected = [
        4.464_072_335_272_207,
        4.720_338_374_599_844,
        6.224_194_334_388_693,
        6.239_772_055_983_21,
        4.6329539783509235,
        2.4696640927216618,
        1.7264785390831474,
        1.6046426630548871,
    ];
    for (i, expected) in expected.into_iter().enumerate() {
        let actual = gps_iono_delay(
            &p,
            i as f64 * 79123.75,
            0.7,
            -1.1,
            0.1 + i as f64 * 0.15,
            i as f64 * 0.7,
        );
        assert!((actual - expected).abs() < 1e-12, "{actual} != {expected}");
    }
}
