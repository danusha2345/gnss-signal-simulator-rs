//! Выбор Galileo RINEX по источнику часов/навигации; порядок записей не влияет.
use crate::constants::*;
use crate::types::*;

pub fn select_ephemeris(
    records: &[GpsEphemeris],
    time: GnssTime,
    svid: u8,
    signal: usize,
) -> Option<GpsEphemeris> {
    records
        .iter()
        .filter(|e| e.svid == svid && e.valid != 0 && e.ura >= 0 && e.ura <= 125)
        .filter(|e| match signal {
            SIGNAL_INDEX_E5A => e.source & 0x102 == 0x102 && e.health & 0x38 == 0,
            SIGNAL_INDEX_E5B => e.source & 5 != 0 && e.source & 0x200 != 0 && e.health & 0x1c0 == 0,
            _ => e.source & 5 != 0 && e.source & 0x200 != 0 && e.health & 7 == 0,
        })
        .filter_map(|e| {
            let dt = (time.Week as i64 - e.week as i64) * 604800 + time.MilliSeconds as i64 / 1000
                - e.toe as i64;
            if dt.abs() > 10800 {
                None
            } else {
                // Спутник передаёт батч только после его toe (RINEX: время передачи ≈ toe+11 мин).
                // Будущий toe телефон отбраковывает, поэтому сначала последний прошедший.
                Some((
                    e,
                    (
                        dt < 0,
                        dt.abs(),
                        -(e.week as i64 * 604800 + e.toe as i64),
                        std::cmp::Reverse(e.iodc),
                        std::cmp::Reverse(e.source),
                        e.af0.to_bits(),
                        e.af1.to_bits(),
                    ),
                ))
            }
        })
        .min_by_key(|(_, key)| *key)
        .map(|(e, _)| *e)
}

/// Групповые задержки относительно часов соответствующей ionosphere-free пары.
/// Для f_low: gamma*BGD, gamma=(f_E1/f_low)^2; для E1: BGD.
pub fn group_delays(e: &GpsEphemeris) -> [f64; 8] {
    let mut d = [0.0; 8];
    let bgd = if e.source & 0x100 != 0 {
        d[1] = (1575.42_f64 / 1176.45).powi(2) * e.tgd;
        e.tgd
    } else {
        d[2] = (1575.42_f64 / 1207.14).powi(2) * e.tgd2;
        e.tgd2
    };
    d[0] = bgd;
    // E5 AltBOC: отдельного BGD нет; масштабируем доступный BGD на частоту 1191.795 МГц.
    d[3] = (1575.42_f64 / 1191.795).powi(2) * bgd;
    d
}
