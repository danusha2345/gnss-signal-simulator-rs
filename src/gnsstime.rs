//! # Системы времени ГНСС
//!
//! Модуль для работы с различными системами времени, используемыми
//! в спутниковых навигационных системах.
//!
//! ## Поддерживаемые системы времени:
//! - **GPS время** - начало эпохи 6 января 1980 г.
//! - **ГЛОНАСС время** - московское время, начало 1 января 1996 г.
//! - **BeiDou время** - начало эпохи 1 января 2006 г.
//! - **Galileo время** - синхронизировано с GPS
//! - **UTC** - Координированное всемирное время
//!
//! ## Основные функции:
//! - Преобразование между системами времени
//! - Учет високосных секунд
//! - Работа с неделями и секундами недели
//! - Вычисление дня года (DOY)
//! - Коррекция часовых поясов
//!
//! Портировано с C++ версии для обеспечения точных временных расчетов.

//----------------------------------------------------------------------
// gnsstime.rs:
//   Conversion between GNSS time systems
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use crate::types::*;

static DAYS_ACC: [i32; 12] = [0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334];

static INSERT_TIME: [u32; 18] = [
    46828800, 78364801, 109900802, 173059203, 252028804, 315187205,
    346723206, 393984007, 425520008, 457056009, 504489610, 551750411,
    599184012, 820108813, 914803214, 1025136015, 1119744016, 1167264017
];

pub struct GnssTimeConverter;

impl GnssTimeConverter {
    pub fn get_leap_second(seconds: u32) -> (i32, bool) {
        for (i, &insert_time) in INSERT_TIME.iter().enumerate() {
            if seconds <= insert_time {
                return (i as i32, seconds == insert_time);
            }
        }
        (INSERT_TIME.len() as i32, false)
    }

    // This program handles the date from Jan. 1, 1984 00:00:00.00 UTC till year 2099
    // date after 2020/12/31 may not have correct leap second correction
    pub fn gps_time_to_utc(gnss_time: GnssTime, use_leap_second: bool) -> UtcTime {
        let mut leap_second = 0i32;
        let mut at_leap_second = use_leap_second;

        // Calculate total days and seconds
        // To prevent seconds less than zero after leap second adjust
        // add seconds of one week
        let mut glonass_time = GlonassTime {
            Day: (gnss_time.Week - 1) * 7,
            MilliSeconds: gnss_time.MilliSeconds + 604800000,
            SubMilliSeconds: gnss_time.SubMilliSeconds,
            LeapYear: 0,
        };

        if use_leap_second {
            let (ls, at_ls) = Self::get_leap_second((gnss_time.Week * 604800 + gnss_time.MilliSeconds / 1000) as u32);
            leap_second = ls;
            at_leap_second = at_ls;
        }

        glonass_time.MilliSeconds -= if at_leap_second { (leap_second + 1) * 1000 } else { leap_second * 1000 };

        let days = glonass_time.MilliSeconds / 86400000;
        glonass_time.Day += days;
        glonass_time.MilliSeconds -= days * 86400000;
        glonass_time.Day -= 208 * 7;

        // Calculate year
        glonass_time.LeapYear = glonass_time.Day / (366 + 365 * 3);
        glonass_time.Day -= glonass_time.LeapYear * (366 + 365 * 3);
        glonass_time.LeapYear -= 2;
        glonass_time.Day += 1;
        glonass_time.MilliSeconds += 10800000;

        let mut time = Self::glonass_time_to_utc(glonass_time);
        if at_leap_second {
            time.Second += 1.0;
        }

        time
    }

    pub fn utc_to_gps_time(utc_time: UtcTime, use_leap_second: bool) -> GnssTime {
        let glonass_time = Self::utc_to_glonass_time(utc_time);
        let total_days = (glonass_time.LeapYear + 3) * (366 + 365 * 3) + glonass_time.Day - 6;
        let mut total_seconds = (total_days * 86400 + glonass_time.MilliSeconds / 1000 - 10800) as u32;
        let temp_seconds = total_seconds;
        let next_day = utc_time.Hour == 0 && utc_time.Minute == 0 && (utc_time.Second as i32) == 0;

        if use_leap_second {
            let (mut leap_second, _) = Self::get_leap_second(temp_seconds);
            let temp_seconds_adj = temp_seconds + leap_second as u32;
            let (leap_second_adj, at_leap_second) = Self::get_leap_second(temp_seconds_adj);
            leap_second = leap_second_adj;
            total_seconds += if at_leap_second && next_day { 
                (leap_second + 1) as u32 
            } else { 
                leap_second as u32 
            };
        }

        let week = (total_seconds / 604800) as i32;
        let mut milli_seconds = ((total_seconds - (week as u32) * 604800) * 1000 + (glonass_time.MilliSeconds % 1000) as u32) as i32;

        GnssTime {
            Week: week,
            MilliSeconds: milli_seconds,
            SubMilliSeconds: glonass_time.SubMilliSeconds,
        }
    }

    pub fn glonass_time_to_utc(mut glonass_time: GlonassTime) -> UtcTime {
        let mut leap_day = 0;

        glonass_time.MilliSeconds -= 10800000;
        if glonass_time.MilliSeconds < 0 {
            glonass_time.MilliSeconds += 86400000;
            glonass_time.Day -= 1;
        }

        glonass_time.LeapYear *= 4;
        glonass_time.Day -= 1;

        if glonass_time.Day >= (366 + 365 * 2) {
            glonass_time.Day -= 366 + 365 * 2;
            glonass_time.LeapYear += 3;
        } else if glonass_time.Day >= (366 + 365) {
            glonass_time.Day -= 366 + 365;
            glonass_time.LeapYear += 2;
        } else if glonass_time.Day >= 366 {
            glonass_time.Day -= 366;
            glonass_time.LeapYear += 1;
        } else if glonass_time.Day >= 60 {
            glonass_time.Day -= 1;
        } else if glonass_time.Day == 59 {
            leap_day = 1;
        }

        let mut i = 1;
        while i < 12 {
            if glonass_time.Day < DAYS_ACC[i] {
                break;
            }
            i += 1;
        }

        let (month, day) = if leap_day != 0 {
            (2, 29)
        } else {
            (i as i32, glonass_time.Day - (DAYS_ACC[i - 1] - 1))
        };

        let year = 1992 + glonass_time.LeapYear;
        let mut seconds = glonass_time.MilliSeconds / 1000;
        let hour = seconds / 3600;
        seconds -= hour * 3600;
        let minute = seconds / 60;
        seconds = glonass_time.MilliSeconds % 60000;
        let second = (seconds as f64 + glonass_time.SubMilliSeconds) / 1000.0;

        UtcTime {
            Year: year,
            Month: month,
            Day: day,
            Hour: hour,
            Minute: minute,
            Second: second,
        }
    }

    pub fn utc_to_glonass_time(utc_time: UtcTime) -> GlonassTime {
        let milli_seconds_f = utc_time.Second * 1000.0;
        let milli_seconds = (((utc_time.Hour * 60) + utc_time.Minute) * 60000 + milli_seconds_f as i32) + 10800000;
        let sub_milli_seconds = milli_seconds_f - milli_seconds_f as i32 as f64;

        let years = utc_time.Year - 1992;
        let mut days = DAYS_ACC[(utc_time.Month - 1) as usize] + utc_time.Day - 1;

        if (years % 4) != 0 || days >= 59 {
            days += 1;
        }
        days += (years % 4) * 365;

        GlonassTime {
            Day: days + 1,
            LeapYear: years / 4,
            MilliSeconds: milli_seconds,
            SubMilliSeconds: sub_milli_seconds,
        }
    }

    pub fn utc_to_glonass_time_corrected(utc_time: UtcTime) -> GlonassTime {
        let milli_seconds_f = utc_time.Second * 1000.0;
        let milli_seconds = (((utc_time.Hour * 60) + utc_time.Minute) * 60000 + milli_seconds_f as i32) + 10800000;
        let sub_milli_seconds = milli_seconds_f - milli_seconds_f as i32 as f64;

        let years = utc_time.Year - 1996;
        let mut days = DAYS_ACC[(utc_time.Month - 1) as usize] + utc_time.Day - 1;

        // Handle leap year correctly
        let years_in_cycle = years % 4;
        if years_in_cycle == 0 {
            // First year of cycle is leap year
            if days >= 59 {
                days += 1;
            }
        } else {
            // Not first year of cycle - add 1 for non-leap adjustment
            days += 1;
            // Add days from previous years in cycle
            if years_in_cycle >= 1 { days += 366; }  // First year was leap
            if years_in_cycle >= 2 { days += 365; }  // Second year
            if years_in_cycle >= 3 { days += 365; }  // Third year
        }

        // Calculate total day number from epoch
        let day = days + 1 + (years / 4) * 1461;
        let leap_year = years / 4;

        GlonassTime {
            Day: day,
            LeapYear: leap_year,
            MilliSeconds: milli_seconds,
            SubMilliSeconds: sub_milli_seconds,
        }
    }

    pub fn utc_to_bds_time(utc_time: UtcTime) -> GnssTime {
        let mut time = Self::utc_to_gps_time(utc_time, true);

        if time.MilliSeconds >= 14000 {
            time.MilliSeconds -= 14000;
            time.Week -= 1356;
        } else {
            time.MilliSeconds += 604800000 - 14000;
            time.Week -= 1357;
        }

        time
    }

    pub fn utc_to_galileo_time(utc_time: UtcTime) -> GnssTime {
        let mut time = Self::utc_to_gps_time(utc_time, true);
        time.Week -= 1024;
        time
    }

    pub fn bds_time_to_utc(mut gnss_time: GnssTime) -> UtcTime {
        gnss_time.MilliSeconds += 14000;
        gnss_time.Week += 1356;
        Self::gps_time_to_utc(gnss_time, true)
    }

    pub fn galileo_time_to_utc(mut gnss_time: GnssTime) -> UtcTime {
        gnss_time.Week += 1024;
        Self::gps_time_to_utc(gnss_time, true)
    }
}

// Convenience functions for global access
pub fn gps_time_to_utc(gnss_time: GnssTime, use_leap_second: bool) -> UtcTime {
    GnssTimeConverter::gps_time_to_utc(gnss_time, use_leap_second)
}

pub fn utc_to_gps_time(utc_time: UtcTime, use_leap_second: bool) -> GnssTime {
    GnssTimeConverter::utc_to_gps_time(utc_time, use_leap_second)
}

pub fn glonass_time_to_utc(glonass_time: GlonassTime) -> UtcTime {
    GnssTimeConverter::glonass_time_to_utc(glonass_time)
}

pub fn utc_to_glonass_time(utc_time: UtcTime) -> GlonassTime {
    GnssTimeConverter::utc_to_glonass_time(utc_time)
}

pub fn utc_to_glonass_time_corrected(utc_time: UtcTime) -> GlonassTime {
    GnssTimeConverter::utc_to_glonass_time_corrected(utc_time)
}

pub fn utc_to_bds_time(utc_time: UtcTime) -> GnssTime {
    GnssTimeConverter::utc_to_bds_time(utc_time)
}

pub fn utc_to_galileo_time(utc_time: UtcTime) -> GnssTime {
    GnssTimeConverter::utc_to_galileo_time(utc_time)
}

pub fn bds_time_to_utc(gnss_time: GnssTime) -> UtcTime {
    GnssTimeConverter::bds_time_to_utc(gnss_time)
}

pub fn galileo_time_to_utc(gnss_time: GnssTime) -> UtcTime {
    GnssTimeConverter::galileo_time_to_utc(gnss_time)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_leap_second() {
        let (leap_sec, at_leap) = GnssTimeConverter::get_leap_second(46828800);
        assert_eq!(leap_sec, 0);
        assert_eq!(at_leap, true);
    }

    #[test]
    fn test_utc_to_gps_conversion() {
        let utc_time = UtcTime {
            Year: 2000,
            Month: 1,
            Day: 1,
            Hour: 12,
            Minute: 0,
            Second: 0.0,
        };

        let gps_time = utc_to_gps_time(utc_time, true);
        let back_to_utc = gps_time_to_utc(gps_time, true);

        // Check that conversion is approximately correct (within 1 second)
        assert_eq!(back_to_utc.Year, utc_time.Year);
        assert_eq!(back_to_utc.Month, utc_time.Month);
        assert_eq!(back_to_utc.Day, utc_time.Day);
        assert_eq!(back_to_utc.Hour, utc_time.Hour);
        assert_eq!(back_to_utc.Minute, utc_time.Minute);
        assert!((back_to_utc.Second - utc_time.Second).abs() < 1.0);
    }

    #[test]
    fn test_glonass_time_conversion() {
        let utc_time = UtcTime {
            Year: 2000,
            Month: 6,
            Day: 15,
            Hour: 18,
            Minute: 30,
            Second: 45.5,
        };

        let glonass_time = utc_to_glonass_time(utc_time);
        let back_to_utc = glonass_time_to_utc(glonass_time);

        // Check that conversion is approximately correct
        assert_eq!(back_to_utc.Year, utc_time.Year);
        assert_eq!(back_to_utc.Month, utc_time.Month);
        assert_eq!(back_to_utc.Day, utc_time.Day);
        assert_eq!(back_to_utc.Hour, utc_time.Hour);
        assert_eq!(back_to_utc.Minute, utc_time.Minute);
        assert!((back_to_utc.Second - utc_time.Second).abs() < 0.001);
    }

    #[test]
    fn test_bds_time_conversion() {
        let utc_time = UtcTime {
            Year: 2020,
            Month: 1,
            Day: 1,
            Hour: 0,
            Minute: 0,
            Second: 0.0,
        };

        let bds_time = utc_to_bds_time(utc_time);
        let back_to_utc = bds_time_to_utc(bds_time);

        // Check that conversion is approximately correct
        assert_eq!(back_to_utc.Year, utc_time.Year);
        assert_eq!(back_to_utc.Month, utc_time.Month);
        assert_eq!(back_to_utc.Day, utc_time.Day);
    }

    #[test]
    fn test_galileo_time_conversion() {
        let utc_time = UtcTime {
            Year: 2020,
            Month: 1,
            Day: 1,
            Hour: 0,
            Minute: 0,
            Second: 0.0,
        };

        let galileo_time = utc_to_galileo_time(utc_time);
        let back_to_utc = galileo_time_to_utc(galileo_time);

        // Check that conversion is approximately correct
        assert_eq!(back_to_utc.Year, utc_time.Year);
        assert_eq!(back_to_utc.Month, utc_time.Month);
        assert_eq!(back_to_utc.Day, utc_time.Day);
    }
}