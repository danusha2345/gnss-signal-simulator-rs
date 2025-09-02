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

    /// КРИТИЧЕСКОЕ ИСПРАВЛЕНИЕ: Преобразование UTC → GPS времени
    /// 
    /// Функция решает историческую ошибку с 4-дневным сдвигом времени, которая 
    /// приводила к тому, что все спутники оказывались под горизонтом.
    /// 
    /// **GPS временная эпоха:** 6 января 1980 г., 00:00:00 UTC (начало GPS недели 0)
    /// **Високосные секунды:** Учитываются для точного преобразования (опциональны)
    /// **Недельный цикл:** 1024 недели (19.7 лет), затем сброс счетчика
    /// 
    /// **Математическая модель:**
    /// 1. Подсчет полных дней от GPS эпохи до целевой даты
    /// 2. Учет високосных лет по григорианскому календарю
    /// 3. Коррекция на високосные секунды (если включена)
    /// 4. Разбиение на недели (604800 сек) и секунды недели
    /// 
    /// **Устраненные проблемы:**
    /// - Неправильное использование ГЛОНАСС эпохи (1996) вместо GPS (1980)
    /// - Некорректная обработка перехода через границы недели
    /// - Ошибки в високосных секундах при граничных условиях
    pub fn utc_to_gps_time(utc_time: UtcTime, use_leap_second: bool) -> GnssTime {
        
        // ЭТАП 1: Точный подсчет дней от GPS эпохи (6 января 1980)
        // 
        // Алгоритм поэтапного накопления дней с учетом високосных лет:
        // - Високосный год: делится на 4, но НЕ на 100, КРОМЕ кратных 400
        // - Примеры: 1980✓, 1984✓, 1900✗, 2000✓, 2004✓
        let mut days_since_gps_epoch = 0i32;
        
        // Накопление полных календарных лет от 1980 до (год-1)
        // Каждый год добавляет 365 или 366 дней в зависимости от високосности
        for year in 1980..utc_time.Year {
            if Self::is_leap_year(year) {
                days_since_gps_epoch += 366; // Високосный год: +1 день (29 февраля)
            } else {
                days_since_gps_epoch += 365; // Обычный год
            }
        }
        
        // ЭТАП 2: Добавление дней текущего года
        // 
        // Подсчет дней от 1 января до указанной даты включительно
        // с учетом высокосности текущего года для февраля
        let days_in_target_year = Self::day_of_year(utc_time.Year, utc_time.Month, utc_time.Day);
        days_since_gps_epoch += days_in_target_year;
        
        // ЭТАП 3: Коррекция на GPS эпоху
        // 
        // GPS эпоха начинается 6 января 1980, НЕ 1 января!
        // Необходимо вычесть 6 дней (1-6 января 1980 не входят в GPS время)
        // Это критически важная коррекция!
        days_since_gps_epoch -= 6; // Коррекция эпохи: 6 января, а не 1 января
        
        // ЭТАП 4: Преобразование в общее количество секунд от GPS эпохи
        // 
        // Формула: total_seconds = days × 86400 + seconds_today
        // где 86400 = 24×60×60 секунд в сутках
        let total_seconds_in_day = utc_time.Hour * 3600 + utc_time.Minute * 60 + utc_time.Second as i32;
        let mut total_seconds = (days_since_gps_epoch * 86400 + total_seconds_in_day) as u32;
        let temp_seconds = total_seconds; // Сохраняем для високосных секунд
        
        // Детектор границы суток (для правильной обработки високосных секунд)
        let next_day = utc_time.Hour == 0 && utc_time.Minute == 0 && (utc_time.Second as i32) == 0;

        // ЭТАП 5: Коррекция на високосные секунды (если включена)
        // 
        // Високосные секунды добавляются ITU для синхронизации с астрономическим временем.
        // С 1980 года добавлено 18 високосных секунд (по состоянию на 2023 год).
        // 
        // **Алгоритм двойной коррекции:**
        // 1. Получаем количество високосных секунд для исходного времени
        // 2. Корректируем время и проверяем заново (итеративная коррекция)
        // 3. Обрабатываем особый случай: момент вставки високосной секунды
        if use_leap_second {
            let (mut leap_second, _) = Self::get_leap_second(temp_seconds);
            let temp_seconds_adj = temp_seconds + leap_second as u32;
            let (leap_second_adj, at_leap_second) = Self::get_leap_second(temp_seconds_adj);
            leap_second = leap_second_adj;
            
            // Специальная обработка момента вставки високосной секунды:
            // В 23:59:60 UTC добавляется +1 секунда к обычной коррекции
            total_seconds += if at_leap_second && next_day { 
                (leap_second + 1) as u32  // +1 за саму високосную секунду
            } else { 
                leap_second as u32        // Стандартная коррекция
            };
        }

        // ЭТАП 6: Разбиение на GPS недели и секунды недели
        // 
        // **GPS неделя:** Период в 604800 секунд (7 × 24 × 60 × 60)
        // **Недельный цикл:** 1024 недели, затем сброс (каждые ~19.7 лет)
        // **Первый rollover:** 22 августа 1999 (неделя 0 → неделя 1024)
        // **Второй rollover:** 7 апреля 2019 (неделя 1024 → неделя 2048)
        let week = (total_seconds / 604800) as i32;  // Номер GPS недели
        let seconds_in_week = total_seconds - (week as u32) * 604800; // Секунды с начала недели
        
        // Преобразование в миллисекунды с сохранением дробной части
        // MilliSeconds содержит полные миллисекунды недели
        // SubMilliSeconds содержит дробную часть секунды (< 1.0)
        let milli_seconds = (seconds_in_week * 1000 + ((utc_time.Second % 1.0) * 1000.0) as u32) as i32;

        // Финальная GPS временная структура
        GnssTime {
            Week: week,                              // Номер недели от GPS эпохи
            MilliSeconds: milli_seconds,             // Миллисекунды с начала недели
            SubMilliSeconds: utc_time.Second % 1.0,  // Дробная часть секунды [0.0, 1.0)
        }
    }
    
    // Вспомогательные функции
    fn is_leap_year(year: i32) -> bool {
        (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)
    }
    
    fn day_of_year(year: i32, month: i32, day: i32) -> i32 {
        let mut days = day;
        
        // Добавляем дни от предыдущих месяцев
        for m in 1..month {
            days += Self::days_in_month(year, m);
        }
        
        days
    }
    
    fn days_in_month(year: i32, month: i32) -> i32 {
        match month {
            1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
            4 | 6 | 9 | 11 => 30,
            2 => if Self::is_leap_year(year) { 29 } else { 28 },
            _ => 0,
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

    /// Преобразование UTC → BeiDou Time (BDT) с коррекцией временной зоны
    /// 
    /// **BeiDou эпоха:** 1 января 2006 г., 00:00:00 UTC
    /// **Оффсет GPS:** 1356 недель + 14 секунд (leap seconds коррекция)
    /// **Особенность:** Учитывает 14-секундную коррекцию високосных секунд
    /// 
    /// **Алгоритм с граничными условиями:**
    /// 1. Если MilliSeconds ≥ 14000: простое вычитание 14 секунд и 1356 недель
    /// 2. Если MilliSeconds < 14000: перенос через границу недели
    /// 
    /// **Математическая модель:**
    /// - 604800000 мс = 7 × 24 × 60 × 60 × 1000 (неделя в мс)
    /// - 14000 мс = 14 секунд (accumulated leap seconds к 2006 году)
    /// - 1356/1357 недель: разница в 1 неделю при переносе
    pub fn utc_to_bds_time(utc_time: UtcTime) -> GnssTime {
        let mut time = Self::utc_to_gps_time(utc_time, true);

        // Обработка граничного случая: перенос через начало недели
        if time.MilliSeconds >= 14000 {
            // ОБЫЧНЫЙ СЛУЧАЙ: достаточно времени в текущей неделе
            time.MilliSeconds -= 14000;  // -14 секунд (leap seconds коррекция)
            time.Week -= 1356;           // -1356 недель (коррекция эпохи)
        } else {
            // КРИТИЧЕСКИЙ СЛУЧАЙ: недостаточно времени, нужно "заимствовать" из предыдущей недели
            time.MilliSeconds += 604800000 - 14000; // +1 неделя минус 14 секунд
            time.Week -= 1357;                       // -1357 недель (дополнительная -1 за "заим")
        }

        time
    }

    /// Преобразование UTC → Galileo System Time (GST)
    /// 
    /// **Galileo эпоха:** 22 августа 1999 г., 00:00:00 UTC (GST Week 0)
    /// **Оффсет GPS:** 1024 недели (19.6 лет после GPS эпохи)
    /// **Особенность:** Дата совпадает с первым GPS rollover
    /// 
    /// **Исторический контекст:**
    /// - 22 августа 1999 - первый GPS rollover (Week 0 → Week 1024)
    /// - Это не случайно! Galileo начался с чистого листа
    /// - Предотвращение проблем rollover на ядре Galileo
    /// 
    /// **Математическая модель:**
    /// GST_week = GPS_week - 1024
    /// где 1024 = (1999-08-22 - 1980-01-06) / 7 дней
    /// 
    /// **Примечание:** В реальности GST опережает GPS на 19 секунд,
    /// но данная имплементация игнорирует этот оффсет для упрощения.
    pub fn utc_to_galileo_time(utc_time: UtcTime) -> GnssTime {
        let mut time = Self::utc_to_gps_time(utc_time, true);
        // Простое вычитание 1024 недель (коррекция эпохи)
        // НЕ учитываем 19-секундный оффсет GST vs GPS
        time.Week -= 1024; // Galileo GST Week 0 = GPS Week 1024
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