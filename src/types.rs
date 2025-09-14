//! # Модуль основных типов данных ГНСС
//!
//! Этот модуль содержит базовые структуры данных и типы, используемые во всей ГНСС системе.
//! Включает в себя:
//! - Перечисления для различных ГНСС систем (GPS, ГЛОНАСС, BeiDou, Galileo)
//! - Типы альманахов для разных спутниковых систем
//! - Общие структуры данных для времени, координат и параметров спутников
//! - Константы и базовые типы для работы с ГНСС данными
//!
//! Модуль служит основой для всех других компонентов системы и обеспечивает
//! единообразное представление данных между различными модулями.

#![allow(non_snake_case)]

/// Логический тип для совместимости с C кодом
/// Используется в старом коде для флагов вместо стандартного bool
pub type Bool = i32;
/// Логическая истина (true) как целое число - совместимость с C API
pub const TRUE: Bool = 1;
/// Логическая ложь (false) как целое число - совместимость с C API
pub const FALSE: Bool = 0;

/// Типы альманахов для различных спутниковых навигационных систем
/// Альманах содержит орбитальную информацию всех спутников в упрощенной форме
/// в отличие от эфемерид, которые содержат точные данные для одного спутника
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AlmanacType {
    /// GPS альманах - глобальная система позиционирования (США)
    AlmanacGps,
    /// BeiDou альманах - китайская навигационная система (中国北斗卫星导航系统)
    AlmanacBds,
    /// Galileo альманах - европейская система навигации (ЕС)
    AlmanacGalileo,
    /// ГЛОНАСС альманах - российская система навигации
    AlmanacGlonass,
    /// Неизвестный тип альманаха - используется при ошибках парсинга
    AlmanacUnknown,
}

/// Перечисление поддерживаемых ГНСС систем
/// Каждая система использует свои частоты, форматы сообщений и временные шкалы
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum GnssSystem {
    /// GPS - Global Positioning System (США, L1/L2/L5 частоты, GPS время)
    #[default]
    GpsSystem,
    /// BeiDou - китайская система навигации (B1/B2/B3 частоты, BDT время)
    BdsSystem,
    /// Galileo - европейская система навигации (E1/E5/E6 частоты, GST время)
    GalileoSystem,
    /// ГЛОНАСС - российская система навигации (G1/G2/G3 частоты, GLONASS время)
    GlonassSystem,
    /// SBAS - Satellite-Based Augmentation System (дифференциальные поправки)
    SbasSystem,
    /// QZSS - Quasi-Zenith Satellite System (Япония, региональная система)
    QzssSystem,
    /// NavIC - Navigation with Indian Constellation (IRNSS, Индия)
    NavICSystem,
}

// Signal index constants moved to constants.rs

/// Структура локальной скорости в координатах ENU (East-North-Up)
/// Используется для представления скорости объекта в местной системе координат
#[derive(Debug, Clone, Copy, Default)]
pub struct LocalSpeed {
    /// Скорость на восток (East) в м/с
    pub ve: f64,
    /// Скорость на север (North) в м/с
    pub vn: f64,
    /// Скорость вверх (Up) в м/с
    pub vu: f64,
    /// Полная горизонтальная скорость в м/с (sqrt(ve² + vn²))
    pub speed: f64,
    /// Курс движения в градусах (0° = север, 90° = восток)
    pub course: f64,
}

/// Кинематическая информация объекта в декартовой системе координат ECEF
/// ECEF (Earth-Centered, Earth-Fixed) - геоцентрическая система координат
/// Содержит позицию и скорость спутника или приемника
#[derive(Debug, Clone, Copy, Default)]
pub struct KinematicInfo {
    /// Координата X в системе ECEF (метры) - через экватор и Гринвич
    pub x: f64,
    /// Координата Y в системе ECEF (метры) - через экватор и 90°E
    pub y: f64,
    /// Координата Z в системе ECEF (метры) - через северный полюс
    pub z: f64,
    /// Скорость по оси X в м/с
    pub vx: f64,
    /// Скорость по оси Y в м/с
    pub vy: f64,
    /// Скорость по оси Z в м/с
    pub vz: f64,
}

/// Географические координаты в системе LLA (Latitude-Longitude-Altitude)
/// Используется для представления позиции в привычных географических координатах
#[derive(Debug, Clone, Copy, Default)]
pub struct LlaPosition {
    /// Широта в градусах (-90° до +90°, отрицательные - южное полушарие)
    pub lat: f64,
    /// Долгота в градусах (-180° до +180°, отрицательные - западное полушарие)
    pub lon: f64,
    /// Высота над эллипсоидом WGS84 в метрах
    pub alt: f64,
}

/// Матрица преобразования из декартовых координат ECEF в локальные координаты ENU
/// Используется для конвертации векторов из глобальной системы в локальную систему координат приемника
/// ENU: East-North-Up (Восток-Север-Вверх)
#[derive(Debug, Clone, Copy, Default)]
pub struct ConvertMatrix {
    /// Коэффициент преобразования X → East (восток)
    pub x2e: f64,
    /// Коэффициент преобразования Y → East (восток)
    pub y2e: f64,
    /// Коэффициент преобразования X → North (север)
    pub x2n: f64,
    /// Коэффициент преобразования Y → North (север)
    pub y2n: f64,
    /// Коэффициент преобразования Z → North (север)
    pub z2n: f64,
    /// Коэффициент преобразования X → Up (вверх)
    pub x2u: f64,
    /// Коэффициент преобразования Y → Up (вверх)
    pub y2u: f64,
    /// Коэффициент преобразования Z → Up (вверх)
    pub z2u: f64,
}

/// Временная структура ГНСС в формате недель и миллисекунд
/// Базируется на GPS времени с эпохой 6 января 1980 года 00:00:00 UTC
/// Общий формат времени для GPS, BeiDou и Galileo (с соответствующими поправками)
#[derive(Debug, Clone, Copy, Default)]
pub struct GnssTime {
    /// Номер недели с начала эпохи ГНСС (например, GPS Week Number)
    /// Для GPS: недели с 6 января 1980 года
    /// Для BeiDou: BDT недели (с поправкой +1356 недель от GPS)
    /// Для Galileo: GST недели (синхронизированы с GPS)
    pub Week: i32,
    /// Миллисекунды от начала недели (0-604799999, где 604800000 = 7*24*60*60*1000)
    pub MilliSeconds: i32,
    /// Доли миллисекунд для субмиллисекундной точности
    pub SubMilliSeconds: f64,
}

impl GnssTime {
    /// Добавляет миллисекунды к времени ГНСС с автоматическим переносом недель
    ///
    /// # Аргументы
    /// * `ms` - Количество миллисекунд для добавления (может быть отрицательным)
    ///
    /// # Возвращает
    /// Новое время ГНСС с корректно обработанными переносами недель
    pub fn add_milliseconds(&self, ms: f64) -> GnssTime {
        let mut new_ms = self.MilliSeconds as f64 + ms;
        let mut new_week = self.Week;

        // Обработка переполнения недели (604800000 мс = 1 неделя = 7*24*60*60*1000)
        while new_ms >= 604800000.0 {
            new_ms -= 604800000.0;
            new_week += 1;
        }

        // Обработка отрицательных значений (переход к предыдущей неделе)
        while new_ms < 0.0 {
            new_ms += 604800000.0;
            new_week -= 1;
        }

        GnssTime {
            Week: new_week,
            MilliSeconds: new_ms as i32,
            SubMilliSeconds: self.SubMilliSeconds,
        }
    }
}

/// Координированное всемирное время (UTC) в календарном формате
/// Используется для представления времени в человеко-читаемом виде
/// Конвертируется в ГНСС время с учетом leap seconds (секунды координации)
#[derive(Debug, Clone, Copy, Default)]
pub struct UtcTime {
    /// Год (например, 2025)
    pub Year: i32,
    /// Месяц (1-12)
    pub Month: i32,
    /// День месяца (1-31)
    pub Day: i32,
    /// Час (0-23)
    pub Hour: i32,
    /// Минута (0-59)
    pub Minute: i32,
    /// Секунда с долями (0.0-59.999...)
    pub Second: f64,
}

/// Специфическая временная система ГЛОНАСС
/// Отличается от GPS времени использованием дней в году вместо недель
/// Эпоха ГЛОНАСС: 1 января 1996 года 00:00:00 UTC (отличается от GPS на 16 лет)
#[derive(Debug, Clone, Copy, Default)]
pub struct GlonassTime {
    /// Високосный год (0 - обычный год, 1 - високосный)
    pub LeapYear: i32,
    /// День года (1-365 или 1-366 для високосного года)
    pub Day: i32,
    /// Миллисекунды от начала дня (0-86399999, где 86400000 = 24*60*60*1000)
    pub MilliSeconds: i32,
    /// Доли миллисекунд для точности
    pub SubMilliSeconds: f64,
}

/// Структура GPS эфемерид - точные орбитальные параметры спутника
/// Согласно IS-GPS-200 спецификации. Также используется для Galileo, QZSS, NavIC
/// Отличается от альманахов высокой точностью и ограниченным временем действия (2-4 часа)
#[derive(Debug, Clone, Copy, Default)]
pub struct GpsEphemeris {
    /// User Range Accuracy - оценка точности дальномера (индекс 0-15)
    pub ura: i16,
    /// Issue of Data Clock - счетчик обновления данных часов (10 бит)
    pub iodc: u16,
    /// Issue of Data Ephemeris - счетчик обновления данных эфемерид (8 бит)
    pub iode: u8,
    /// Идентификатор спутника (PRN/SVID: 1-32 для GPS, 1-63 для BeiDou)
    pub svid: u8,
    /// Источник данных: LNAV(0), CNAV(1), FNAV(1), или специальные форматы
    pub source: u8,
    /// Флаг валидности эфемерид (0 = невалидны, 1 = валидны)
    pub valid: u8,
    /// Дополнительные флаги (системно-специфичные)
    pub flag: u16,
    /// Статус здоровья спутника (0 = здоров, >0 = проблемы)
    pub health: u16,
    /// Time of Ephemeris - опорное время эфемерид (секунды недели)
    pub toe: i32,
    /// Time of Clock - опорное время данных часов (секунды недели)
    pub toc: i32,
    /// Time of Prediction - время прогноза (не всегда используется)
    pub top: i32,
    /// GPS/GNSS неделя эфемерид
    pub week: i32,

    // === ОРБИТАЛЬНЫЕ ПАРАМЕТРЫ КЕПЛЕРА ===
    /// Mean Anomaly at Reference Time - средняя аномалия в опорное время (радианы)
    pub M0: f64,
    /// Mean Motion Difference - поправка к вычисленному среднему движению (рад/с)
    pub delta_n: f64,
    /// Rate of Mean Motion Difference - скорость изменения delta_n (рад/с²)
    pub delta_n_dot: f64,
    /// Eccentricity - эксцентриситет орбиты (безразмерная, 0-1)
    pub ecc: f64,
    /// Square Root of Semi-Major Axis - корень из большой полуоси (м¹⁄²)
    pub sqrtA: f64,
    /// Rate of Semi-Major Axis - скорость изменения большой полуоси (м/с)
    pub axis_dot: f64,
    /// Longitude of Ascending Node - долгота восходящего узла в опорное время (рад)
    pub omega0: f64,
    /// Inclination Angle - угол наклонения орбиты в опорное время (рад)
    pub i0: f64,
    /// Argument of Perigee - аргумент перигея (рад)
    pub w: f64,
    /// Rate of Right Ascension - скорость изменения долготы восходящего узла (рад/с)
    pub omega_dot: f64,
    /// Rate of Inclination Angle - скорость изменения наклонения (рад/с)
    pub idot: f64,

    // === ГАРМОНИЧЕСКИЕ КОРРЕКЦИИ ОРБИТЫ ===
    /// Cosine Harmonic Correction Term to Argument of Latitude - косинусная поправка к аргументу широты (рад)
    pub cuc: f64,
    /// Sine Harmonic Correction Term to Argument of Latitude - синусная поправка к аргументу широты (рад)
    pub cus: f64,
    /// Cosine Harmonic Correction Term to Orbital Radius - косинусная поправка к радиусу орбиты (м)
    pub crc: f64,
    /// Sine Harmonic Correction Term to Orbital Radius - синусная поправка к радиусу орбиты (м)
    pub crs: f64,
    /// Cosine Harmonic Correction Term to Inclination - косинусная поправка к наклонению (рад)
    pub cic: f64,
    /// Sine Harmonic Correction Term to Inclination - синусная поправка к наклонению (рад)
    pub cis: f64,

    // === ПАРАМЕТРЫ ЧАСОВ И ЗАДЕРЖЕК ===
    /// SV Clock Bias - смещение часов спутника (секунды)
    pub af0: f64,
    /// SV Clock Drift - скорость дрейфа часов (сек/сек)
    pub af1: f64,
    /// SV Clock Drift Rate - ускорение дрейфа часов (сек/сек²)
    pub af2: f64,
    /// Total Group Delay - общая групповая задержка (секунды)
    pub tgd: f64,
    /// Дополнительная групповая задержка 2 (системно-специфична)
    pub tgd2: f64,
    /// Расширенные параметры групповой задержки для разных частот
    pub tgd_ext: [f64; 5],

    // === ВЫЧИСЛЯЕМЫЕ ПАРАМЕТРЫ (кэшированные) ===
    /// Большая полуось орбиты (метры) = sqrtA²
    pub axis: f64,
    /// Скорректированное среднее движение (рад/с) = √(GM/a³) + delta_n
    pub n: f64,
    /// sqrt(1 - ecc²) - корректирующий множитель эксцентриситета
    pub root_ecc: f64,
    /// Долгота восходящего узла в момент времени t (рад)
    pub omega_t: f64,
    /// Изменение долготы узла с учетом вращения Земли
    pub omega_delta: f64,
    /// Эксцентрическая аномалия (рад) - решение уравнения Кеплера
    pub Ek: f64,
    /// Скорость изменения эксцентрической аномалии (рад/с)
    pub Ek_dot: f64,
}

// BeiDou ephemeris - специальная структура для BeiDou (COMPASS) системы
// Основана на RINEX 3.04 спецификации для BeiDou навигационных сообщений
#[derive(Debug, Clone, Copy, Default)]
pub struct BeiDouEphemeris {
    // управляющие поля
    pub ura: i16,    // User Range Accuracy - точность пользовательского диапазона
    pub iodc: u16,   // Issue of Data Clock - проблема данных часов
    pub iode: u8,    // Issue of Data Ephemeris - проблема данных эфемерид
    pub svid: u8,    // Satellite Vehicle ID - идентификатор спутника (1-63)
    pub source: u8,  // источник данных (D1/D2, B-CNAV1, B-CNAV2, B-CNAV3)
    pub valid: u8,   // флаг валидности
    pub flag: u16,   // дополнительные флаги
    pub health: u16, // статус здоровья спутника
    pub aode: u8,    // Age of Data Ephemeris - возраст данных эфемерид (специфично для BeiDou)
    pub aodc: u8,    // Age of Data Clock - возраст данных часов (специфично для BeiDou)

    // временные параметры
    pub toe: i32,   // Time of Ephemeris (секунды BDT недели)
    pub toc: i32,   // Time of Clock (секунды BDT недели)
    pub top: i32,   // Time of Prediction (не используется в BeiDou)
    pub week: i32,  // BeiDou неделя (отличается от GPS на 1356 недель)
    pub weekh: i32, // Week number в header (старшие биты)

    // орбитальные параметры Кеплера
    pub M0: f64,          // Mean Anomaly at Reference Time (рад)
    pub delta_n: f64,     // Mean Motion Difference from Computed Value (рад/сек)
    pub delta_n_dot: f64, // Rate of Mean Motion Difference (только для B-CNAV2/3)
    pub ecc: f64,         // Eccentricity (безразмерная)
    pub sqrtA: f64,       // Square Root of Semi-Major Axis (м^1/2)
    pub axis_dot: f64,    // Rate of Semi-Major Axis (только для B-CNAV2/3)
    pub omega0: f64,      // Longitude of Ascending Node at Weekly Epoch (рад)
    pub i0: f64,          // Inclination Angle at Reference Time (рад)
    pub w: f64,           // Argument of Perigee (рад)
    pub omega_dot: f64,   // Rate of Right Ascension (рад/сек)
    pub idot: f64,        // Rate of Inclination Angle (рад/сек)

    // гармонические коррекции орбиты
    pub cuc: f64, // Cosine Harmonic Correction Term to Argument of Latitude (рад)
    pub cus: f64, // Sine Harmonic Correction Term to Argument of Latitude (рад)
    pub crc: f64, // Cosine Harmonic Correction Term to Orbital Radius (м)
    pub crs: f64, // Sine Harmonic Correction Term to Orbital Radius (м)
    pub cic: f64, // Cosine Harmonic Correction Term to Inclination (рад)
    pub cis: f64, // Sine Harmonic Correction Term to Inclination (рад)

    // параметры часов и задержки
    pub af0: f64,      // SV Clock Bias Coefficient (сек)
    pub af1: f64,      // SV Clock Drift Coefficient (сек/сек)
    pub af2: f64,      // SV Clock Drift Rate Coefficient (сек/сек^2)
    pub tgd1: f64,     // Equipment Group Delay для B1I сигнала (сек)
    pub tgd2: f64,     // Equipment Group Delay для B2I сигнала (сек)
    pub tgd_b1cp: f64, // Differential Code Bias для B1C/P (только для B-CNAV1)
    pub tgd_b2ap: f64, // Differential Code Bias для B2a/P (только для B-CNAV2)
    pub tgd_b2bp: f64, // Differential Code Bias для B2b/P (только для B-CNAV3)

    // BeiDou специфические параметры
    pub sat_type: u8,       // Тип спутника: GEO(0), IGSO(1), MEO(2)
    pub urai: u8,           // User Range Accuracy Index (0-15)
    pub integrity_flag: u8, // Integrity flag (специфично для BeiDou)

    // производные переменные (вычисляемые)
    pub axis: f64,        // большая полуось (м)
    pub n: f64,           // скорректированное среднее движение (рад/сек)
    pub root_ecc: f64,    // sqrt(1 - ecc^2)
    pub omega_t: f64,     // долгота восходящего узла в момент t
    pub omega_delta: f64, // изменение долготы узла
    pub Ek: f64,          // эксцентрическая аномалия
    pub Ek_dot: f64,      // скорость изменения эксцентрической аномалии
}

// Definitions for source field
pub const EPH_SOURCE_LNAV: u8 = 0;
pub const EPH_SOURCE_D1D2: u8 = 0;
pub const EPH_SOURCE_INAV: u8 = 0;
pub const EPH_SOURCE_CNAV: u8 = 1;
pub const EPH_SOURCE_CNV1: u8 = 1;
pub const EPH_SOURCE_FNAV: u8 = 1;
pub const EPH_SOURCE_CNV2: u8 = 2;
pub const EPH_SOURCE_CNV3: u8 = 3;

// BeiDou specific source field definitions
pub const BDS_SOURCE_D1D2: u8 = 0; // D1/D2 navigation messages (legacy)
pub const BDS_SOURCE_BCNAV1: u8 = 1; // B-CNAV1 для B1C сигнала
pub const BDS_SOURCE_BCNAV2: u8 = 2; // B-CNAV2 для B2a сигнала
pub const BDS_SOURCE_BCNAV3: u8 = 3; // B-CNAV3 для B2b сигнала

impl BeiDouEphemeris {
    /// Конвертирует BeiDouEphemeris в GpsEphemeris для обратной совместимости
    /// Используется в старых частях кода, которые ожидают GpsEphemeris
    pub fn to_gps_ephemeris(&self) -> GpsEphemeris {
        GpsEphemeris {
            ura: self.ura,
            iodc: self.iodc,
            iode: self.iode,
            svid: self.svid,
            source: self.source,
            valid: self.valid,
            flag: self.flag,
            health: self.health,
            toe: self.toe,
            toc: self.toc,
            top: self.top,
            week: self.week,
            M0: self.M0,
            delta_n: self.delta_n,
            delta_n_dot: self.delta_n_dot,
            ecc: self.ecc,
            sqrtA: self.sqrtA,
            axis_dot: self.axis_dot,
            omega0: self.omega0,
            i0: self.i0,
            w: self.w,
            omega_dot: self.omega_dot,
            idot: self.idot,
            cuc: self.cuc,
            cus: self.cus,
            crc: self.crc,
            crs: self.crs,
            cic: self.cic,
            cis: self.cis,
            af0: self.af0,
            af1: self.af1,
            af2: self.af2,
            tgd: self.tgd1,  // BeiDou TGD1 -> GPS TGD
            tgd2: self.tgd2, // BeiDou TGD2 сохраняется
            tgd_ext: [self.tgd_b1cp, self.tgd_b2ap, self.tgd_b2bp, 0.0, 0.0], // Упаковываем дополнительные TGD
            axis: self.axis,
            n: self.n,
            root_ecc: self.root_ecc,
            omega_t: self.omega_t,
            omega_delta: self.omega_delta,
            Ek: self.Ek,
            Ek_dot: self.Ek_dot,
        }
    }

    /// Создаёт BeiDouEphemeris из GpsEphemeris (обратная конвертация)
    pub fn from_gps_ephemeris(gps: &GpsEphemeris) -> Self {
        BeiDouEphemeris {
            ura: gps.ura,
            iodc: gps.iodc,
            iode: gps.iode,
            svid: gps.svid,
            source: gps.source,
            valid: gps.valid,
            flag: gps.flag,
            health: gps.health,
            toe: gps.toe,
            toc: gps.toc,
            top: gps.top,
            week: gps.week,
            M0: gps.M0,
            delta_n: gps.delta_n,
            delta_n_dot: gps.delta_n_dot,
            ecc: gps.ecc,
            sqrtA: gps.sqrtA,
            axis_dot: gps.axis_dot,
            omega0: gps.omega0,
            i0: gps.i0,
            w: gps.w,
            omega_dot: gps.omega_dot,
            idot: gps.idot,
            cuc: gps.cuc,
            cus: gps.cus,
            crc: gps.crc,
            crs: gps.crs,
            cic: gps.cic,
            cis: gps.cis,
            af0: gps.af0,
            af1: gps.af1,
            af2: gps.af2,
            tgd1: gps.tgd,  // GPS TGD -> BeiDou TGD1
            tgd2: gps.tgd2, // TGD2 сохраняется
            tgd_b1cp: if !gps.tgd_ext.is_empty() {
                gps.tgd_ext[0]
            } else {
                0.0
            },
            tgd_b2ap: if gps.tgd_ext.len() > 1 {
                gps.tgd_ext[1]
            } else {
                0.0
            },
            tgd_b2bp: if gps.tgd_ext.len() > 2 {
                gps.tgd_ext[2]
            } else {
                0.0
            },
            axis: gps.axis,
            n: gps.n,
            root_ecc: gps.root_ecc,
            omega_t: gps.omega_t,
            omega_delta: gps.omega_delta,
            Ek: gps.Ek,
            Ek_dot: gps.Ek_dot,
            aode: 0, // Значения по умолчанию для BeiDou-специфичных полей
            aodc: 0,
            sat_type: 0,
            integrity_flag: 0,
            urai: 0,
            weekh: 0,
        }
    }
}

// BeiDou satellite type definitions
pub const BDS_SAT_GEO: u8 = 0; // Geostationary satellites (C01-C05)
pub const BDS_SAT_IGSO: u8 = 1; // Inclined Geosynchronous Orbit satellites (C06-C17)
pub const BDS_SAT_MEO: u8 = 2; // Medium Earth Orbit satellites (C18-C63)

// GPS almanac
#[derive(Debug, Clone, Copy, Default)]
pub struct GpsAlmanac {
    pub valid: u8,
    pub flag: u8,
    pub health: u8,
    pub svid: u8,
    pub toa: i32,
    pub week: i32,
    pub M0: f64,
    pub ecc: f64,
    pub sqrtA: f64,
    pub omega0: f64,
    pub i0: f64,
    pub w: f64,
    pub omega_dot: f64,
    pub af0: f64,
    pub af1: f64,
}

// GLONASS ephemeris
#[derive(Debug, Clone, Copy, Default)]
pub struct GlonassEphemeris {
    pub valid: u8,
    pub flag: u8,
    pub freq: i8,
    pub slot: u8,
    pub P: u8,
    pub M: u8,
    pub Ft: u8,
    pub n: u8,
    pub Bn: u8,
    pub En: u8,
    pub tb: u32,
    pub day: u16,
    pub tk: u16,
    pub gamma: f64,
    pub tn: f64,
    pub dtn: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
    pub ax: f64,
    pub ay: f64,
    pub az: f64,
    // derived variables
    pub tc: f64,
    pub PosVelT: KinematicInfo,
}

// GLONASS almanac
#[derive(Debug, Clone, Copy, Default)]
pub struct GlonassAlmanac {
    pub flag: u8,
    pub freq: i8,
    pub leap_year: i16,
    pub day: i16,
    pub t: f64,
    pub lambda: f64,
    pub di: f64,
    pub ecc: f64,
    pub w: f64,
    pub dt: f64,
    pub dt_dot: f64,
    pub clock_error: f64,
}

// Ionospheric parameters
#[derive(Debug, Clone, Copy, Default)]
pub struct IonoParam {
    pub a0: f64,
    pub a1: f64,
    pub a2: f64,
    pub a3: f64,
    pub b0: f64,
    pub b1: f64,
    pub b2: f64,
    pub b3: f64,
    pub flag: u32,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct IonoNequick {
    pub ai0: f64,
    pub ai1: f64,
    pub ai2: f64,
    pub flag: u32,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct IonoBdgim {
    pub alpha1: f64,
    pub alpha2: f64,
    pub alpha3: f64,
    pub alpha4: f64,
    pub alpha5: f64,
    pub alpha6: f64,
    pub alpha7: f64,
    pub alpha8: f64,
    pub alpha9: f64,
    pub flag: u32,
}

// UTC parameters
#[derive(Debug, Clone, Copy, Default)]
pub struct UtcParam {
    pub A0: f64,
    pub A1: f64,
    pub A2: f64,
    pub WN: i16,
    pub WNLSF: i16,
    pub tot: u8,
    pub TLS: i8,
    pub TLSF: i8,
    pub DN: u8,
    pub flag: u32,
}

pub const MAX_OBS_NUMBER: usize = 6;

#[derive(Debug, Clone, Copy, Default)]
pub struct SatObservation {
    pub system: i32,
    pub svid: i32,
    pub ValidMask: u32,
    pub PseudoRange: [f64; MAX_OBS_NUMBER],
    pub CarrierPhase: [f64; MAX_OBS_NUMBER],
    pub Doppler: [f64; MAX_OBS_NUMBER],
    pub CN0: [f64; MAX_OBS_NUMBER],
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum OutputType {
    #[default]
    OutputTypePosition,
    OutputTypeObservation,
    OutputTypeIFdata,
    OutputTypeBaseband,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum OutputFormat {
    #[default]
    OutputFormatEcef,
    OutputFormatLla,
    OutputFormatNmea,
    OutputFormatKml,
    OutputFormatRinex,
    OutputFormatIQ8,
    OutputFormatIQ4,
}

#[derive(Debug, Clone)]
pub struct OutputParam {
    pub filename: [u8; 256],
    pub config_filename: String, // Путь к файлу конфигурации
    pub Type: OutputType,
    pub Format: OutputFormat,
    pub GpsMaskOut: u32,
    pub GlonassMaskOut: u32,
    pub BdsMaskOut: u64,
    pub GalileoMaskOut: u64,
    pub ElevationMask: f64,
    pub Interval: i32,
    pub SampleFreq: i32,
    pub CenterFreq: i32,
    pub CompactConfig: CompactConfig, // 32-битная конфигурация
}

impl Default for OutputParam {
    fn default() -> Self {
        Self {
            filename: [0u8; 256],
            config_filename: String::new(),
            Type: OutputType::default(),
            Format: OutputFormat::default(),
            GpsMaskOut: 0,
            GlonassMaskOut: 0,
            BdsMaskOut: 0,
            GalileoMaskOut: 0,
            ElevationMask: 0.0,
            Interval: 0,
            SampleFreq: 0,
            CenterFreq: 0,
            CompactConfig: CompactConfig::default(),
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct DelayConfig {
    pub SystemDelay: [f64; 4],
    pub ReceiverDelay: [[f64; 8]; 4],
}

#[derive(Debug, Clone, Copy, Default)]
pub struct SatelliteParam {
    pub system: GnssSystem,
    pub svid: i32,
    pub FreqID: i32,
    pub CN0: i32,
    pub PosTimeTag: i32,
    pub PosVel: KinematicInfo,
    pub Acc: [f64; 3],
    pub TravelTime: f64,
    pub IonoDelay: f64,
    pub GroupDelay: [f64; 8],
    pub Elevation: f64,
    pub Azimuth: f64,
    pub RelativeSpeed: f64,
    pub LosVector: [f64; 3],
}

// Compact configuration structure - 32-bit approach
#[derive(Debug, Clone, Copy, Default)]
pub struct CompactConfig {
    pub config: u32,
}

// System parsing bits (0-3)
pub const PARSE_GPS: u32 = 1 << 0; // 0x1
pub const PARSE_BDS: u32 = 1 << 1; // 0x2
pub const PARSE_GALILEO: u32 = 1 << 2; // 0x4
pub const PARSE_GLONASS: u32 = 1 << 3; // 0x8

// Signal generation bits (4-31) - mapped from SIGNAL_INDEX + 4
pub const GEN_L1CA: u32 = 1 << 4; // GPS L1CA  (SIGNAL_INDEX_L1CA + 4)
pub const GEN_L1C: u32 = 1 << 5; // GPS L1C   (SIGNAL_INDEX_L1C + 4)
pub const GEN_L2C: u32 = 1 << 6; // GPS L2C   (SIGNAL_INDEX_L2C + 4)
pub const GEN_L2P: u32 = 1 << 7; // GPS L2P   (SIGNAL_INDEX_L2P + 4)
pub const GEN_L5: u32 = 1 << 8; // GPS L5    (SIGNAL_INDEX_L5 + 4)
pub const GEN_B1C: u32 = 1 << 12; // BDS B1C   (SIGNAL_INDEX_B1C + 4)
pub const GEN_B1I: u32 = 1 << 13; // BDS B1I   (SIGNAL_INDEX_B1I + 4)
pub const GEN_B2I: u32 = 1 << 14; // BDS B2I   (SIGNAL_INDEX_B2I + 4)
pub const GEN_B3I: u32 = 1 << 15; // BDS B3I   (SIGNAL_INDEX_B3I + 4)
pub const GEN_B2A: u32 = 1 << 16; // BDS B2A   (SIGNAL_INDEX_B2A + 4)
pub const GEN_B2B: u32 = 1 << 17; // BDS B2B   (SIGNAL_INDEX_B2B + 4)
pub const GEN_B2AB: u32 = 1 << 18; // BDS B2AB  (SIGNAL_INDEX_B2AB + 4)
pub const GEN_E1: u32 = 1 << 20; // GAL E1    (SIGNAL_INDEX_E1 + 4)
pub const GEN_E5A: u32 = 1 << 21; // GAL E5A   (SIGNAL_INDEX_E5A + 4)
pub const GEN_E5B: u32 = 1 << 22; // GAL E5B   (SIGNAL_INDEX_E5B + 4)
pub const GEN_E6: u32 = 1 << 24; // GAL E6    (SIGNAL_INDEX_E6 + 4)
pub const GEN_G1: u32 = 1 << 28; // GLO G1    (SIGNAL_INDEX_G1 + 4)
pub const GEN_G2: u32 = 1 << 29; // GLO G2    (SIGNAL_INDEX_G2 + 4)
pub const GEN_G3: u32 = 1 << 30; // GLO G3    (SIGNAL_INDEX_G3 + 4)

impl CompactConfig {
    pub fn new() -> Self {
        Self { config: 0 }
    }

    // Check if system parsing is enabled
    pub fn should_parse_gps(&self) -> bool {
        self.config & PARSE_GPS != 0
    }

    pub fn should_parse_bds(&self) -> bool {
        self.config & PARSE_BDS != 0
    }

    pub fn should_parse_galileo(&self) -> bool {
        self.config & PARSE_GALILEO != 0
    }

    pub fn should_parse_glonass(&self) -> bool {
        self.config & PARSE_GLONASS != 0
    }

    // Check if signal generation is enabled
    pub fn is_signal_enabled(&self, signal_bit: u32) -> bool {
        self.config & signal_bit != 0
    }

    // Enable system parsing
    pub fn enable_system_parsing(&mut self, system_bit: u32) {
        self.config |= system_bit;
    }

    // Enable signal generation
    pub fn enable_signal(&mut self, signal_bit: u32) {
        self.config |= signal_bit;
    }

    // Convert SIGNAL_INDEX to generation bit
    pub fn signal_index_to_gen_bit(signal_index: usize) -> Option<u32> {
        match signal_index {
            0 => Some(GEN_L1CA),  // SIGNAL_INDEX_L1CA
            1 => Some(GEN_L1C),   // SIGNAL_INDEX_L1C
            2 => Some(GEN_L2C),   // SIGNAL_INDEX_L2C
            3 => Some(GEN_L2P),   // SIGNAL_INDEX_L2P
            4 => Some(GEN_L5),    // SIGNAL_INDEX_L5
            8 => Some(GEN_B1C),   // SIGNAL_INDEX_B1C
            9 => Some(GEN_B1I),   // SIGNAL_INDEX_B1I
            10 => Some(GEN_B2I),  // SIGNAL_INDEX_B2I
            11 => Some(GEN_B3I),  // SIGNAL_INDEX_B3I
            12 => Some(GEN_B2A),  // SIGNAL_INDEX_B2A
            13 => Some(GEN_B2B),  // SIGNAL_INDEX_B2B
            14 => Some(GEN_B2AB), // SIGNAL_INDEX_B2AB
            16 => Some(GEN_E1),   // SIGNAL_INDEX_E1
            17 => Some(GEN_E5A),  // SIGNAL_INDEX_E5A
            18 => Some(GEN_E5B),  // SIGNAL_INDEX_E5B
            20 => Some(GEN_E6),   // SIGNAL_INDEX_E6
            24 => Some(GEN_G1),   // SIGNAL_INDEX_G1
            25 => Some(GEN_G2),   // SIGNAL_INDEX_G2
            26 => Some(GEN_G3),   // SIGNAL_INDEX_G3
            _ => None,
        }
    }
}

// Type aliases for backwards compatibility
pub type Ephemeris = GpsEphemeris;

impl KinematicInfo {
    pub fn pos_vel(&self) -> [f64; 6] {
        [self.x, self.y, self.z, self.vx, self.vy, self.vz]
    }
}

impl GpsEphemeris {
    pub fn new() -> Self {
        Self::default()
    }
}

impl GlonassEphemeris {
    pub fn new() -> Self {
        Self::default()
    }
}
