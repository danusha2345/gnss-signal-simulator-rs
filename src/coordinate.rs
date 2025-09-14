//----------------------------------------------------------------------
// coordinate.rs:
//   Implementation of coordinate related functions
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------
//! # Преобразования координат
//!
//! Модуль для преобразований между различными системами координат,
//! используемыми в ГНСС навигации и обработке спутниковых данных.
//!
//! ## Основные функции:
//! - Преобразование WGS84 геодезические ↔ ECEF декартовы координаты
//! - Преобразование ECEF ↔ ENU (East-North-Up) локальные координаты
//! - Расчет матриц поворота между системами координат
//! - Преобразование скоростей между системами координат
//! - Работа с различными геодезическими датумами
//!
//! ## Поддерживаемые системы координат:
//! - WGS84 (основная система GPS)
//! - ПЗ-90 (система ГЛОНАСС)
//! - CGCS2000 (система BeiDou)
//! - GTRF (система Galileo)
//! - Локальные топоцентрические системы (ENU, NED)

use crate::constants::*;
use crate::types::*;

use std::f64::consts::PI;

const COARSE_STEP: f64 = 30.0;
const COS_5: f64 = 0.996_194_698_091_745_5;
const SIN_5: f64 = 0.087_155_742_747_658_18;
const REL_HUMI: f64 = 0.7;

/// GPS clock correction calculation
pub fn gps_clock_correction(eph: &GpsEphemeris, transmit_time: f64) -> f64 {
    let mut time_diff = transmit_time - eph.toc as f64;

    // protection for time ring back at week end
    if time_diff > 302400.0 {
        time_diff -= 604800.0;
    }
    if time_diff < -302400.0 {
        time_diff += 604800.0;
    }

    let mut clock_adj = eph.af0 + (eph.af1 + eph.af2 * time_diff) * time_diff;
    clock_adj *= 1.0 - eph.af1; // adjustment to time

    clock_adj
}

/// GLONASS clock correction calculation
pub fn glonass_clock_correction(eph: &GlonassEphemeris, transmit_time: f64) -> f64 {
    let mut time_diff = transmit_time - eph.tb as f64;

    if time_diff > 43200.0 {
        time_diff -= 86400.0;
    } else if time_diff < -43200.0 {
        time_diff += 86400.0;
    }

    -eph.tn + eph.gamma * time_diff
}

/// Расчет положения, скорости и ускорения GPS спутника по эфемеридам
///
/// Основан на классическом алгоритме Kepler-Lagrange для решения уравнения Кеплера:
///
/// **Основные этапы:**
/// 1. Решение уравнения Кеплера: E = M + e*sin(E) (метод Newton-Raphson)
/// 2. Вычисление истинной аномалии ν = atan2(√(1-e²)·sin(E), cos(E)-e)
/// 3. Применение пертурбационных поправок (Cuc, Cus, Crc, Crs, Cic, Cis)
/// 4. Преобразование из орбитальной плоскости в ECEF координаты
///
/// **Математические константы:**
/// - Полупериод GPS недели: 302400 сек (3.5 дня) - для оболочки перехода недель
/// - Точность сходимости: 1e-14 радиан (ок. 0.0001 массекунд дуги)
/// - Максимальное количество итераций: 10 (для предотвращения бесконечных циклов)
///
/// **Критическая особенность:** Обработка перехода GPS недели (604800 сек цикл)
/// Ошибка может привести к неправильному расчету положения на сотни километров!
pub fn gps_sat_pos_speed_eph(
    system: GnssSystem,
    transmit_time: f64,
    eph: &mut GpsEphemeris,
    pos_vel: &mut KinematicInfo,
    mut acc: Option<&mut [f64; 3]>,
) -> bool {
    // Отладка входных параметров отключена для уменьшения вывода
    // if eph.svid == 1 {  // Только для первого спутника для уменьшения вывода
    //     println!("[GPS_POS_DEBUG] Input: transmit_time={}, toe={}, svid={}",
    //              transmit_time, eph.toe, eph.svid);
    // }

    // calculate time difference
    // КАК В C ВЕРСИИ: transmit_time уже в секундах недели, не применяем модульное деление
    let mut delta_t = transmit_time - (eph.toe as f64);

    // КРИТИЧЕСКИЙ ФИКС: правильная обработка перехода недели
    // Если эфемериды из прошлой недели (большая положительная разница)
    if delta_t > 302400.0 {
        delta_t -= 604800.0;
    }
    // Если эфемериды из следующей недели (большая отрицательная разница)
    if delta_t < -302400.0 {
        delta_t += 604800.0;
    }

    // РЕШЕНИЕ УРАВНЕНИЯ КЕПЛЕРА: E = M + e·sin(E)
    //
    // Метод Newton-Raphson для решения трансцендентного уравнения:
    // f(E) = E - e·sin(E) - M = 0
    // f'(E) = 1 - e·cos(E)
    // E_{n+1} = E_n - f(E_n)/f'(E_n)
    //
    // Параметры:
    // - alpha: коррекция среднего движения (δn̈·Δt)
    // - mk: средняя аномалия M = M₀ + (n + α/2)·Δt
    // - ek: эксцентрическая аномалия E (искомая величина)
    let alpha = eph.delta_n_dot * delta_t;
    let mk = eph.M0 + (eph.n + alpha / 2.0) * delta_t;
    let mut ek = mk; // Начальная оценка: E₀ = M
    let mut ek1 = ek;

    // Итерационный процесс Newton-Raphson (10 итераций максимум)
    // Типичная сходимость: 3-5 итераций для достижения 1e-14 точности
    for _ in 0..10 {
        ek = mk + eph.ecc * ek.sin(); // Упрощенная форма: E = M + e·sin(E)
        if (ek - ek1).abs() < 1e-14 {
            // Критерий сходимости
            break;
        }
        ek1 = ek;
    }
    eph.Ek = ek;

    // assign ek1 as 1-e*cos(Ek)
    ek1 = 1.0 - (eph.ecc * ek.cos());

    // ВЫЧИСЛЕНИЕ ИСТИННОЙ АНОМАЛИИ ν (ню)
    //
    // Формула: ν = atan2(√(1-e²)·sin(E), cos(E) - e) + ω
    // Где:
    // - E: эксцентрическая аномалия
    // - e: эксцентриситет орбиты (0 < e < 1)
    // - ω: аргумент перигея
    // - atan2: 4-квадрантная арктангенс функция
    //
    // phi(k) = аргумент широты = истинная аномалия + аргумент перигея
    let phi = (eph.root_ecc * ek.sin()).atan2(ek.cos() - eph.ecc) + eph.w;
    let sin_temp = (phi + phi).sin();
    let cos_temp = (phi + phi).cos();

    // get u(k), r(k) and i(k)
    let mut uk = phi;
    let mut rk = (eph.axis + eph.axis_dot * delta_t) * ek1;
    let mut ik = eph.i0 + (eph.idot * delta_t);

    // ПРИМЕНЕНИЕ ПЕРТУРБАЦИОННЫХ ПОПРАВОК 2-ГО ПОРЯДКА
    //
    // Коррекции для учета несферичности Земли и возмущений:
    //
    // **Аргумент широты u(k):**
    // δu = Cuc·cos(2φ) + Cus·sin(2φ)
    //
    // **Радиальное расстояние r(k):**
    // δr = Crc·cos(2φ) + Crs·sin(2φ)
    //
    // **Наклонение орбиты i(k):**
    // δi = Cic·cos(2φ) + Cis·sin(2φ)
    //
    // Где: 2φ = 2·(ν + ω) - удвоенный аргумент широты
    let duk = (eph.cuc * cos_temp) + (eph.cus * sin_temp); // Коррекция аргумента широты
    let drk = (eph.crc * cos_temp) + (eph.crs * sin_temp); // Коррекция радиального расстояния
    let dik = (eph.cic * cos_temp) + (eph.cis * sin_temp); // Коррекция наклонения орбиты
    uk += duk;
    rk += drk;
    ik += dik;

    // ========== ВЫЧИСЛЕНИЕ ПРОИЗВОДНЫХ (СКОРОСТИ) ОРБИТАЛЬНЫХ ПАРАМЕТРОВ ==========
    //
    // НЕОБХОДИМОСТЬ: Для расчета доплеровского сдвига частоты и компенсации
    // эффекта Саньяк-Ферма необходима временная производная всех орбитальных параметров.
    //
    // ОСНОВНЫЕ ФОРМУЛЫ ДЛЯ ПРОИЗВОДНЫХ:
    //
    // 1. Производная эксцентрической аномалии:
    //    Ė̇ = (n + α) / (1 - e·cos(E))
    //    где: n - среднее движение, α - корр. ср. движения
    //
    // 2. Производная аргумента широты:
    //    u̇ = Ė̇ · √(1-e²) / (1 - e·cos(E))
    //
    // 3. Производная радиального расстояния:
    //    ṙ = a·e·sin(E)·Ė̇ + ȧ·(1 - e·cos(E))
    //    где: ȧ - скорость изменения большой полуоси

    // Скорость изменения эксцентрической аномалии (рад/сек)
    eph.Ek_dot = (eph.n + alpha) / ek1;

    // Скорость изменения аргумента широты (рад/сек)
    let mut uk_dot = eph.Ek_dot * eph.root_ecc / ek1;

    // Удвоенная скорость аргумента широты (для пертурб. коррекций)
    let phi_dot = uk_dot * 2.0;

    // Скорость изменения радиального расстояния (м/сек)
    // Первый слагаемый: от эксцентриситета (a·e·sin(E)·Ė̇)
    // Второй слагаемый: от изменения большой полуоси (ȧ)
    let mut rk_dot = eph.axis * eph.ecc * ek.sin() * eph.Ek_dot + eph.axis_dot * ek1;

    // СКОРОСТИ ПЕРТУРБАЦИОННЫХ КОРРЕКЦИЙ 2-ГО ПОРЯДКА:
    //
    // Производные гармонических коррекций по формулам:
    // d/dt[C·cos(2φ) + S·sin(2φ)] = 2φ̇·[S·cos(2φ) - C·sin(2φ)]

    // Скорость изм. радиальн. коррекции: δṙ = Crs·cos(2φ) - Crc·sin(2φ)
    let drk_dot = ((eph.crs * cos_temp) - (eph.crc * sin_temp)) * phi_dot;

    // Скорость изм. коррекции аргумента широты: δu̇ = Cus·cos(2φ) - Cuc·sin(2φ)
    let duk_dot = ((eph.cus * cos_temp) - (eph.cuc * sin_temp)) * phi_dot;

    // Скорость изм. коррекции наклонения: δi̇ = Cis·cos(2φ) - Cic·sin(2φ)
    let dik_dot = ((eph.cis * cos_temp) - (eph.cic * sin_temp)) * phi_dot;
    rk_dot += drk_dot;
    uk_dot += duk_dot;
    let ik_dot = eph.idot + dik_dot;

    // ========== ВЫЧИСЛЕНИЕ УСКОРЕНИЙ (ВТОРЫЕ ПРОИЗВОДНЫЕ) ==========
    //
    // НАЗНАЧЕНИЕ: Вычисление ускорений спутника для:
    // - Компенсации релятивистских эффектов
    // - Учета сил тяжести и солнечного давления
    // - Повышенной точности позиционирования
    //
    // МАТЕМАТИЧЕСКИЕ ОСНОВЫ:
    // Ускорение - вторая производная положения по времени:
    // ¨r = d²r/dt², u¨ = d²u/dt², i¨ = d²i/dt²
    let (rk_dot2, uk_dot2, ik_dot2) = if acc.is_some() {
        // Вторая производная эксцентрической аномалии:
        // ¨E = -(Ė̇² · e · sin(E)) / (1 - e·cos(E))
        let ek_dot2 = -eph.Ek_dot * eph.Ek_dot * eph.ecc * ek.sin() / ek1;

        // Вторая производная удвоенного аргумента широты:
        // ¨2φ = 2 · ¨E · √(1-e²) / (1 - e·cos(E))
        let phi_dot2 = 2.0 * ek_dot2 * eph.root_ecc / ek1;

        // Коэффициенты для сокращения вычислений:
        let alpha_acc = 2.0 * phi_dot2 / phi_dot; // Отношение второй к первой производной
        let beta = phi_dot * phi_dot; // Квадрат первой производной (4φ̇²)

        // Ускорение радиального расстояния:
        // ¨r = a·e·[¨E·sin(E) + Ė̇²·cos(E)] + α·δṙ̇ - β·δr
        let rk_dot2 =
            eph.axis * eph.ecc * (ek.sin() * ek_dot2 + ek.cos() * eph.Ek_dot * eph.Ek_dot)
                + alpha_acc * drk_dot
                - beta * drk;

        // Ускорение аргумента широты:
        // u¨ = ¨2φ + α·δu̇ - β·δu
        let uk_dot2 = phi_dot2 + alpha_acc * duk_dot - beta * duk;

        // Ускорение наклонения орбиты:
        // i¨ = α·δi̇ - β·δi
        let ik_dot2 = alpha_acc * dik_dot - beta * dik;

        (rk_dot2, uk_dot2, ik_dot2)
    } else {
        // Если ускорения не требуются, возвращаем нули
        (0.0, 0.0, 0.0)
    };

    // ========== ПЕРЕХОД В ОРБИТАЛЬНУЮ ПЛОСКОСТЬ ==========
    //
    // Преобразование полярных орбитальных координат (r, u) в декартовы координаты
    // орбитальной плоскости (Xp, Yp).
    //
    // ПОЛЯРНЫЕ → ДЕКАРТОВЫ КООРДИНАТЫ:
    // Xp = r · cos(u) - положение по оси X в орбитальной плоскости
    // Yp = r · sin(u) - положение по оси Y в орбитальной плоскости
    //
    // ПРОИЗВОДНЫЕ (скорости):
    // Xṗp = ṙ̇ · cos(u) - r · sin(u) · u̇ = ṙ̇ · cos(u) - Yp · u̇
    // Yṗp = ṙ̇ · sin(u) + r · cos(u) · u̇ = ṙ̇ · sin(u) + Xp · u̇

    let sin_temp = uk.sin(); // sin(аргумент широты)
    let cos_temp = uk.cos(); // cos(аргумент широты)

    // Положение в орбитальной плоскости (метры)
    let xp = rk * cos_temp; // Xp - координата по направлению к перигею
    let yp = rk * sin_temp; // Yp - координата перпендикулярно Xp

    // Скорость в орбитальной плоскости (м/сек)
    let xp_dot = rk_dot * cos_temp - yp * uk_dot; // dXp/dt
    let yp_dot = rk_dot * sin_temp + xp * uk_dot; // dYp/dt

    // calculate intermediate variables for acceleration
    let (xp_dot2, yp_dot2) = if acc.is_some() {
        let xp_dot2 = rk_dot2 * uk.cos()
            - 2.0 * uk_dot * rk_dot * uk.sin()
            - uk_dot * uk_dot * xp
            - uk_dot2 * yp;
        let yp_dot2 = rk_dot2 * uk.sin() + 2.0 * uk_dot * rk_dot * uk.cos() - uk_dot * uk_dot * yp
            + uk_dot2 * xp;
        (xp_dot2, yp_dot2)
    } else {
        (0.0, 0.0)
    };

    // ПРЕОБРАЗОВАНИЕ В ECEF КООРДИНАТЫ
    //
    // Переход из орбитальной плоскости в систему ECEF (Earth-Centered, Earth-Fixed):
    //
    // **Долгота восходящего узла:**
    // Ω(t) = Ω₀ + (Ω̇ - ωₑ)·Δt
    //
    // КРИТИЧНО: Используем упрощенную формулу без учета вращения Земли ωₑ
    // в позиции (только в скорости). Это соответствует C версии.
    let omega = eph.omega_t + eph.omega_delta * delta_t;
    // Матрица поворота из орбитальной плоскости в ECEF:
    // ┌   ┐   ┌                                      ┐  ┌  ┐
    // │ X │   │  cos Ω  -sin Ω cos i   sin Ω sin i │ │xₚ│
    // │ Y │ = │  sin Ω   cos Ω cos i  -cos Ω sin i │ │yₚ│
    // │ Z │   │    0         sin i        cos i    │ │0│
    // └   ┘   └                                      ┘ └  ┘
    let sin_temp = omega.sin(); // sin(Ω)
    let cos_temp = omega.cos(); // cos(Ω)
    let phi_sin = ik.sin(); // sin(i) - синус наклонения орбиты
    pos_vel.z = yp * phi_sin; // Z = yₚ·sin(i)
    pos_vel.vz = yp_dot * phi_sin; // Vz = ẏₚ·sin(i)

    let phi_cos = ik.cos(); // cos(i) - косинус наклонения орбиты
    pos_vel.x = xp * cos_temp - yp * phi_cos * sin_temp; // X = xₚ·cos(Ω) - yₚ·cos(i)·sin(Ω)
    pos_vel.y = xp * sin_temp + yp * phi_cos * cos_temp; // Y = xₚ·sin(Ω) + yₚ·cos(i)·cos(Ω)

    // phi_dot assign as yp_dot * cos(ik) - z * ik_dot
    let phi_dot_final = yp_dot * phi_cos - pos_vel.z * ik_dot;
    pos_vel.vx = xp_dot * cos_temp - phi_dot_final * sin_temp;
    pos_vel.vy = xp_dot * sin_temp + phi_dot_final * cos_temp;
    pos_vel.vx -= pos_vel.y * eph.omega_delta;
    pos_vel.vy += pos_vel.x * eph.omega_delta;
    pos_vel.vz += yp * ik_dot * phi_cos;

    // calculate acceleration if given valid array pointer
    if let Some(acc_array) = acc.as_mut() {
        let alpha_final = pos_vel.vz * ik_dot + pos_vel.z * ik_dot2 - xp_dot * eph.omega_delta
            + yp_dot * ik_dot * phi_sin
            - yp_dot2 * phi_cos;
        let beta_final =
            xp_dot2 + pos_vel.z * ik_dot * eph.omega_delta - yp_dot * eph.omega_delta * phi_cos;
        acc_array[0] =
            -pos_vel.vy * eph.omega_delta + alpha_final * sin_temp + beta_final * cos_temp;
        acc_array[1] =
            pos_vel.vx * eph.omega_delta - alpha_final * cos_temp + beta_final * sin_temp;
        acc_array[2] = (yp_dot2 - yp * ik_dot * ik_dot) * phi_sin
            + (yp * ik_dot2 + 2.0 * yp_dot * ik_dot) * phi_cos;
    }

    // ========== СПЕЦИАЛЬНАЯ ОБРАБОТКА BeiDou GEO СПУТНИКОВ ==========
    //
    // НАЗНАЧЕНИЕ: Особая обработка геостационарных спутников BeiDou (C01-C05)
    //
    // ОСОБЕННОСТИ GEO ОРБИТ:
    // - Геостационарная орбита на высоте ~35786 км
    // - Орбитальный период 24 часа (синхрон с вращением Земли)
    // - Требует специальных координатных преобразований
    //
    // МАТЕМАТИЧЕСКИЕ ОПЕРАЦИИ:
    // 1. Поворот на -5° вокруг оси X (согласно BDS-SIS-ICD)
    // 2. Поворот на угол CGCS2000_OMEGDOTE * Δt (компенсация вращения Земли)
    // 3. Компенсация вращения Земли в скорости
    //
    // ИСТОЧНИК: BeiDou Navigation Satellite System Signal In Space Interface Control Document
    //          (BDS-SIS-ICD), Version 2.1, Section 5.3.1.5
    if system == GnssSystem::BdsSystem && eph.svid <= 5 {
        // ШАГ 1: ПОВОРОТ НА -5° ВОКРУГ ОСИ X
        // Матрица поворота Rx(-5°) применяется к Y и Z компонентам:
        // | Y' |   |  cos(-5°)  -sin(-5°) | | Y |   |  cos(5°)   sin(5°) | | Y |
        // | Z' | = |  sin(-5°)   cos(-5°) | | Z | = | -sin(5°)   cos(5°) | | Z |

        // Сохраняем временные значения для последующих преобразований
        let yp_temp = pos_vel.y * COS_5 - pos_vel.z * SIN_5; // Y' (повернутая Y-координата)
        pos_vel.z = pos_vel.z * COS_5 + pos_vel.y * SIN_5; // Z' (повернутая Z-координата)
        let yp_dot_temp = pos_vel.vy * COS_5 - pos_vel.vz * SIN_5; // VY' (повернутая Y-скорость)
        pos_vel.vz = pos_vel.vz * COS_5 + pos_vel.vy * SIN_5; // VZ' (повернутая Z-скорость)

        // ШАГ 2: ПОВОРОТ НА УГОЛ CGCS2000_OMEGDOTE * Δt ВОКРУГ ОСИ Z
        // Компенсация вращения системы координат CGCS2000 относительно WGS84
        // CGCS2000_OMEGDOTE = 7.2921150e-5 рад/с (скорость вращения Земли)
        let omega_rot = CGCS2000_OMEGDOTE * delta_t; // Угол поворота за время Δt
        let sin_temp = omega_rot.sin();
        let cos_temp = omega_rot.cos();

        // Матрица поворота Rz(ωΔt) применяется к X и Y компонентам:
        // | X' |   | cos(ωΔt)  -sin(ωΔt) | | X |
        // | Y' | = | sin(ωΔt)   cos(ωΔt) | | Y |
        pos_vel.y = yp_temp * cos_temp - pos_vel.x * sin_temp; // Y'' (окончательная Y)
        pos_vel.x = pos_vel.x * cos_temp + yp_temp * sin_temp; // X'' (окончательная X)
        pos_vel.vy = yp_dot_temp * cos_temp - pos_vel.vx * sin_temp; // VY'' (окончательная VY)
        pos_vel.vx = pos_vel.vx * cos_temp + yp_dot_temp * sin_temp; // VX'' (окончательная VX)

        // ШАГ 3: КОМПЕНСАЦИЯ ВРАЩЕНИЯ ЗЕМЛИ В СКОРОСТИ
        // Учитываем эффект вращения Земли на относительную скорость спутника:
        // V₁' = V₁ + Y · ωₑ  (кориолисова поправка по X)
        // V₂' = V₂ - X · ωₑ  (кориолисова поправка по Y)
        pos_vel.vx += pos_vel.y * CGCS2000_OMEGDOTE;
        pos_vel.vy -= pos_vel.x * CGCS2000_OMEGDOTE;

        if let Some(acc_array) = acc.as_mut() {
            // first rotate -5 degree
            let yp_acc = acc_array[1] * COS_5 - acc_array[2] * SIN_5; // rotated ay
            acc_array[2] = acc_array[2] * COS_5 + acc_array[1] * SIN_5; // rotated az
            acc_array[1] = yp_acc * cos_temp - acc_array[0] * sin_temp;
            acc_array[0] = acc_array[0] * cos_temp + yp_acc * sin_temp;

            // earth rotate compensation on acceleration
            acc_array[0] += pos_vel.vy * CGCS2000_OMEGDOTE;
            acc_array[1] -= pos_vel.vx * CGCS2000_OMEGDOTE;
        }
    }

    // if ephemeris expire, return false
    if delta_t.abs() > 7200.0 {
        // println!("[GPS_POS_DEBUG] Ephemeris expired: delta_t = {} > 7200", delta_t);
        false
    } else {
        true // Упрощаем логику - просто возвращаем true если delta_t в пределах нормы
    }
}

/// GLONASS satellite position and velocity calculation from ephemeris
pub fn glonass_sat_pos_speed_eph(
    transmit_time: f64,
    eph: &mut GlonassEphemeris,
    pos_vel: &mut KinematicInfo,
    acc: Option<&mut [f64; 3]>,
) -> bool {
    let mut delta_t = transmit_time - eph.tb as f64;

    if delta_t > 43200.0 {
        delta_t -= 86400.0;
    } else if delta_t < -43200.0 {
        delta_t += 86400.0;
    }

    let _delta_t_residual = if (eph.flag & 0x2) == 0 {
        // satellite position and velocity in CIS coordinate
        let mut state = [
            eph.x,
            eph.y,
            eph.z,
            eph.vx - PZ90_OMEGDOTE * eph.y,
            eph.vy + PZ90_OMEGDOTE * eph.x,
            eph.vz,
            eph.ax,
            eph.ay,
            eph.az,
        ];

        let step_number = (delta_t / COARSE_STEP) as i32;
        let max_steps = 7200; // Максимум 7200 шагов (2 часа при 1-секундном шаге)
        if step_number >= 0 {
            let limited_steps = step_number.min(max_steps);
            for _ in 0..limited_steps {
                runge_kutta(COARSE_STEP, &mut state);
            }
        } else {
            let limited_steps = (-step_number).min(max_steps);
            for _ in 0..limited_steps {
                // КРИТИЧЕСКИЙ ФИКС: исправлен бесконечный цикл
                runge_kutta(-COARSE_STEP, &mut state);
            }
        }

        // Update ephemeris state
        eph.PosVelT.x = state[0];
        eph.PosVelT.y = state[1];
        eph.PosVelT.z = state[2];
        eph.PosVelT.vx = state[3];
        eph.PosVelT.vy = state[4];
        eph.PosVelT.vz = state[5];

        delta_t - (step_number as f64) * COARSE_STEP
    } else {
        // prediction from tc
        let mut state = [
            eph.PosVelT.x,
            eph.PosVelT.y,
            eph.PosVelT.z,
            eph.PosVelT.vx,
            eph.PosVelT.vy,
            eph.PosVelT.vz,
            eph.ax,
            eph.ay,
            eph.az,
        ];

        let mut delta_t1 = transmit_time - eph.tc;
        if delta_t1 > 43200.0 {
            delta_t1 -= 86400.0;
        } else if delta_t1 < -43200.0 {
            delta_t1 += 86400.0;
        }

        runge_kutta(delta_t1, &mut state);

        // Update ephemeris state
        eph.PosVelT.x = state[0];
        eph.PosVelT.y = state[1];
        eph.PosVelT.z = state[2];
        eph.PosVelT.vx = state[3];
        eph.PosVelT.vy = state[4];
        eph.PosVelT.vz = state[5];

        delta_t1
    };

    eph.tc = transmit_time;
    eph.flag |= 0x2; // can predict from eph.tc instead of eph.tb

    // CIS to CTS(PZ-90) conversion
    cis_to_cts(&eph.PosVelT, delta_t, pos_vel, acc);

    // УСПЕШНОЕ ЗАВЕРШЕНИЕ ВЫЧИСЛЕНИЙ
    // Возвращаем true, что означает успешный расчет позиции и скорости спутника
    // Полученные координаты в pos_vel готовы для использования в навигационных вычислениях
    true
}

/// Преобразование ECEF координат в геодезические LLA
///
/// Алгоритм основан на методе Bowring (1985) для преобразования
/// декартовых ECEF координат в эллипсоидальные:
///
/// **Входные параметры:**
/// - X, Y, Z: ECEF координаты в метрах (система WGS84)
///
/// **Выходные параметры:**
/// - φ (lat): широта в радианах (-π/2 до +π/2)
/// - λ (lon): долгота в радианах (-π до +π)
/// - h (alt): высота над эллипсоидом WGS84 в метрах
///
/// **Математические константы WGS84:**
/// - a = 6378137.0 м (большая полуось)
/// - b = 6356752.314245 м (малая полуось)
/// - e² = 0.00669437999014 (квадрат эксцентриситета)
pub fn ecef_to_lla(ecef_pos: &KinematicInfo) -> LlaPosition {
    // Радиальное расстояние от оси Z в экваториальной плоскости
    // p = √(X² + Y²) - проекция на экваториальную плоскость
    let p = (ecef_pos.x * ecef_pos.x + ecef_pos.y * ecef_pos.y).sqrt();

    // Обработка особого случая: северный/южный полюс
    // Когда p ≈ 0, точка лежит на оси вращения Земли
    // Долгота неопределена, принимаем 0°
    if p < 1e-10 {
        return LlaPosition {
            lon: 0.0, // По соглашению долгота = 0° на полюсах
            lat: if ecef_pos.z > 0.0 {
                PI / 2.0
            } else {
                -PI / 2.0
            }, // ±90° в зависимости от знака Z
            alt: if ecef_pos.z > 0.0 {
                ecef_pos.z - WGS_AXIS_B // Высота над северным полюсом
            } else {
                -ecef_pos.z - WGS_AXIS_B // Высота над южным полюсом
            },
        };
    }

    // МЕТОД BOWRING (1985): итеративное решение для широты
    //
    // Шаг 1: Вспомогательный угол θ (приближенная широта)
    // θ = atan(Z·a / (p·b))
    // Где: a - большая полуось, b - малая полуось WGS84
    let theta = (ecef_pos.z * WGS_AXIS_A / (p * WGS_AXIS_B)).atan();

    // Шаг 2: Точное вычисление широты φ с учетом эксцентриситета
    // φ = atan((Z + e'²·b·sin³θ) / (p - e²·a·cos³θ))
    // Где:
    // - e² = 0.00669437999014 (первый эксцентриситет WGS84)
    // - e'² = 0.00673949674228 (второй эксцентриситет WGS84)
    let lat = ((ecef_pos.z + WGS_E2_SQR * WGS_AXIS_B * theta.sin().powi(3))
        / (p - WGS_E1_SQR * WGS_AXIS_A * theta.cos().powi(3)))
    .atan();

    // Шаг 3: Вычисление долготы (просто!)
    // λ = atan2(Y, X) - 4-квадрантная арктангенс функция
    let lon = ecef_pos.y.atan2(ecef_pos.x);

    // Шаг 4: Вычисление высоты над эллипсоидом
    // N(φ) = a / √(1 - e²·sin²φ) - радиус кривизны в приме вертикала
    // h = p/cosφ - N(φ) - эллипсоидальная высота
    let n = WGS_AXIS_A / (1.0 - WGS_E1_SQR * lat.sin() * lat.sin()).sqrt(); // Радиус кривизны N(φ)
    let alt = p / lat.cos() - n; // Высота над WGS84 эллипсоидом

    LlaPosition { lat, lon, alt }
}

/// Преобразование геодезических LLA координат в ECEF
///
/// Прямое преобразование с использованием стандартных формул WGS84:
///
/// **Математическая модель:**
/// X = (N(φ) + h) · cos(φ) · cos(λ)
/// Y = (N(φ) + h) · cos(φ) · sin(λ)
/// Z = (N(φ)(1-e²) + h) · sin(φ)
///
/// Где:
/// - N(φ) = a/√(1-e²sin²φ) - радиус кривизны в приме вертикала
/// - φ, λ, h - широта, долгота, высота
/// - e² - квадрат эксцентриситета WGS84
///
/// **Особенность:** Скорость анулируется (статичное преобразование)
pub fn lla_to_ecef(lla_pos: &LlaPosition) -> KinematicInfo {
    // Вычисление радиуса кривизны в приме вертикала
    // N(φ) = a / √(1 - e² · sin²(φ)) - основное соотношение для WGS84
    let n = WGS_AXIS_A / (1.0 - WGS_E1_SQR * lla_pos.lat.sin() * lla_pos.lat.sin()).sqrt();

    // Прямое применение формул WGS84 для преобразования:
    KinematicInfo {
        // X = (N + h) · cos(φ) · cos(λ)
        x: (n + lla_pos.alt) * lla_pos.lat.cos() * lla_pos.lon.cos(),
        // Y = (N + h) · cos(φ) · sin(λ)
        y: (n + lla_pos.alt) * lla_pos.lat.cos() * lla_pos.lon.sin(),
        // Z = (N(1-e²) + h) · sin(φ) - учитываем сжатие Земли
        z: (n * (1.0 - WGS_E1_SQR) + lla_pos.alt) * lla_pos.lat.sin(),
        // Скорость = 0 (статичное преобразование только позиции)
        vx: 0.0,
        vy: 0.0,
        vz: 0.0,
    }
}

/// Calculate conversion matrix from ECEF position
pub fn calc_conv_matrix_from_ecef(position: &KinematicInfo) -> ConvertMatrix {
    let pos_lla = ecef_to_lla(position);
    calc_conv_matrix_lla(&pos_lla)
}

/// Вычисление матрицы преобразования ECEF → ENU (East-North-Up)
///
/// Матрица поворота для перехода от глобальных ECEF к локальным топоцентрическим:
///
/// **Математическая модель:**
/// ┌        ┐   ┌                                     ┐ ┌    ┐
/// │ East   │   │ -sin(λ)      cos(λ)         0   │ │ ΔX │
/// │ North  │ = │ -sin(φ)cos(λ) -sin(φ)sin(λ) cos(φ) │ │ ΔY │
/// │ Up     │   │  cos(φ)cos(λ)  cos(φ)sin(λ) sin(φ) │ │ ΔZ │
/// └        ┘   └                                     ┘ └    ┘
///
/// Где:
/// - φ (lat) - широта опорной точки
/// - λ (lon) - долгота опорной точки
/// - ΔX, ΔY, ΔZ - относительные ECEF координаты
pub fn calc_conv_matrix_lla(position: &LlaPosition) -> ConvertMatrix {
    // DEBUG: Матрица отключена для уменьшения вывода
    // println!("[MATRIX-DEBUG] Input LLA: lat={:.4}°, lon={:.4}°",
    //          position.lat.to_degrees(), position.lon.to_degrees());

    // Коэффициенты матрицы преобразования ECEF → ENU:
    ConvertMatrix {
        // Направление East (Восток): перпендикулярно меридиану
        x2e: -position.lon.sin(), // -sin(λ) - компонента X для East
        y2e: position.lon.cos(),  // cos(λ) - компонента Y для East

        // Направление North (Север): по меридиану, касательно к эллипсоиду
        x2n: -position.lat.sin() * position.lon.cos(), // -sin(φ)cos(λ)
        y2n: -position.lat.sin() * position.lon.sin(), // -sin(φ)sin(λ)
        z2n: position.lat.cos(),                       // cos(φ)

        // Направление Up (Вверх): по нормали к эллипсоиду
        x2u: position.lat.cos() * position.lon.cos(), // cos(φ)cos(λ)
        y2u: position.lat.cos() * position.lon.sin(), // cos(φ)sin(λ)
        z2u: position.lat.sin(),                      // sin(φ)
    }
}

pub fn calc_up_vector(position: &LlaPosition) -> [f64; 3] {
    [
        position.lat.cos() * position.lon.cos(),
        position.lat.cos() * position.lon.sin(),
        position.lat.sin(),
    ]
}

/// Convert ENU speed to course and speed
pub fn speed_enu_to_course(speed: &mut LocalSpeed) {
    speed.speed = (speed.ve * speed.ve + speed.vn * speed.vn).sqrt();
    speed.course = speed.ve.atan2(speed.vn);
    if speed.course < 0.0 {
        speed.course += PI2;
    }
}

/// Convert course and speed to ENU speed
pub fn speed_course_to_enu(speed: &mut LocalSpeed) {
    speed.ve = speed.speed * speed.course.sin();
    speed.vn = speed.speed * speed.course.cos();
}

/// Convert ECEF speed to local speed
pub fn speed_ecef_to_local(
    convert_matrix: &ConvertMatrix,
    pos_vel: &KinematicInfo,
    speed: &mut LocalSpeed,
) {
    speed.ve = pos_vel.vx * convert_matrix.x2e + pos_vel.vy * convert_matrix.y2e;
    speed.vn = pos_vel.vx * convert_matrix.x2n
        + pos_vel.vy * convert_matrix.y2n
        + pos_vel.vz * convert_matrix.z2n;
    speed.vu = pos_vel.vx * convert_matrix.x2u
        + pos_vel.vy * convert_matrix.y2u
        + pos_vel.vz * convert_matrix.z2u;
    speed_enu_to_course(speed);
}

/// Convert local speed to ECEF speed
pub fn speed_local_to_ecef(
    convert_matrix: &ConvertMatrix,
    speed: &LocalSpeed,
    pos_vel: &mut KinematicInfo,
) {
    pos_vel.vx = speed.ve * convert_matrix.x2e
        + speed.vn * convert_matrix.x2n
        + speed.vu * convert_matrix.x2u;
    pos_vel.vy = speed.ve * convert_matrix.y2e
        + speed.vn * convert_matrix.y2n
        + speed.vu * convert_matrix.y2u;
    pos_vel.vz = speed.vn * convert_matrix.z2n + speed.vu * convert_matrix.z2u;
}

/// Convert local speed to ECEF speed using LLA position
pub fn speed_local_to_ecef_lla(
    lla_pos: &LlaPosition,
    speed: &LocalSpeed,
    pos_vel: &mut KinematicInfo,
) {
    let convert_matrix = calc_conv_matrix_lla(lla_pos);
    speed_local_to_ecef(&convert_matrix, speed, pos_vel);
}

/// Calculate satellite elevation and azimuth
pub fn sat_el_az_from_lla(
    position_lla: &LlaPosition,
    los_vector: &[f64; 3],
    elevation: &mut f64,
    azimuth: &mut f64,
) {
    let convert_matrix = calc_conv_matrix_lla(position_lla);
    let local_los = [
        los_vector[0] * convert_matrix.x2e + los_vector[1] * convert_matrix.y2e,
        los_vector[0] * convert_matrix.x2n
            + los_vector[1] * convert_matrix.y2n
            + los_vector[2] * convert_matrix.z2n,
        los_vector[0] * convert_matrix.x2u
            + los_vector[1] * convert_matrix.y2u
            + los_vector[2] * convert_matrix.z2u,
    ];

    // Убираем отладку для краткости

    // КРИТИЧЕСКИЙ ФИКС: Правильный расчёт азимута
    // В навигации азимут измеряется от севера по часовой стрелке
    // Но из-за особенностей системы координат нужно инвертировать восточную компоненту
    *azimuth = (-local_los[0]).atan2(local_los[1]);
    if *azimuth < 0.0 {
        *azimuth += PI2;
    }
    *elevation = local_los[2].asin();
}

/// Calculate satellite elevation and azimuth from receiver and satellite positions
pub fn sat_el_az_from_positions(
    receiver: &KinematicInfo,
    satellite: &KinematicInfo,
    elevation: &mut f64,
    azimuth: &mut f64,
) {
    let mut los_vector = [0.0; 3];
    let position = ecef_to_lla(receiver);

    let receiver_pos = [
        receiver.pos_vel()[0],
        receiver.pos_vel()[1],
        receiver.pos_vel()[2],
    ];
    let satellite_pos = [
        satellite.pos_vel()[0],
        satellite.pos_vel()[1],
        satellite.pos_vel()[2],
    ];

    geometry_distance_array(&receiver_pos, &satellite_pos, Some(&mut los_vector));
    sat_el_az_from_lla(&position, &los_vector, elevation, azimuth);
}

/// Calculate geometry distance between two positions
pub fn geometry_distance_array(
    receiver: &[f64; 3],
    satellite: &[f64; 3],
    los_vector: Option<&mut [f64; 3]>,
) -> f64 {
    let dx = satellite[0] - receiver[0];
    let dy = satellite[1] - receiver[1];
    let dz = satellite[2] - receiver[2];
    let r_geometric = (dx * dx + dy * dy + dz * dz).sqrt();

    // Calculate LOS vector using geometric distance (before Earth rotation compensation)
    if let Some(los) = los_vector {
        los[0] = dx / r_geometric;
        los[1] = dy / r_geometric;
        los[2] = dz / r_geometric;
    }

    // add earth rotate compensation to final distance
    r_geometric
        + (satellite[0] * receiver[1] - satellite[1] * receiver[0]) * (WGS_OMEGDOTE / LIGHT_SPEED)
}

/// Calculate geometry distance between two kinematic info structures
pub fn geometry_distance(
    receiver: &KinematicInfo,
    satellite: &KinematicInfo,
    los_vector: Option<&mut [f64; 3]>,
) -> f64 {
    let receiver_pos = receiver.pos_vel();
    let satellite_pos = satellite.pos_vel();
    geometry_distance_array(
        &[receiver_pos[0], receiver_pos[1], receiver_pos[2]],
        &[satellite_pos[0], satellite_pos[1], satellite_pos[2]],
        los_vector,
    )
}

/// Calculate satellite relative speed
pub fn sat_relative_speed(receiver: &KinematicInfo, satellite: &KinematicInfo) -> f64 {
    let dx = receiver.x - satellite.x;
    let dy = receiver.y - satellite.y;
    let dz = receiver.z - satellite.z;
    let dvx = receiver.vx - satellite.vx;
    let dvy = receiver.vy - satellite.vy;
    let dvz = receiver.vz - satellite.vz;
    let distance = (dx * dx + dy * dy + dz * dz).sqrt();

    (dx * dvx + dy * dvy + dz * dvz) / distance
}

/// Calculate GPS ionosphere delay
pub fn gps_iono_delay(
    iono_param: &IonoParam,
    time: f64,
    lat: f64,
    lon: f64,
    elevation: f64,
    azimuth: f64,
) -> f64 {
    let el = elevation / PI;
    let psi = 0.0137 / (el + 0.11) - 0.022;
    let mut lat_rad = lat / PI + psi * azimuth.cos();
    lat_rad = lat_rad.clamp(-0.416, 0.416);

    let lon_rad = lon / PI + psi * azimuth.sin() / (lat_rad * PI).cos();
    lat_rad += 0.064 * ((lon_rad - 1.617) * PI).cos();

    let f = 1.0 + 16.0 * (0.53 - el).powi(3);
    let mut per = iono_param.b0
        + (iono_param.b1 + (iono_param.b2 + iono_param.b3 * lat_rad) * lat_rad) * lat_rad;

    if per < 72000.0 {
        per = 72000.0;
    }

    let mut t = (43200.0 * lon_rad) + time;
    while t >= 86400.0 {
        t -= 86400.0;
    }
    while t < 0.0 {
        t += 86400.0;
    }

    let x = PI2 * (t - 50400.0) / per;
    let f_scaled = f * LIGHT_SPEED;

    if x >= 1.57 || x <= -1.57 {
        f_scaled * 5e-9
    } else {
        let amp = iono_param.a0
            + (iono_param.a1 + (iono_param.a2 + iono_param.a3 * lat_rad) * lat_rad) * lat_rad;
        if amp < 0.0 {
            f_scaled * 5e-9
        } else {
            let x_sq = x * x;
            let x1 = 1.0 - x_sq / 2.0 + x_sq * x_sq / 24.0;
            f_scaled * (5e-9 + amp * x1)
        }
    }
}

/// Calculate troposphere delay
pub fn tropo_delay(lat: f64, altitude: f64, elevation: f64) -> f64 {
    const T0: f64 = 273.16 + 15.0; // average temperature at sea level

    if !(-100.0..=1e4).contains(&altitude) || elevation <= 0.0 {
        return 0.0;
    }

    let altitude_adj = if altitude < 0.0 { 0.0 } else { altitude };

    let pressure = 1013.25 * (1.0 - 2.2557e-5 * altitude_adj).powf(5.2568);
    let t = T0 - 6.5e-3 * altitude_adj;
    let e = 6.108 * REL_HUMI * ((17.15 * t - 4684.0) / (t - 38.45)).exp();

    let z = PI / 2.0 - elevation;
    let trph = 0.0022767 * pressure
        / (1.0 - 0.00266 * (2.0 * lat).cos() - 0.00028 * altitude_adj / 1e3)
        / z.cos();
    let trpw = 0.002277 * (1255.0 / t + 0.05) * e / z.cos();

    trph + trpw
}

// Helper functions for GLONASS calculations

fn runge_kutta(h: f64, state: &mut [f64; 9]) {
    let mut state1 = [0.0; 9];
    let mut vel_acc1 = [0.0; 6];
    let mut vel_acc2 = [0.0; 6];
    let mut vel_acc3 = [0.0; 6];
    let mut vel_acc4 = [0.0; 6];

    state1[6] = state[6];
    state1[7] = state[7];
    state1[8] = state[8];

    calc_acceleration(state, &mut vel_acc1);
    predict_state(state, &mut state1, &vel_acc1, 0.5 * h);
    calc_acceleration(&state1, &mut vel_acc2);
    predict_state(state, &mut state1, &vel_acc2, 0.5 * h);
    calc_acceleration(&state1, &mut vel_acc3);
    predict_state(state, &mut state1, &vel_acc3, h);
    calc_acceleration(&state1, &mut vel_acc4);

    for i in 0..6 {
        vel_acc1[i] = (vel_acc1[i] + vel_acc4[i] + 2.0 * (vel_acc2[i] + vel_acc3[i])) / 6.0;
    }

    let mut new_state = [0.0; 9];
    predict_state(state, &mut new_state, &vel_acc1, h);
    for i in 0..9 {
        state[i] = new_state[i];
    }
}

fn calc_acceleration(state: &[f64; 9], acc: &mut [f64; 6]) {
    acc[0] = state[3];
    acc[1] = state[4];
    acc[2] = state[5];

    let r2 = state[0] * state[0] + state[1] * state[1] + state[2] * state[2];
    let r3 = r2 * r2.sqrt();
    let coef20 = PZ90_C20AE2 / r2;
    let scale20 = PZ90_GM * (1.5 * coef20 * (5.0 * state[2] * state[2] / r2 - 1.0) - 1.0) / r3;

    acc[3] = scale20 * state[0] + state[6]; // x acceleration
    acc[4] = scale20 * state[1] + state[7]; // y acceleration
    acc[5] = (scale20 - 3.0 * PZ90_GM * coef20 / r3) * state[2] + state[8]; // z acceleration
}

fn predict_state(state: &[f64; 9], state1: &mut [f64; 9], vel_acc: &[f64; 6], step: f64) {
    for i in 0..6 {
        state1[i] = state[i] + vel_acc[i] * step;
    }
}

fn cis_to_cts(
    state: &KinematicInfo,
    delta_t: f64,
    cts_pos: &mut KinematicInfo,
    mut acc: Option<&mut [f64; 3]>,
) {
    // calculate rotate angle between CIS and CTS
    let omega = PZ90_OMEGDOTE * delta_t;
    let cos_value = omega.cos();
    let sin_value = omega.sin();

    // calculate position
    cts_pos.x = state.x * cos_value + state.y * sin_value;
    cts_pos.y = state.y * cos_value - state.x * sin_value;
    cts_pos.z = state.z;

    // calculate velocity
    cts_pos.vx = state.vx * cos_value + state.vy * sin_value;
    cts_pos.vy = state.vy * cos_value - state.vx * sin_value;
    cts_pos.vz = state.vz;

    // calculate acceleration
    if let Some(acc_array) = acc.as_mut() {
        acc_array[0] = state.x * cos_value + state.y * sin_value; // assuming state has acceleration
        acc_array[1] = state.y * cos_value - state.x * sin_value;
        acc_array[2] = state.z;
    }

    // additional compensation on velocity/acceleration
    let cos_comp = cos_value * PZ90_OMEGDOTE;
    let sin_comp = sin_value * PZ90_OMEGDOTE;
    cts_pos.vx -= state.x * sin_comp - state.y * cos_comp;
    cts_pos.vy -= state.y * sin_comp + state.x * cos_comp;

    if let Some(acc_array) = acc.as_mut() {
        acc_array[0] -= state.vx * sin_comp - state.vy * cos_comp;
        acc_array[1] -= state.vy * sin_comp + state.vx * cos_comp;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ecef_to_lla_conversion() {
        let ecef = KinematicInfo {
            x: 4000000.0,
            y: 3000000.0,
            z: 5000000.0,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
        };

        let lla = ecef_to_lla(&ecef);
        let ecef_back = lla_to_ecef(&lla);

        // Численная погрешность преобразований на уровне миллиметров
        let tol = 1e-2; // 1 см
        assert!((ecef.x - ecef_back.x).abs() < tol);
        assert!((ecef.y - ecef_back.y).abs() < tol);
        assert!((ecef.z - ecef_back.z).abs() < tol);
    }

    #[test]
    fn test_geometry_distance() {
        let receiver = [0.0, 0.0, 0.0];
        let satellite = [1000.0, 0.0, 0.0];
        let mut los_vector = [0.0; 3];

        let distance = geometry_distance_array(&receiver, &satellite, Some(&mut los_vector));

        assert!((distance - 1000.0).abs() < 1e-6);
        assert!((los_vector[0] - 1.0).abs() < 1e-6);
        assert!(los_vector[1].abs() < 1e-6);
        assert!(los_vector[2].abs() < 1e-6);
    }
}
// Additional functions for trajectory support
