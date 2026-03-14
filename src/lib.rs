#![allow(
    clippy::needless_range_loop,
    clippy::manual_memcpy,
    clippy::unnecessary_cast,
    clippy::erasing_op,
    clippy::missing_safety_doc,
    clippy::too_many_arguments
)]
//! # GNSS Rust Library
//!
//! Библиотека для обработки данных глобальных навигационных спутниковых систем (ГНСС/GNSS).
//! Портирована с C++/C для работы с различными типами навигационных сообщений и сигналов
//! спутниковых систем GPS, ГЛОНАСС, BeiDou, Galileo.
//!
//! ## Основные компоненты:
//! - Обработка навигационных битов различных форматов
//! - Генерация промежуточных частотных данных (IF)
//! - Расчеты траекторий и параметров спутников
//! - Работа с альманахами и эфемеридами
//! - Преобразования координат и времени
//!
//! ## Поддерживаемые системы:
//! - GPS (L1, L2, L5)
//! - ГЛОНАСС (G1, G2, G3)
//! - BeiDou/Compass (B1, B2, B3)
//! - Galileo (E1, E5a, E5b, E6)

pub mod almanac;
pub mod bcnav1bit;
pub mod bcnav2bit;
pub mod bcnav3bit;
pub mod bcnavbit;
pub mod crc24q;
pub mod d1d2navbit;
pub mod debug_positions;
pub mod fastmath;
pub mod fnavbit;
// pub mod fnavbit_backup;
pub mod cnav2bit;
pub mod cnavbit;
pub mod complex_number;
pub mod constants;
pub mod coordinate;
pub mod delay_model;
pub mod gnavbit;
pub mod gnsstime;
pub mod ifdatagen;
pub mod inavbit;
pub mod json_interpreter; // Восстановлено для RINEX парсинга
pub mod json_parser;
pub mod l5cnavbit;
pub mod ldpc;
pub mod lnavbit;
pub mod logutil;
pub mod memory_code;
pub mod nav_data; // NEW: Unified NavData enum for parallelization
pub mod nav_decode;
pub mod navbit;
pub mod navdata;
pub mod pilotbit;
pub mod powercontrol;
pub mod prngenerate;
pub mod sat_if_signal;
pub mod satellite_param;
pub mod satellite_signal;
pub mod trajectory;
pub mod types;
// МАКСИМАЛЬНОЕ АППАРАТНОЕ УСКОРЕНИЕ
pub mod avx512_intrinsics;
pub mod benchmarks;
pub mod cuda_acceleration;
pub mod pvt;
// ОПТИМИЗИРОВАННОЕ КЭШИРОВАНИЕ ДЛЯ GNSS СИГНАЛОВ
pub mod cache_manager;
pub mod optimized_sat_signal;
pub mod cache_integration_patch;

pub use almanac::*;
pub use bcnav1bit::*;
pub use bcnav2bit::*;
pub use bcnav3bit::*;
pub use bcnavbit::*;
pub use d1d2navbit::*;
pub use fastmath::*;
pub use fnavbit::*;
// pub use fnavbit_backup::*;
pub use cnav2bit::*;
pub use cnavbit::*;
pub use complex_number::*;
pub use constants::*;
pub use coordinate::*;
pub use delay_model::*;
pub use gnavbit::*;
pub use gnsstime::*;
pub use ifdatagen::{GenerationStats, IFDataGen, NavBitTrait};
pub use types::*;
// ВРЕМЕННО ОТКЛЮЧЕНО: pub use json_interpreter::*;
pub use avx512_intrinsics::*;
pub use benchmarks::*;
pub use cuda_acceleration::*;
pub use cache_manager::*;
pub use optimized_sat_signal::*;
pub use cache_integration_patch::*;
pub use inavbit::*;
pub use json_parser::*;
pub use l5cnavbit::*;
pub use ldpc::*;
pub use lnavbit::*;
pub use memory_code::*;
pub use nav_data::{NavData, NavMessageType};
pub use navbit::*;
pub use navdata::{CNavData, NavDataType};
pub use pilotbit::*;
pub use powercontrol::*;
pub use prngenerate::*;
pub use sat_if_signal::*;
pub use satellite_param::*;
pub use satellite_signal::*;
pub use trajectory::*;
