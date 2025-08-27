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
pub mod fastmath;
pub mod fnavbit;
// pub mod fnavbit_backup;
pub mod gnavbit;
pub mod gnsstime;
pub mod ifdatagen;
pub mod types;
pub mod json_interpreter;
pub mod json_parser;
pub mod l5cnavbit;
pub mod lnavbit;
pub mod navbit;
pub mod navdata;
pub mod pilotbit;
pub mod powercontrol;
pub mod prngenerate;
pub mod constants;
pub mod memory_code;
pub mod trajectory;
pub mod satellite_param;
pub mod satellite_signal;
pub mod sat_if_signal;
pub mod cnavbit;
pub mod cnav2bit;
pub mod complex_number;
pub mod coordinate;
pub mod inavbit;
pub mod ldpc;

pub use almanac::*;
pub use bcnav1bit::*;
pub use bcnav2bit::*;
pub use bcnav3bit::*;
pub use bcnavbit::*;
pub use d1d2navbit::*;
pub use fastmath::*;
pub use fnavbit::*;
// pub use fnavbit_backup::*;
pub use gnavbit::*;
pub use gnsstime::*;
pub use ifdatagen::{NavBitTrait, IFDataGen, GenerationStats};
pub use types::*;
pub use constants::*;
pub use cnavbit::*;
pub use cnav2bit::*;
pub use complex_number::*;
pub use coordinate::*;
pub use json_interpreter::*;
pub use json_parser::*;
pub use l5cnavbit::*;
pub use lnavbit::*;
pub use navbit::*;
pub use navdata::{CNavData, NavDataType};
pub use pilotbit::*;
pub use powercontrol::*;
pub use prngenerate::*;
pub use memory_code::*;
pub use trajectory::*;
pub use satellite_param::*;
pub use satellite_signal::*;
pub use sat_if_signal::*;
pub use inavbit::*;
pub use ldpc::*;
