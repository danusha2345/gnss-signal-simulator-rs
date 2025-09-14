use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::OnceLock;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum LogLevel {
    Quiet = 0,
    Normal = 1,
    Verbose = 2,
}

impl From<usize> for LogLevel {
    fn from(v: usize) -> Self {
        match v {
            0 => LogLevel::Quiet,
            2 => LogLevel::Verbose,
            _ => LogLevel::Normal,
        }
    }
}

static LEVEL: OnceLock<AtomicUsize> = OnceLock::new();

fn init_level() -> LogLevel {
    match std::env::var("GNSS_VERBOSE") {
        Ok(v) if matches!(v.as_str(), "1" | "true" | "TRUE" | "yes" | "on") => LogLevel::Verbose,
        _ => LogLevel::Normal,
    }
}

fn level_cell() -> &'static AtomicUsize {
    LEVEL.get_or_init(|| AtomicUsize::new(init_level() as usize))
}

pub fn set_level(level: LogLevel) {
    level_cell().store(level as usize, Ordering::Relaxed);
}

pub fn level() -> LogLevel {
    level_cell().load(Ordering::Relaxed).into()
}

pub fn is_verbose() -> bool {
    level() == LogLevel::Verbose
}

pub fn is_quiet() -> bool {
    level() == LogLevel::Quiet
}

#[macro_export]
macro_rules! vprintln {
    ($($arg:tt)*) => {
        if $crate::logutil::is_verbose() {
            println!($($arg)*);
        }
    };
}

#[macro_export]
macro_rules! dprintln {
    ($($arg:tt)*) => {
        if cfg!(debug_assertions) && $crate::logutil::is_verbose() {
            println!($($arg)*);
        }
    };
}
