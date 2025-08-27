//----------------------------------------------------------------------
// memory_code.rs:
//   Galileo memory codes for E1 and E6 signals
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

// Galileo E1 memory codes (actual size from file)
pub static E1_MEMORY_CODE: [u32; 12800] = include!("memory_code_e1.rs");

// Galileo E6 memory codes (actual size from file)
pub static E6_MEMORY_CODE: [u32; 15984] = include!("memory_code_e6.rs");