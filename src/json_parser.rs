//! # Модуль парсинга JSON конфигураций
//!
//! Этот модуль реализует потоковую обработку JSON данных для ГНСС системы.
//! Основные возможности:
//! - Парсинг конфигурационных файлов в формате JSON
//! - Обработка потоковых JSON данных для real-time систем
//! - Извлечение параметров спутников, траекторий и сигналов из JSON
//! - Валидация структуры и типов данных в JSON объектах
//! - Поддержка различных числовых форматов и строковых значений
//!
//! Модуль обеспечивает гибкую настройку ГНСС системы через
//! конфигурационные файлы и внешние источники данных.

//----------------------------------------------------------------------
// json_parser.rs:
//   JSON stream process class implementation
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use std::ffi::CStr;
use std::os::raw::c_char;
use std::ptr;

// Constants
const MAX_KEY_LENGTH: usize = 32 - 1;
const MAX_STRING_LENGTH: usize = 192 - 1;

// JSON Number Union
#[repr(C)]
#[derive(Copy, Clone)]
pub union JsonNumberUnion {
    pub d_data: f64,
    pub l_data: i64,
}

// JsonObject ValueType enum
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ValueType {
    ValueTypeNull = 0,
    ValueTypeObject = 1,
    ValueTypeArray = 2,
    ValueTypeString = 3,
    ValueTypeIntNumber = 4,
    ValueTypeFloatNumber = 5,
    ValueTypeTrue = 6,
    ValueTypeFalse = 7,
}

// JsonObject structure
#[repr(C)]
pub struct JsonObjectParser {
    pub key: [c_char; MAX_KEY_LENGTH + 1],
    pub value_type: ValueType,
    pub p_next_object: *mut JsonObjectParser,
    pub p_object_content: *mut JsonObjectParser,
    pub number: JsonNumberUnion,
    pub string: [c_char; MAX_STRING_LENGTH + 1],
}

// Alias для совместимости с другими модулями
pub type JsonObject = JsonObjectParser;

impl JsonObjectParser {
    fn new() -> Box<JsonObjectParser> {
        Box::new(JsonObjectParser {
            key: [0; MAX_KEY_LENGTH + 1],
            value_type: ValueType::ValueTypeNull,
            p_next_object: ptr::null_mut(),
            p_object_content: ptr::null_mut(),
            number: JsonNumberUnion { l_data: 0 },
            string: [0; MAX_STRING_LENGTH + 1],
        })
    }
}

// Function pointer type for GetStream
type GetStreamFunc = fn(&mut JsonStream, source: *mut std::ffi::c_void) -> i32;

// JsonStream structure
pub struct JsonStream {
    root_object: *mut JsonObjectParser,
    stream: [c_char; 256],
    p: *const c_char,
    reader: Option<BufReader<File>>,
    current_line: String,
    line_pos: usize,
}

impl Default for JsonStream {
    fn default() -> Self {
        Self::new()
    }
}

impl JsonStream {
    pub fn new() -> Self {
        let mut instance = JsonStream {
            root_object: ptr::null_mut(),
            stream: [0; 256],
            p: ptr::null(),
            reader: None,
            current_line: String::new(),
            line_pos: 0,
        };
        // Инициализируем указатель p на stream
        instance.p = instance.stream.as_ptr();
        instance
    }

    pub fn parse_string(&mut self, json_string: &str) -> i32 {
        // Очистим предыдущий объект
        if !self.root_object.is_null() {
            self.delete_tree(self.root_object);
            self.root_object = ptr::null_mut();
        }

        // Скопируем строку в внутренний буфер
        let bytes = json_string.as_bytes();
        let copy_len = std::cmp::min(bytes.len(), self.stream.len() - 1);
        
        for i in 0..copy_len {
            self.stream[i] = bytes[i] as c_char;
        }
        self.stream[copy_len] = 0; // null terminator
        
        self.p = self.stream.as_ptr();
        self.reader = None; // убеждаемся что режим файла отключен

        // Парсим объект
        let result = self.parse_object(false, Self::get_string_stream, ptr::null_mut());
        self.root_object = result;
        
        if result.is_null() { -1 } else { 0 }
    }

    fn get_string_stream(&mut self, _source: *mut std::ffi::c_void) -> i32 {
        -1 // Для строкового режима всегда возвращаем EOF
    }

    pub fn delete_tree(&mut self, object: *mut JsonObjectParser) {
        if object.is_null() {
            return;
        }

        unsafe {
            let mut current_object = object;
            while !current_object.is_null() {
                let obj_ref = &*current_object;
                
                if obj_ref.value_type == ValueType::ValueTypeObject || 
                   obj_ref.value_type == ValueType::ValueTypeArray {
                    self.delete_tree(obj_ref.p_object_content);
                }
                
                let next_object = obj_ref.p_next_object;
                drop(Box::from_raw(current_object));
                current_object = next_object;
            }
        }
    }

    pub fn read_file(&mut self, file: &str) -> i32 {
        match File::open(file) {
            Ok(f) => {
                self.reader = Some(BufReader::new(f));
                self.line_pos = 0;
                self.current_line.clear();
                
                // ✅ Парсим, пока reader еще доступен
                let result = self.parse_object(false, Self::get_file_stream, ptr::null_mut());
                
                // ✅ Сохраняем результат ПЕРЕД закрытием reader
                self.root_object = result;
                self.reader = None;  // Теперь можно закрыть
                0
            },
            Err(e) => {
                eprintln!("[ERROR]\tFailed to open file: {} - Error: {}", file, e);
                -1
            }
        }
    }

    pub fn write_file(&self, file: &str) -> i32 {
        match File::create(file) {
            Ok(mut fp) => {
                if !self.root_object.is_null() {
                    unsafe {
                        self.output_object(&mut fp, (*self.root_object).p_object_content, 1, true);
                    }
                }
                0
            },
            Err(_) => -1,
        }
    }

    pub fn parse_object(&mut self, is_object: bool, get_stream_func: GetStreamFunc, source: *mut std::ffi::c_void) -> *mut JsonObjectParser {
        let mut object: *mut JsonObjectParser = ptr::null_mut();
        let mut cur_object: *mut JsonObjectParser = ptr::null_mut();
        let mut stage = if is_object { 0 } else { 3 };

        if !is_object {
            // for array, create new object list
            cur_object = self.get_new_object();
            object = cur_object;
            if object.is_null() {
                return ptr::null_mut();
            }
        }

        loop {
            if self.current_char() == 0
                && get_stream_func(self, source) < 0 {
                    break;
                }
            if self.current_char() == 0 {
                continue;
            }

            match stage {
                0 => { // waiting for '{'
                    if self.current_char() == b'{' as c_char {
                        stage = 1;
                        cur_object = self.get_new_object();
                        object = cur_object;
                        if object.is_null() {
                            return ptr::null_mut();
                        }
                    }
                },
                1 => { // waiting for '\"' as start of key
                    if self.current_char() == b'\"' as c_char {
                        unsafe {
                            self.copy_string(&mut (*cur_object).key, MAX_KEY_LENGTH);
                            // Debug check for elevationAdjust
                            let key_str = CStr::from_ptr((*cur_object).key.as_ptr()).to_string_lossy();
                            if key_str == "elevationAdjust" {
                                // Debug breakpoint equivalent
                            }
                        }
                        stage = 2;
                    }
                },
                2 => { // waiting for ':'
                    if self.current_char() == b':' as c_char {
                        stage = 3;
                    }
                },
                3 => { // waiting for value
                    if self.current_char() == b'{' as c_char {
                        // value is an object
                        unsafe {
                            (*cur_object).value_type = ValueType::ValueTypeObject;
                            (*cur_object).p_object_content = self.parse_object(true, get_stream_func, source);
                        }
                    } else if self.current_char() == b'[' as c_char {
                        // value is an array
                        self.advance_char();
                        unsafe {
                            (*cur_object).value_type = ValueType::ValueTypeArray;
                            (*cur_object).p_object_content = self.parse_object(false, get_stream_func, source);
                        }
                    } else if !self.is_white_space(self.current_char()) {
                        self.get_value_content(cur_object);
                    } else {
                        self.advance_char();
                        continue;
                    }
                    stage = 4;
                },
                4 => { // determine whether end of object or next key/value pair
                    if self.current_char() == b'}' as c_char || self.current_char() == b']' as c_char {
                        return object;
                    } else if self.current_char() == b',' as c_char {
                        unsafe {
                            (*cur_object).p_next_object = self.get_new_object();
                            if (*cur_object).p_next_object.is_null() {
                                return object;
                            }
                            cur_object = (*cur_object).p_next_object;
                        }
                        stage = if is_object { 1 } else { 3 }; // go to next key/value pair or value
                    }
                },
                _ => {}
            }
            self.advance_char();
        }

        object
    }

    fn get_new_object(&self) -> *mut JsonObjectParser {
        let object = JsonObjectParser::new();
        Box::into_raw(object)
    }

    fn get_file_stream(&mut self, _source: *mut std::ffi::c_void) -> i32 {
        if let Some(ref mut reader) = self.reader {
            self.current_line.clear();
            match reader.read_line(&mut self.current_line) {
                Ok(0) => -1, // EOF
                Ok(_) => {
                    self.line_pos = 0;
                    1
                },
                Err(_) => -1,
            }
        } else {
            -1
        }
    }

    fn current_char(&self) -> c_char {
        if self.reader.is_some() {
            if self.line_pos < self.current_line.len() {
                self.current_line.as_bytes()[self.line_pos] as c_char
            } else {
                0
            }
        } else {
            unsafe {
                if self.p.is_null() {
                    0
                } else {
                    *self.p
                }
            }
        }
    }

    fn advance_char(&mut self) {
        if self.reader.is_some() {
            self.line_pos += 1;
        } else {
            unsafe {
                if !self.p.is_null() {
                    self.p = self.p.offset(1);
                }
            }
        }
    }

    fn copy_string(&mut self, dest: &mut [c_char], max_length: usize) -> i32 {
        let mut length = 0;
        
        self.advance_char(); // skip starting '\"'
        
        while self.current_char() != 0 && self.current_char() != b'\"' as c_char {
            if length < max_length {
                if self.current_char() == b'\\' as c_char {
                    dest[length] = self.escape_character();
                } else {
                    dest[length] = self.current_char();
                }
                length += 1;
            }
            self.advance_char();
        }
        
        if length < dest.len() {
            dest[length] = 0;
        }
        
        if self.current_char() == 0 {
            // Move back one position if we hit end of string
            if self.reader.is_some() {
                if self.line_pos > 0 {
                    self.line_pos -= 1;
                }
            } else {
                unsafe {
                    if !self.p.is_null() {
                        self.p = self.p.offset(-1);
                    }
                }
            }
        }
        
        length as i32
    }

    fn is_white_space(&self, ch: c_char) -> bool {
        ch == b' ' as c_char || ch == b'\r' as c_char || ch == b'\n' as c_char || ch == b'\t' as c_char
    }

    fn escape_character(&mut self) -> c_char {
        self.advance_char();
        let ch = self.current_char();

        match ch as u8 {
            b'\"' | b'\\' | b'/' => ch,
            b'b' => b'\x08' as c_char, // backspace
            b'f' => b'\x0C' as c_char, // form feed
            b'n' => b'\n' as c_char,
            b'r' => b'\r' as c_char,
            b't' => b'\t' as c_char,
            b'u' => {
                // Unicode escape sequence - правильная реализация
                let mut hex_chars = [0u8; 4];
                for i in 0..4 {
                    self.advance_char();
                    hex_chars[i] = self.current_char() as u8;
                }
                // Правильно парсим hex строку в число
                let hex_str = std::str::from_utf8(&hex_chars).unwrap_or("0000");
                let hex_value = u32::from_str_radix(hex_str, 16).unwrap_or(0);
                (hex_value & 0xFF) as c_char
            },
            _ => 0,
        }
    }

    fn get_value_content(&mut self, object: *mut JsonObjectParser) -> i32 {
        unsafe {
            if self.current_char() == b'\"' as c_char {
                // value is string
                (*object).value_type = ValueType::ValueTypeString;
                return self.copy_string(&mut (*object).string, MAX_STRING_LENGTH);
            } else if self.current_char() == b'-' as c_char || 
                     (self.current_char() >= b'0' as c_char && self.current_char() <= b'9' as c_char) {
                // value is number
                return self.get_number(object);
            } else if self.current_char() == b't' as c_char || self.current_char() == b'T' as c_char {
                (*object).value_type = ValueType::ValueTypeTrue;
            } else if self.current_char() == b'f' as c_char || self.current_char() == b'F' as c_char {
                (*object).value_type = ValueType::ValueTypeFalse;
            } else if self.current_char() == b'n' as c_char || self.current_char() == b'N' as c_char {
                (*object).value_type = ValueType::ValueTypeNull;
            }
            
            (*object).p_object_content = ptr::null_mut();

            while self.current_char() != 0 && 
                  self.current_char() != b',' as c_char && 
                  !self.is_white_space(self.current_char()) {
                self.advance_char();
            }
            
            // Move back one position
            if self.reader.is_some() {
                if self.line_pos > 0 {
                    self.line_pos -= 1;
                }
            } else if !self.p.is_null() {
                self.p = self.p.offset(-1);
            }
        }
        0
    }

    fn get_number(&mut self, object: *mut JsonObjectParser) -> i32 {
        let mut section = 0;
        let mut exp = 0i32;
        let mut sign = false;
        let mut exp_sign = false;
        let mut finish = false;
        let mut fraction = 0.1f64;

        unsafe {
            (*object).value_type = ValueType::ValueTypeIntNumber;
            (*object).number.l_data = 0;
            
            if self.current_char() == b'-' as c_char {
                sign = true;
                self.advance_char();
            }

            while self.current_char() != 0 {
                match section {
                    0 => { // integer part
                        if self.current_char() >= b'0' as c_char && self.current_char() <= b'9' as c_char {
                            (*object).number.l_data = (*object).number.l_data * 10 + 
                                                     (self.current_char() - b'0' as c_char) as i64;
                        } else if self.current_char() == b'.' as c_char || 
                                 self.current_char() == b'e' as c_char || 
                                 self.current_char() == b'E' as c_char {
                            (*object).number.d_data = (*object).number.l_data as f64;
                            (*object).value_type = ValueType::ValueTypeFloatNumber;
                            section = if self.current_char() == b'.' as c_char { 1 } else { 2 };
                        } else {
                            finish = true;
                        }
                    },
                    1 => { // fraction part
                        if self.current_char() >= b'0' as c_char && self.current_char() <= b'9' as c_char {
                            (*object).number.d_data += fraction * (self.current_char() - b'0' as c_char) as f64;
                            fraction *= 0.1;
                        } else if self.current_char() == b'e' as c_char || self.current_char() == b'E' as c_char {
                            section = 2;
                        } else {
                            finish = true;
                        }
                    },
                    2 => { // exponent part
                        if self.current_char() == b'+' as c_char || self.current_char() == b'-' as c_char {
                            exp_sign = self.current_char() == b'-' as c_char;
                        } else if self.current_char() >= b'0' as c_char && self.current_char() <= b'9' as c_char {
                            exp = exp * 10 + (self.current_char() - b'0' as c_char) as i32;
                        } else {
                            finish = true;
                        }
                    },
                    _ => {}
                }
                
                if finish {
                    // Move back one position
                    if self.reader.is_some() {
                        if self.line_pos > 0 {
                            self.line_pos -= 1;
                        }
                    } else if !self.p.is_null() {
                        self.p = self.p.offset(-1);
                    }
                    break;
                }
                self.advance_char();
            }

            if (*object).value_type == ValueType::ValueTypeFloatNumber {
                if exp_sign {
                    exp = -exp;
                }
                (*object).number.d_data *= 10.0f64.powi(exp);
                if sign {
                    (*object).number.d_data = -(*object).number.d_data;
                }
            } else if sign {
                // negative integer
                (*object).number.l_data = -(*object).number.l_data;
            }
        }
        0
    }

    fn output_object(&self, fp: &mut File, object: *mut JsonObjectParser, depth: i32, has_key: bool) -> i32 {
        if has_key {
            let _ = writeln!(fp, "{{");
        } else {
            let _ = writeln!(fp, "[");
        }

        let mut current_object = object;
        while !current_object.is_null() {
            self.output_key_value(fp, current_object, depth, has_key);
            unsafe {
                current_object = (*current_object).p_next_object;
                if !current_object.is_null() {
                    let _ = writeln!(fp, ",");
                }
            }
        }

        let _ = writeln!(fp);
        for _ in 0..(depth - 1) {
            let _ = write!(fp, "\t");
        }
        let _ = write!(fp, "{}", if has_key { '}' } else { ']' });
        0
    }

    fn output_key_value(&self, fp: &mut File, object: *mut JsonObjectParser, depth: i32, has_key: bool) -> i32 {
        if object.is_null() {
            return -1;
        }

        for _ in 0..depth {
            let _ = write!(fp, "\t");
        }

        unsafe {
            if has_key {
                self.output_string(fp, (*object).key.as_ptr());
                let _ = write!(fp, ": ");
            }

            match (*object).value_type {
                ValueType::ValueTypeObject => {
                    self.output_object(fp, (*object).p_object_content, depth + 1, true);
                },
                ValueType::ValueTypeArray => {
                    self.output_object(fp, (*object).p_object_content, depth + 1, false);
                },
                ValueType::ValueTypeString => {
                    self.output_string(fp, (*object).string.as_ptr());
                },
                ValueType::ValueTypeIntNumber => {
                    let _ = write!(fp, "{}", (*object).number.l_data);
                },
                ValueType::ValueTypeFloatNumber => {
                    let _ = write!(fp, "{}", (*object).number.d_data);
                },
                ValueType::ValueTypeTrue => {
                    let _ = write!(fp, "true");
                },
                ValueType::ValueTypeFalse => {
                    let _ = write!(fp, "false");
                },
                ValueType::ValueTypeNull => {
                    let _ = write!(fp, "null");
                },
            }
        }
        0
    }

    fn output_string(&self, fp: &mut File, str_ptr: *const c_char) -> i32 {
        let _ = write!(fp, "\"");
        
        unsafe {
            let mut ptr = str_ptr;
            while *ptr != 0 {
                match *ptr as u8 {
                    b'\x08' => { let _ = write!(fp, "\\b"); }, // backspace
                    b'\t' => { let _ = write!(fp, "\\t"); },
                    b'\n' => { let _ = write!(fp, "\\n"); },
                    b'\x0C' => { let _ = write!(fp, "\\f"); }, // form feed
                    b'\r' => { let _ = write!(fp, "\\r"); },
                    b'\"' => { let _ = write!(fp, "\\\""); },
                    b'/' => { let _ = write!(fp, "\\/"); },
                    b'\\' => { let _ = write!(fp, "\\\\"); },
                    c if !(0x20..0x7f).contains(&c) => {
                        let _ = write!(fp, "\\u{:04x}", c as u32);
                    },
                    c => {
                        let _ = write!(fp, "{}", c as char);
                    }
                }
                ptr = ptr.offset(1);
            }
        }
        
        let _ = write!(fp, "\"");
        0
    }

    pub fn get_root_object(&self) -> *mut JsonObjectParser {
        self.root_object
    }

    pub fn get_first_object(cur_object: *mut JsonObjectParser) -> *mut JsonObjectParser {
        if cur_object.is_null() {
            ptr::null_mut()
        } else {
            unsafe { (*cur_object).p_object_content }
        }
    }

    pub fn get_next_object(cur_object: *mut JsonObjectParser) -> *mut JsonObjectParser {
        if cur_object.is_null() {
            ptr::null_mut()
        } else {
            unsafe { (*cur_object).p_next_object }
        }
    }
}

impl Drop for JsonStream {
    fn drop(&mut self) {
        if !self.root_object.is_null() {
            self.delete_tree(self.root_object);
        }
    }
}