// Тестовый файл для проверки работы rust-analyzer

use std::collections::HashMap;

pub struct TestStruct {
    pub name: String,
    pub value: i32,
}

impl TestStruct {
    pub fn new(name: String, value: i32) -> Self {
        Self { name, value }
    }
    
    pub fn get_name(&self) -> &str {
        &self.name
    }
    
    pub fn set_value(&mut self, new_value: i32) {
        self.value = new_value;
    }
}

pub fn test_function() -> Result<TestStruct, String> {
    let mut map = HashMap::new();
    map.insert("test", 42);
    
    if let Some(value) = map.get("test") {
        Ok(TestStruct::new("test".to_string(), *value))
    } else {
        Err("Value not found".to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_struct_creation() {
        let test = TestStruct::new("hello".to_string(), 100);
        assert_eq!(test.get_name(), "hello");
        assert_eq!(test.value, 100);
    }
}