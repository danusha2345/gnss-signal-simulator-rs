//! Завершённый файл становится видимым только после проверки размера и sync.
use std::fs::{self, File, OpenOptions};
use std::io::{self, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
static NEXT_ID: AtomicU64 = AtomicU64::new(0);

pub(crate) struct AtomicOutput {
    writer: Option<BufWriter<File>>,
    temporary: PathBuf,
    target: PathBuf,
    committed: bool,
}
impl AtomicOutput {
    pub(crate) fn new(target: &Path) -> io::Result<Self> {
        if target.exists() && !target.is_file() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "Output must be a regular file",
            ));
        }
        let parent = target
            .parent()
            .filter(|p| !p.as_os_str().is_empty())
            .unwrap_or(Path::new("."));
        let mut name = target
            .file_name()
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "Missing output filename"))?
            .to_os_string();
        name.push(format!(
            ".{}.{}.partial",
            std::process::id(),
            NEXT_ID.fetch_add(1, Ordering::Relaxed)
        ));
        let temporary = parent.join(name);
        let file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&temporary)?;
        Ok(Self {
            writer: Some(BufWriter::with_capacity(1 << 20, file)),
            temporary,
            target: target.to_path_buf(),
            committed: false,
        })
    }
    pub(crate) fn writer(&mut self) -> &mut BufWriter<File> {
        self.writer.as_mut().expect("output is not yet committed")
    }
    pub(crate) fn commit(mut self, expected_bytes: u64) -> io::Result<()> {
        let mut writer = self.writer.take().expect("output is not yet committed");
        writer.flush()?;
        if writer.get_ref().metadata()?.len() != expected_bytes {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "Generated file size mismatch",
            ));
        }
        writer.get_ref().sync_all()?;
        drop(writer);
        fs::rename(&self.temporary, &self.target)?;
        self.committed = true;
        // После rename файл уже заменён; ошибка sync каталога означает неопределённую
        // долговечность публикации, а не сохранность старой версии.
        #[cfg(unix)]
        File::open(
            self.target
                .parent()
                .filter(|p| !p.as_os_str().is_empty())
                .unwrap_or(Path::new(".")),
        )?
        .sync_all()?;
        Ok(())
    }
}
impl Drop for AtomicOutput {
    fn drop(&mut self) {
        drop(self.writer.take());
        if !self.committed {
            let _ = fs::remove_file(&self.temporary);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn failed_generation_preserves_existing_file_and_cleans_partial() {
        let dir = PathBuf::from(format!(".scratch/atomic_output_{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let target = dir.join("test.C8");
        fs::write(&target, b"old").unwrap();
        {
            let mut output = AtomicOutput::new(&target).unwrap();
            output.writer().write_all(b"partial").unwrap();
        }
        assert_eq!(fs::read(&target).unwrap(), b"old");
        let mut output = AtomicOutput::new(&target).unwrap();
        output.writer().write_all(b"new").unwrap();
        assert!(output.commit(4).is_err());
        assert_eq!(fs::read(&target).unwrap(), b"old");
        assert_eq!(fs::read_dir(&dir).unwrap().count(), 1);
        let mut output = AtomicOutput::new(&target).unwrap();
        output.writer().write_all(b"new").unwrap();
        output.commit(3).unwrap();
        assert_eq!(fs::read(&target).unwrap(), b"new");
        assert_eq!(fs::read_dir(&dir).unwrap().count(), 1);
        fs::remove_dir_all(dir).unwrap();
    }
}
