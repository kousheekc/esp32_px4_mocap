//! Settings persistence (thanks Claude)

use esp_bootloader_esp_idf::partitions::{
    self, DataPartitionSubType, PartitionType, PARTITION_TABLE_MAX_LEN,
};
use esp_hal::peripherals::FLASH;
use esp_storage::FlashStorage;
use log::{error, info};
use settings::{Settings, WIRE_LEN};

pub struct FlashStore {
    flash: FlashStorage<'static>,
    nvs_offset: Option<u32>,
}

impl FlashStore {
    pub fn new(flash: FLASH<'static>) -> Self {
        let mut flash = FlashStorage::new(flash);

        let mut scratch = alloc::vec![0u8; PARTITION_TABLE_MAX_LEN];
        let nvs_offset = match partitions::read_partition_table(&mut flash, &mut scratch) {
            Ok(table) => match table.find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
            {
                Ok(Some(nvs)) => {
                    info!("store: nvs partition at {:#x} ({} B)", nvs.offset(), nvs.len());
                    Some(nvs.offset())
                }
                _ => {
                    error!("store: no nvs partition in the partition table");
                    None
                }
            },
            Err(e) => {
                error!("store: cannot read partition table ({e:?})");
                None
            }
        };

        Self { flash, nvs_offset }
    }

    pub fn load(&mut self) -> Option<Settings> {
        let offset = self.nvs_offset?;
        let mut buf = [0u8; WIRE_LEN];
        embedded_storage::ReadStorage::read(&mut self.flash, offset, &mut buf).ok()?;
        Settings::from_bytes(&buf)
    }

    pub fn save(&mut self, s: &Settings) -> Result<(), &'static str> {
        let offset = self.nvs_offset.ok_or("no nvs partition found")?;
        let mut buf = [0u8; WIRE_LEN];
        s.to_bytes(&mut buf);
        embedded_storage::Storage::write(&mut self.flash, offset, &buf)
            .map_err(|_| "flash write failed")
    }

    pub fn erase(&mut self) {
        if let Some(offset) = self.nvs_offset {
            let _ = embedded_storage::Storage::write(&mut self.flash, offset, &[0u8; 4]);
        }
    }
}
