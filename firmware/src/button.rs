//! Factory reset hold the BOOT button for 3 seconds to reset the settings

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use esp_hal::gpio::Input;
use log::{info, warn};

use crate::config;
use crate::store::FlashStore;

#[embassy_executor::task]
pub async fn factory_reset_task(
    mut button: Input<'static>,
    store: &'static Mutex<CriticalSectionRawMutex, FlashStore>,
) -> ! {
    loop {
        button.wait_for_low().await;
        let pressed = Instant::now();

        loop {
            Timer::after(Duration::from_millis(50)).await;
            if button.is_high() {
                info!("button: released after {} ms", pressed.elapsed().as_millis());
                break;
            }
            if pressed.elapsed() >= Duration::from_millis(config::RESET_HOLD_MS) {
                warn!("button: BOOT held {} ms — factory reset", config::RESET_HOLD_MS);
                store.lock().await.erase();
                esp_hal::system::software_reset();
            }
        }
    }
}
