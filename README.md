![ESP32 PX4 MoCap Bridge](media/cover.png)

---

# ESP32 PX4 MoCap Bridge

![Rust](https://img.shields.io/badge/Rust-stable-orange?logo=rust)
![Embassy](https://img.shields.io/badge/async-Embassy-blue)
![PX4](https://img.shields.io/badge/PX4-Autopilot-0088cc)
![MAVLink](https://img.shields.io/badge/protocol-MAVLink%20v2-brightgreen)
![License](https://img.shields.io/badge/license-BSD--3--Clause-green)

Bare-metal Rust firmware for an ESP32 that streams mocap rigid body poses to a flight controller over MAVLink.

The bridge receives motion capture rigid body poses over WiFi, transforms them appropriately, and streams them over UART as MAVLink `VISION_POSITION_ESTIMATE` messages for external-vision fusion in the flight controller. This gives you a plug-and-play module, with no major electrical or mechanical changes to your airframe without the need for a full companion computer on the drone just to forward poses.

Everything is configured at runtime through a setup portal served by the ESP32: no recompiling, no hardcoded credentials.

You can watch a working example here with a Pixhawk 6X on a X500 quadcopter:

https://github.com/user-attachments/assets/ee866f0b-1d2e-4d50-8783-3c1a3e9491da

## Features

- **Bare-metal Rust** on the ESP32-C6 (`no_std`, [esp-hal](https://github.com/esp-rs/esp-hal) + [Embassy](https://embassy.dev/) async)
- **Tracking-loss safety** - stops streaming when the rigid body goes stale so EKF2 coasts instead of fusing garbage, and increments the MAVLink `reset_counter` on reacquisition
- **OptiTrack and Qualisys** - receives NatNet multicast from Motive or the QTM real-time protocol from Qualisys Track Manager
- **WiFi setup portal** - setup portal with a web form for WiFi credentials, mocap source, and MAVLink settings, persisted to flash
- **Factory reset** - hold the BOOT button for 3 seconds to wipe settings and return to the portal

## How it works
<!-- PLACEHOLDER: system/architecture diagram (mocap cameras, Motive PC, WiFi router, ESP32, FC) -->

The firmware boots into one of two modes:

- **Bridge mode** - when valid settings exist in flash: joins your WiFi as a client, subscribes to the NatNet multicast group (OptiTrack) or opens a QTM real-time session (Qualisys), and streams `VISION_POSITION_ESTIMATE` (default 100 Hz) plus `HEARTBEAT` (1 Hz) over UART.
- **Portal mode** - on first boot or after a factory reset: starts a WiFi access point with a setup portal through a web browser.

## Getting started

### Prerequisites

Hardware:

- An **ESP32-C6** development board (any board with USB flashing works)
- A **PX4 flight controller** with a free UART/telemetry port (e.g. `TELEM2`)
- 4 x **jumper wires** (TX, RX, GND, 5V) both sides are 3.3 V logic, no level shifter needed
- An **OptiTrack** system running **Motive**, or a **Qualisys** system running **QTM**, with a tracked rigid body
- A **2.4 GHz WiFi network** on the same LAN as the mocap PC

Software:

- [Rust](https://rustup.rs/) (stable, `riscv32imac` target, and `rust-src` are installed automatically via `rust-toolchain.toml`)
- [espflash](https://github.com/esp-rs/espflash)

### Installation

1. Clone the repository:

   ```sh
   git clone https://github.com/kousheekc/esp32_px4_mocap.git
   cd esp32_px4_mocap
   ```

2. Connect the ESP32-C6 over USB, then build, flash, and open the serial monitor in one step:

   ```sh
   cd firmware
   cargo run --release
   ```

That's it. On first boot the log will show the firmware entering portal mode.

## Usage

### 1. Configure via the setup portal

On a fresh flash (or after a factory reset) the bridge starts its own WiFi access point:

1. Join the WiFi network **`esp`** (password **`12345678`**) from your phone or laptop.
2. The portal opens automatically (captive portal) or browse to **http://192.168.4.1/**.
3. Fill in the form and hit save. The board reboots straight into bridge mode.

![User setting configuration form](media/webform.png)

| Setting | Default | Notes |
|---|---|---|
| WiFi SSID / password | — | Your 2.4 GHz network, shared with the mocap PC |
| Source | OptiTrack | OptiTrack (NatNet) or Qualisys (QTM) |
| Rigid body | `32` | OptiTrack: the *Streaming ID* in Motive. Qualisys: position in QTM's 6DOF body list (1 = first) |
| Multicast address | `239.255.42.99` | OptiTrack only: Motive's default interface |
| Data port | `1511` | OptiTrack only: Motive's default data port |
| NatNet version | `3.1` | OptiTrack only: match Motive's *Streaming* pane (2.x–4.x supported) |
| QTM PC address | `192.168.1.100` | Qualisys only: IP of the PC running QTM |
| RT port | `22223` | Qualisys only: QTM's little-endian real-time port |
| MAVLink sysid / compid | `1` / `197` | 197 = vision/odometry source |
| UART baud | `921600` | Must match the FC serial port baud |
| VPE rate | `100 Hz` | Qualisys: frame rate requested from QTM. Each received pose is forwarded as soon as it arrives, so the transmit rate follows the incoming stream (OptiTrack: Motive's streaming rate) |
| Heartbeat rate | `1 Hz` | Must divide the VPE rate |
| Tracking timeout | `200 ms` | Stop streaming after no valid pose for this long |

To change settings later, hold the **BOOT** button for **3 seconds**, the settings are erased and the board reboots back into the portal.

### 2. Wire the bridge to the flight controller

| ESP32-C6 | Flight controller |
|---|---|
| GPIO21 (TX) | UART RX (e.g. `TELEM2` RX) |
| GPIO2 (RX) | UART TX (e.g. `TELEM2` TX) |
| GND | GND |
| 5V / USB | 5V |

<!-- PLACEHOLDER: wiring schematic / photo of the ESP32-C6 connected to a flight controller -->

### 3. Set up Motive streaming

In Motive's **Streaming** pane:

1. Set transmission type to **Multicast** with the default address (`239.255.42.99`) and data port (`1511`).
2. Select the network interface that is on the same LAN as your WiFi.
3. Note the **Streaming ID** of your rigid body and enter it in the portal.

Once the bridge connects, the serial monitor logs the incoming packet rate, the number of `VISION_POSITION_ESTIMATE` messages sent, and the latest pose once per second.

### 3b. Set up QTM real-time output (Qualisys)

Use this instead of step 3 when the source is Qualisys:

1. Make sure QTM's real-time server is enabled (*Project Options → Processing → Real-Time Output*) and allowed through the Windows firewall (TCP 22223, and UDP out to the bridge).
2. Define your rigid body under *6DOF Tracking* and note its position in the list, enter that as the rigid body number in the portal.
3. Keep QTM's global coordinate system Z-up (the default); the bridge converts Z-up to PX4's NED frame.
4. Start a measurement (or real-time preview). The bridge connects to QTM over TCP, requests 6DOF frames at the VPE rate over UDP (local port `22230`), and reconnects automatically if QTM restarts.
5. The once-per-second serial log line reports `missed N (frame step S)`: QTM frames that never reached the bridge, judged from gaps in QTM's frame number (`S` is the normal step, capture rate / stream rate). Non-zero `missed` means frames are lost between QTM and the bridge, usually on the WiFi link.

### 4. PX4 setup

On the PX4 side (via QGroundControl parameters):

1. **Enable MAVLink on the serial port** the bridge is wired to, e.g. for `TELEM2`:
   - `MAV_1_CONFIG` = `TELEM 2`
   - `MAV_1_MODE` = `External Vision`
   - `SER_TEL2_BAUD` = `921600`
2. **Configure EKF2 to fuse external vision:**
   - `EKF2_EV_CTRL` = `11` (horizontal position + vertical position + yaw)
   - `EKF2_HGT_REF` = `Vision`
   - For a pure indoor setup, disable GPS fusion: `EKF2_GPS_CTRL` = `0`
   - Optionally tune `EKF2_EV_DELAY` to your mocap + WiFi latency (see *Timestamps* below)
3. **Verify before flying:**
   - In QGroundControl's **MAVLink Inspector**, confirm `VISION_POSITION_ESTIMATE` is arriving at the configured rate.
   - Check `LOCAL_POSITION_NED` follows when you carry the vehicle around the arena, and that yaw matches reality
   - Cover the markers and confirm the bridge logs "tracking lost" and PX4 keeps a sane estimate.

### Timestamps

With Qualisys, each `VISION_POSITION_ESTIMATE` carries the frame's capture time rather than the time it was sent, so WiFi delivering frames late or in bursts doesn't change the timestamp PX4 sees. The bridge maps QTM's frame timestamps onto its own clock using the fastest-arriving recent packet, and answers PX4's `TIMESYNC` requests so PX4 can map the bridge clock onto its own. PX4 needs about 500 timesync exchanges (~50 s after boot) before it trusts the timestamps; until then it stamps messages on arrival, as it would without timesync. `EKF2_EV_DELAY` then only has to cover the fixed part of the latency (QTM processing and the fastest network transit). OptiTrack poses are still stamped on arrival at the bridge.

The QTM serial log line also reports `dup N` (frames dropped because another QTM session was streaming the same frames to the bridge, e.g. one left over from before a reset), `age max` (the most any forwarded frame had aged since capture) and `timesync N` (replies sent to PX4 in the last second, ~10 when PX4's External Vision link is working).

See the PX4 [External Position Estimation](https://docs.px4.io/main/en/ros/external_position_estimation.html) guide for background and tuning advice.

<!-- PLACEHOLDER: QGroundControl screenshot showing VISION_POSITION_ESTIMATE in the MAVLink Inspector / local position tracking -->

## Next steps

Planned extensions, roughly in order:

- [x] **Qualisys support** - QTM real-time protocol receiver alongside NatNet.
- [ ] **ArduPilot support** - ArduPilot accepts the same `VISION_POSITION_ESTIMATE` message, so add the corresponding setup documentation and any protocol tweaks needed for its external-navigation EKF sources.

Contributions toward either are very welcome.

## License

This project is licensed under the BSD 3-Clause License - see the [LICENSE](https://github.com/kousheekc/esp32_px4_mocap/blob/main/LICENSE) file for details.

## Contact

Kousheek Chakraborty - kousheekc@gmail.com

Project Link: [https://github.com/kousheekc/esp32_px4_mocap](https://github.com/kousheekc/esp32_px4_mocap)

If you encounter any difficulties, feel free to reach out through the Issues section. If you find any bugs or have improvements to suggest, don't hesitate to make a pull request.
