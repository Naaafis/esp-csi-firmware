# esp-csi-firmware

Firmware for the ESP32-S3 nodes in the WiFi-CSI 3D pose-estimation demo. One **TX** node broadcasts ESP-NOW packets continuously; three **RX** nodes capture the Channel State Information (CSI) the TX provokes and stream raw CSI frames over UART/USB to the host. The host (a separate repo, `csi-pointcloud-host`) aggregates the three RX streams, preprocesses, and runs the `csi2pointcloud` transformer to produce a 3D point cloud of a human in real time.

Built on [`esp-csi-rs`](https://crates.io/crates/esp-csi-rs) 0.6.0 — a Rust `no-std` abstraction over the Espressif WiFi driver's CSI hooks. We use it instead of the C ESP-IDF reference (`esp-csi`) because the rest of the toolchain is Rust and `esp-csi-rs` already exposes a clean callback for inline CSI processing.

---

## Prerequisites

1. **Rust + esp toolchain.** This is Xtensa, not RISC-V. Install via [`espup`](https://github.com/esp-rs/espup):
   ```bash
   cargo install espup
   espup install
   # source the env each new shell — espup prints the exact path
   . $HOME/export-esp.sh
   ```
   `rust-toolchain.toml` in each crate pins `channel = "esp"`, so `cargo` will pick the right toolchain automatically once `espup` has run.

2. **espflash.** Used as the runner for `cargo run`.
   ```bash
   cargo install espflash
   ```

3. **Xtensa target.** Installed by `espup`. Verified target: `xtensa-esp32s3-none-elf`.

4. **udev / port access.** On macOS the ESP32-S3 enumerates as `/dev/cu.usbmodem*`. Plug it in and run `espflash board-info` once to confirm.

---

## Repo layout

```
esp-csi-firmware/
├── tx/                  # TX firmware — Central node, broadcasts ESP-NOW at 10 Hz
│   ├── .cargo/config.toml   # cargo aliases + linker flags + build-std
│   ├── Cargo.toml
│   ├── rust-toolchain.toml  # channel = "esp"
│   └── src/
│       ├── main.rs      # esp_rtos::main entry, CSINode init, run loop
│       └── lib.rs       # empty (binary crate)
├── rx/                  # RX firmware — Peripheral node + CSI callback + UART framer
│                          (Phase 1 Task 1.2 — currently empty)
└── README.md            # this file
```

Each subdirectory is its own Cargo project so the TX and RX images stay independent.

---

## Build & flash (verified commands)

The relevant aliases live in `tx/.cargo/config.toml`:

```toml
[alias]
esp32s3       = "run   --release --features=esp32s3 --target=xtensa-esp32s3-none-elf"
esp32s3-build = "build --release --features=esp32s3 --target=xtensa-esp32s3-none-elf"
```

`[target.'cfg(target_arch = "xtensa")']` sets `runner = "espflash flash --monitor"`, so `cargo esp32s3` flashes and opens a serial monitor in one step.

From inside `tx/` (and later `rx/`):

| Goal | Command |
|---|---|
| Compile (no flash) | `cargo esp32s3-build` |
| Flash + monitor (115200 baud) | `cargo esp32s3` |
| Flash a specific port | `cargo esp32s3 -- --port /dev/cu.usbmodem101` |
| Just monitor an already-flashed device | `espflash monitor --port /dev/cu.usbmodemXXX` |

If a build fails with "linker `rust-lld` not found" or similar, the most likely cause is the `esp` toolchain isn't active in this shell — re-source `$HOME/export-esp.sh`.

### Dependency pins (verified to compile together)

| Crate | Version |
|---|---|
| `esp-csi-rs` | `0.6.0` (features: `no-std`, `println`, `auto`, `esp32s3`) |
| `esp-hal` | `~1.1` (features: `esp32s3`, `rt`, `unstable`) |
| `esp-radio` | `0.18.0` (`esp32s3`, `wifi`, `sniffer`, `esp-alloc`, `unstable`, `esp-now`, `csi`) |
| `esp-rtos` | `0.3.0` (`esp32s3`, `embassy`, `esp-alloc`, `esp-radio`) |
| `esp-bootloader-esp-idf` | `0.5` (`esp32s3`) |
| `esp-backtrace` | `0.18` (`panic-handler`) |
| `esp-println` | `0.18` |
| `embassy-executor` | `0.10.0` (`task-arena-size-65536`) |
| `embassy-time` | `0.5.0` |
| Rust edition | `2024` |

Don't drift these pins without re-verifying — `esp-radio`'s API moved between 0.17 and 0.18 (`ClientConfig` → `StationConfig`, `AuthMethod` → `AuthenticationMethod`).

---

## Phase 1 — MVP (1 TX + 1 RX)

The goal is a single TX–RX pair producing real CSI frames over USB at ~10 Hz. The host can then visualize the raw CSI (no model inference yet — see "Phase boundaries" in `csi-pointcloud-host/README.md`).

### Task 1.1 — TX Central firmware **[DONE]**

- **Files:** `tx/Cargo.toml`, `tx/.cargo/config.toml`, `tx/rust-toolchain.toml`, `tx/src/main.rs`, `tx/src/lib.rs`
- **What it does:** Initializes esp-hal, brings up the WiFi controller and `esp-rtos`/embassy, then constructs a `CSINode`:
  ```rust
  CSINode::new(
      esp_csi_rs::Node::Central(
          esp_csi_rs::CentralOpMode::EspNow(EspNowConfig::default())
      ),
      CollectionMode::Listener,    // TX does not process its own CSI
      Some(CsiConfig::default()),
      Some(10),                    // traffic_freq_hz = 10 Hz
      csi_hardware,
  );
  ```
  PHY is fixed to `Protocol::N` and `WifiPhyRate::RateMcs0Lgi` for repeatable CSI. A 1 Hz `stats_task` prints `get_pps_tx()` and `get_total_tx_packets()` so you can confirm the radio is actually transmitting.
- **Reference:** mirrors `csi-rag-docs/esp-csi-rs/examples/esp_now_central.rs` (public API used: `CSINode::new`, `set_protocol`, `set_rate`, `node.run()`, `get_pps_tx`, `get_total_tx_packets`, the `log_ln!` macro).
- **Build/flash:** `cd tx && cargo esp32s3`
- **Acceptance:** monitor shows `TX PPS: 10` (±1) within ~3 s of boot. Total packets climbs monotonically.

### Task 1.2 — RX Peripheral firmware **[TODO]**

Goal: capture every CSI packet the TX provokes, frame it, and write it out the serial port so the host can decode it.

**Step 1 — Scaffold the crate.** Mirror `tx/` exactly:
- Copy `tx/.cargo/config.toml`, `tx/rust-toolchain.toml` verbatim into `rx/`.
- Copy `tx/Cargo.toml`, rename the package to `rx`. Same dependencies, same versions.
- Create `rx/src/main.rs` and `rx/src/lib.rs` (empty stub).

**Step 2 — Bring up the Peripheral CSINode.** Pattern from `csi-rag-docs/esp-csi-rs/examples/esp_now_peripheral.rs`:
```rust
let mut node = CSINode::new(
    esp_csi_rs::Node::Peripheral(
        esp_csi_rs::PeripheralOpMode::EspNow(EspNowConfig::default())
    ),
    CollectionMode::Listener,        // we'll collect via the callback, not the built-in collector
    Some(CsiConfig::default()),
    Some(10),                        // match TX traffic rate
    csi_hardware,
);
node.set_protocol(esp_radio::wifi::Protocol::N);
node.set_rate(esp_radio::esp_now::WifiPhyRate::RateMcs0Lgi);
```

**Step 3 — Register the inline CSI callback.** `set_csi_callback(on_csi)` registers a `fn(&CSIDataPacket)` that fires on the WiFi task. The `CSIDataPacket` struct (defined at `csi-rag-docs/esp-csi-rs/src/lib/csi.rs:98`) carries `mac`, `rssi`, `timestamp`, `sequence_number: u16`, `csi_data_len: u16`, and `csi_data: heapless::Vec<i8, 612>`. **The callback runs in the WiFi hot path — no heap, no locking, no blocking serial I/O.** Copy the bytes you need into a static SPSC queue and return.

**Step 4 — Choose the delivery path.** `esp-csi-rs` exposes two paths (see `csi-rag-docs/esp-csi-rs/examples/csi_callback_test.rs`):
- `CsiDeliveryMode::Callback` — inline, lowest latency, must be cheap.
- `CsiDeliveryMode::Async` — `client.next_csi_packet().await` on a user task; pays a ~640 B memcpy per packet but lets you do real work (UART writes, framing) off the WiFi task.

Recommended for the RX framer: **`Async`**. The inline callback does almost nothing; an embassy task dequeues with `CSINodeClient::next_csi_packet().await`, frames, and writes to UART. This keeps the WiFi callback wait-free while letting the framer block on UART backpressure.

**Step 5 — Define the UART frame format.** Fixed, little-endian, designed to be self-synchronizing in case the host opens the port mid-stream:

| Bytes | Field | Type | Notes |
|---|---|---|---|
| 0..4 | magic | `u32` | `0xC51F_E5DA` constant ("CSI FEDA" mnemonic) |
| 4..5 | node_id | `u8` | 0 for the single Phase 1 RX; 0/1/2 in Phase 2 |
| 5..7 | sequence_number | `u16` LE | copied straight from `CSIDataPacket.sequence_number` |
| 7..8 | rssi | `i8` | clamped from `i32` rssi |
| 8..12 | timestamp_us | `u32` LE | from `CSIDataPacket.timestamp` |
| 12..14 | csi_len | `u16` LE | number of `i8` samples in payload |
| 14..(14+csi_len) | csi_data | `i8[csi_len]` | raw, untouched bytes from `csi_data` |
| last 1 | xor_checksum | `u8` | XOR of all preceding bytes (cheap integrity check) |

Total frame size: `15 + csi_len` bytes. At a typical HT40 CSI payload of ~384 bytes that's ~400 bytes per frame; at 10 Hz that's 4 kB/s — well inside the 115200-baud limit (~11.5 kB/s) but Phase 2 with 3 RX × 4 kB/s = 12 kB/s pushes against it. **Bump the baud to 921600** for Phase 2 (set `MONITOR_BAUD` in `.cargo/config.toml` and add `--baud 921600` to `espflash`).

**Step 6 — UART writer task.** Use `esp-hal` UART on the same pins espflash uses (the USB-serial bridge). Allocate a static `[u8; 1024]` scratch buffer, format the frame in place, then `uart.write_all(&buf[..n])`. Don't `println!` — that goes through the logging system and competes with the CSI dump path.

**Step 7 — Stats task.** Mirror the TX `stats_task`: every 1 s log `get_pps_rx()`, `get_total_rx_packets()`, `get_dropped_packets_rx()`, plus a local atomic counter of frames written to UART. Drift between the two indicates the UART can't keep up.

**Build/flash:** `cd rx && cargo esp32s3`

**Acceptance:**
1. Monitor shows `RX PPS` matching TX PPS (10) within 1 packet/s.
2. `get_dropped_packets_rx()` stays at 0 over a 60 s run.
3. On the host, `cat /dev/cu.usbmodemXXX` (after detaching the espflash monitor) shows a periodic byte pattern starting with `DA E5 1F C5` (little-endian magic) at ~10 Hz.
4. A throwaway Python script that opens the port, finds the magic, and prints `sequence_number` shows a strictly increasing counter (modulo `u16` wrap).

---

## Phase 2 — Scale to 3 RX

### Task 2.1 — Node ID byte in the frame header

Already provisioned (byte 4 of the Phase 1 frame, see Task 1.2 Step 5). At flash time, set `node_id` via either:
- A `const NODE_ID: u8 = 0;` flipped per build, or
- Reading the device's MAC at boot and hashing the low byte into 0/1/2.

Recommend option 1: simpler, no surprises, and matches espflash's per-port flow.

### Task 2.2 — Per-unit flashing workflow

espflash picks the first USB serial port it sees, which is fragile with 3 boards plugged in. Two options:

- **Explicit `--port` per board.** Maintain a `scripts/flash_rx.sh <node_id> <port>` that:
  1. Edits `rx/src/main.rs` (or reads from an env var via a `build.rs`) to set `NODE_ID = $1`.
  2. Runs `cargo esp32s3 -- --port $2`.
  3. Restores `NODE_ID` to 0.
- **`SerialPort`-by-VID/PID with USB-serial number lookup.** Each ESP32-S3 has a unique USB serial number; `espflash board-info --port /dev/cu.usbmodemXXX` prints it. Build a small `~/.csi-firmware/ports.toml` mapping serial → node_id, plus a wrapper script. More setup, but no source edits.

Option 1 is fine for Phase 2; switch to option 2 only if flashing becomes a pain point.

### Task 2.3 — Frame synchronization across 3 RX

The host needs to assemble a `[3, 114, 2, 10]` model input where the "3" dimension's three slices are *the same TX packet seen by RX 0, 1, 2*. Two design options:

**Option A — Align by TX sequence number (recommended).**
Every CSI packet a Peripheral captures has a `sequence_number: u16` that is the sequence number of the *ESP-NOW packet from the TX* that triggered the capture. All three RX nodes see the same TX packet, so all three frames for the same physical packet share the same `sequence_number`. The host maintains a small per-RX buffer keyed by `sequence_number` and emits a triple as soon as all three RX have reported the same seq. Wrap at `u16` is handled by treating sequence as modulo-65536 and using a sliding window.

Pros: zero extra firmware work, exact alignment.
Cons: if one RX consistently misses packets, that seq never completes and times out — host needs a timeout policy (drop after 200 ms).

**Option B — Host-side timestamp alignment.**
Tag each UART frame with a host-side arrival timestamp; cluster frames within a ±10 ms window. Lossier and more complex than Option A.

**Decision:** go with Option A. The `sequence_number` field is already in `CSIDataPacket` and already in the Phase 1 frame format.

---

## Known risks / open questions

- **UART bandwidth at Phase 2.** 3 RX × ~400 B × 10 Hz = 12 kB/s — over 115200's 11.5 kB/s ceiling once overhead is counted. Plan: bump to 921600 baud in Phase 2.
- **`sequence_number` wrap.** `u16` wraps every ~6500 s at 10 Hz. The host's sliding-window aligner must handle wrap; don't rely on monotonicity.
- **RX clock sync.** Three independent ESP32-S3s have ~20 ppm crystals. Their `timestamp` (microseconds) fields drift relative to each other — don't use them for cross-RX alignment, use `sequence_number`.
- **CSI payload size variability.** `csi_data_len` depends on the captured packet's bandwidth/format (`bandwidth` field, `data_format`). HT20 is shorter than HT40. The host preprocessor must read `csi_len` from the frame header rather than assuming a fixed length.
- **Callback hot-path discipline.** Anything that allocates or locks inside `on_csi` will stall the WiFi task and start dropping packets. The `Async` delivery mode exists precisely so the framer can take its time without that risk.
- **`set_csi_logging_enabled(false)`.** The default `init_logger` enables a per-packet UART dump (`LogMode::Text`) that will collide with our binary frame stream. The RX firmware must call `set_csi_logging_enabled(false)` after `init_logger` to suppress it, matching what `csi_callback_test.rs` does.
- **No `defmt`.** We use the `println` logging backend; switching to `defmt` would change the linker script (`-Tdefmt.x`) and break the stats line decode on the host monitor — don't change without reason.
