# Detailed Changes — esp-csi-cli-rs v0.5.0

Complete, file-by-file record of every change in this branch. The headline is
the upgrade from `esp-csi-rs` 0.6.1 (pinned git rev) to crates.io **0.7.0**,
plus full integration of the new 0.7.0 capabilities, the breaking-change fixes
that upgrade requires, two pre-existing build fixes uncovered during
verification, and logging documentation.

Diff summary (vs base):

```
 .gitignore        |   5 ++-
 Cargo.lock        |  37 +++---
 Cargo.toml        |  37 +++++--
 README.md         |  33 ++++--
 build.rs          |  26 +++++-
 src/cli/cmds.rs   | 115 ++++++++++++++++++++--
 src/cli/mod.rs    |  46 +++++++--
 src/cli/serial.rs |   3 ++
 src/config.rs     |  27 +++++
 src/main.rs       |  75 +++++++++++---
 10 files changed, 336 insertions(+), 68 deletions(-)
```

---

## 1. `Cargo.toml`

### 1.1 Package version
- `version = "0.4.2"` → **`"0.5.0"`**. Minor bump: new CLI commands/options, a
  changed `show-stats` output, and changed logging-feature semantics.

### 1.2 Core dependency upgrade
- `esp-csi-rs`: `{ git = "…", rev = "d17ba572…", … }` (resolved to **0.6.1**) →
  **`{ version = "0.7.0", default-features = false }`** (published on crates.io;
  verified `max_version = 0.7.0`). `Cargo.lock` now sources it from
  `registry+https://github.com/rust-lang/crates.io-index`.

### 1.3 Logging feature wiring + documentation
- `jtag-serial`: `["esp-csi-rs/jtag-serial"]` →
  **`["esp-csi-rs/jtag-serial", "async-print"]`**. Mirrors esp-csi-rs 0.7.0,
  where `jtag-serial` transitively forces `async-print` (the JTAG backend can
  block under load; a blocked log write stalls the collection hot path and drops
  packets). Keeping this crate's own `async-print` flag in sync makes the
  coupling explicit.
- Expanded doc comments on `async-print`, `defmt`, `jtag-serial`, and `uart`:
  - `async-print`: explains non-blocking logging and its throughput role;
    pair with `jtag-serial`, not `uart`.
  - `defmt`: now notes it is **incompatible with `async-print`** (duplicate defmt
    `#[global_logger]` → `_defmt_acquire` multiply defined), so it can't be used
    with `jtag-serial`; use the `*-defmt` aliases (auto) instead.
  - `uart`: do **not** combine with `async-print`.
- `statistics` comment updated: "PPS / latency / drop" → "PPS / rate / drop +
  ESP-NOW TX counters" (latency telemetry was removed upstream).

### 1.4 Other dependency bumps (in this branch)
- `embedded-io-07` (alias for `embedded-io`): `0.7.0` → `0.7.1`.
- `heapless`: `0.7` → `0.9.3`.
- `embassy-time`: `0.5.0` → `0.5.1`.
- `embedded-io` (plain, for `menu` 0.6.x): kept/restored at **`0.6.1`** — see
  §8 (build fix).

---

## 2. `src/config.rs` — `UserConfig` extensions

Added three runtime-config fields (snapshotted by the collection task at each
`start`, so no extra plumbing is needed):

- `peer_mac: Option<[u8; 6]>` — explicit ESP-NOW peer MAC. `Some` enables
  per-node source-MAC filtering (`EspNowConfig::with_peer_mac`); `None` keeps
  automatic magic-prefix pairing.
- `ht40_secondary: Option<SecondaryChannel>` — forced HT40 TX PHY
  (`EspNowConfig::with_ht40`); `None` = HT20/legacy per rate.
- `delivery_raw: bool` — when set, the next run registers the zero-copy raw CSI
  fast-path instead of the full callback.

Also: imported `esp_radio::wifi::SecondaryChannel`; initialized all three fields
to `None`/`false` in `UserConfig::new()`; extended the manual `Debug` impl to
render them (MAC bytes; HT40 as Above/Below/None).

---

## 3. `src/main.rs` — node build, raw fast-path, import gating

### 3.1 New imports
- Added `set_csi_raw_callback` and `set_raw_listen` to the `esp_csi_rs` use list.

### 3.2 `build_espnow_config` helper (new)
- Factored ESP-NOW config construction into one helper that chains
  `with_channel`/`with_phy_rate` and conditionally `with_peer_mac` /
  `with_ht40` from the snapshot. Both `EspNowCentral` and `EspNowPeripheral`
  node arms now call it (replacing the duplicated inline builders).

### 3.3 `raw_csi_noop` callback (new)
- A do-nothing `fn raw_csi_noop() {}` used as the raw fast-path callback;
  documented that no CSI data is delivered/logged and the q-key stop peek is
  intentionally absent (raw runs are duration-bound / reset-driven).

### 3.4 Delivery wiring (in `csi_collection`)
- The run previously always did `set_csi_logging_enabled(false);
  set_csi_callback(csi_log_and_check);`. Now branches on
  `user_config.delivery_raw`:
  - **raw**: `set_raw_listen(true); set_csi_raw_callback(raw_csi_noop);`
    (also skips ESP-NOW control-packet ingest for a like-for-like CPU cost).
  - **normal**: `set_raw_listen(false); set_csi_callback(csi_log_and_check);`.

### 3.5 `is_jtag` import gating (build fix — see §8)
- `use cli::is_jtag;` re-gated from "any chip" to **`all(feature = "auto",
  any(chip…))`** to match the function's definition.

---

## 4. `src/cli/cmds.rs` — command handlers

### 4.1 Imports
- Removed the now-deleted `get_one_way_latency` / `get_two_way_latency` from the
  `statistics` import.
- Added `esp_csi_rs::central::esp_now::{get_tx_queued_packets,
  get_tx_confirmed_packets, get_tx_failed_packets}` (statistics-gated).
- Added `esp_radio::wifi::SecondaryChannel`.

### 4.2 `parse_mac` helper (new)
- Parses `aa:bb:cc:dd:ee:ff` (also `-` separators), case-insensitive, strict
  validation → `Option<[u8; 6]>`.

### 4.3 `set_wifi`
- Parses two new args: `--peer-mac` (empty clears to auto; invalid → error
  message) and `--ht40=above|below|none`. Mutations use the existing
  `USER_CONFIG.lock(...)` pattern.
- Confirmation output now prints the ESP-NOW peer MAC (or "auto") and TX PHY
  (HT20/legacy vs HT40 above/below).

### 4.4 `set_csi_delivery_cmd`
- `--mode` now accepts **`raw`** in addition to `off|callback|async`. `raw` sets
  the `delivery_raw` flag (applies at next `start`); the other three clear it and
  take effect immediately via `set_csi_delivery_mode`. Invalid-mode message
  updated to list `raw`.

### 4.5 `show_config`
- The `[WiFi]` block now prints the configured peer MAC (or "auto") and TX PHY.

### 4.6 `show_stats`
- Removed the two latency lines (functions removed upstream). Added
  **TX Queued / TX Confirmed / TX Failed** packet counters.

---

## 5. `src/cli/mod.rs` — menu definition & help text

- `is_jtag` re-export re-gated with `feature = "auto"` (build fix, §8).
- `set-wifi` item: added `peer-mac` and `ht40` `NamedValue` parameters, plus
  expanded help text with options, an example, and a note that the ESP-NOW
  options only apply in ESP-NOW modes.
- `set-csi-delivery` item: `--mode` help and usage now include `raw`, with a
  full description of the fast-path (applies at next `start`, no CSI delivered,
  no q-key stop, skips ESP-NOW control ingest).
- `show-stats` help: removed the "one-way / two-way latency" line; added the
  ESP-NOW TX queued/confirmed/failed counters.

---

## 6. `src/cli/serial.rs` — import gating (build fix)

- `use esp_hal::uart::Uart;` re-gated to
  **`#[cfg(any(feature = "esp32", feature = "uart", feature = "auto"))]`** — the
  only configurations that actually reference `Uart` (the ESP32 alias, the
  forced-`uart` alias, and the runtime `auto` enum). Prevents an unused-import
  warning under forced `jtag-serial`.

---

## 7. `build.rs` — defmt + async-print guard (new)

- Replaced the empty `fn main() {}` with a guard that emits a readable
  `cargo:warning` when **both** `CARGO_FEATURE_DEFMT` and
  `CARGO_FEATURE_ASYNC_PRINT` are set, explaining the conflict and pointing to
  the optimal setup (`jtag-serial` + `serialized` log mode). Surfaces the issue
  ahead of the otherwise-cryptic `_defmt_acquire multiply defined` link error.
  Silent for all other configurations.

---

## 8. Build fixes (pre-existing issues, surfaced during verification)

These were not caused by the upgrade but blocked it / blocked forced-backend
builds; fixing them was necessary to ship the recommended configurations.

### 8.1 Duplicate `embedded-io`
- The plain `embedded-io` dependency had drifted to `0.7.1`, making it resolve to
  the same crate+version as the `embedded-io-07` alias (`embedded-io 0.7.1`) →
  Cargo error: *"depends on crate `embedded-io v0.7.1` multiple times with
  different names."* Restored the plain dep to **`0.6.1`** (required by `menu`
  0.6.1; `serial.rs` implements `ErrorType`/`Write` for both 0.6.x and the
  0.7.x alias). This matches the rationale already documented in the
  `embedded-io-07` comment.

### 8.2 `is_jtag` feature gating (E0432)
- `is_jtag()` (`serial.rs`) is compiled only under `feature = "auto"`, but its
  re-export (`cli/mod.rs`) and import (`main.rs`) were gated only on the chip
  features. Forced `jtag-serial`/`uart` builds (no `auto`) then failed with
  `unresolved import …is_jtag`. Both sites are now gated on
  `all(feature = "auto", any(chip…))`, matching the definition and its sole call
  site (which is already `auto`-gated). Forced-backend builds now compile.

---

## 9. `README.md` — logging guidance

- Custom-build feature table updated: `async-print` (auto-enabled by
  `jtag-serial`), `jtag-serial` (auto-enables `async-print`), `uart` (do not
  combine with `async-print`), and `statistics` (PPS/rate/drop + TX counters,
  not latency).
- The manual-build example was changed from the now-unbuildable
  `defmt,jtag-serial` combo to a working `println,jtag-serial` example.
- Added a performance call-out: **most optimal setup = `jtag-serial` +
  `async-print` + `serialized` log mode** (`set-log-mode --mode=serialized`),
  plus a clear warning not to pair `defmt` with `async-print`, with the upstream
  root cause and the `*-defmt` aliases as the defmt alternative.

---

## 10. `.gitignore`
- Added `.cachebro/` (local cache directory).

---

## Root cause of the `defmt` + `async-print` conflict (reference)

- `esp-println` registers `#[defmt::global_logger]` only under its
  `defmt-espflash` feature (`esp-println/src/defmt.rs`). Correct.
- `esp-csi-rs` 0.7.0 registers a **second** `#[defmt::global_logger]`
  (`AsyncDefmtBackend`) under `cfg(all(feature = "defmt", feature = "async-print"))`
  (`src/lib/logging/logging.rs:206`), intending to be the sole logger in that
  config — but its `Cargo.toml` `defmt` feature **unconditionally** enables
  `esp-println/defmt-espflash`. With both `defmt` and `async-print` on, both
  loggers compile → `_defmt_acquire` multiply defined. This is an upstream
  esp-csi-rs feature-wiring inconsistency; there is no clean consumer-side
  workaround, hence the "do not combine" guidance and the `build.rs` warning.

---

## Verification

- `cargo build --release` for all five targets (esp32 / c3 / c5 / c6 / s3): pass.
- `cargo check` for Xtensa (esp32) and RISC-V (esp32c6): pass.
- `cargo clippy` (esp32c6): no new warnings vs base.
- Forced `jtag-serial` + `println`, `*-defmt` aliases, and default `auto` build:
  all compile.
- `build.rs` warning fires only on `defmt` + `async-print`.
- **Pending (needs hardware):** on-device smoke test of peer-MAC pairing, HT40
  PHY forcing, raw delivery mode, and the regression paths (sniffer/callback/
  async + q-key stop).
