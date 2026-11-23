# AOG-TaskController — Protocol Reference

This document describes the interfaces AOG-TaskController exposes to the rest of the system. The intended audience is anyone writing a client — AgOpenGPS (AgIO), AgValoniaGPS, or a generic ISOBUS controller — that needs to talk to the TC.

For Linux deployment of the TC itself (systemd, CAN setup, the daemon footprint), see [LINUX_DAEMON.md](LINUX_DAEMON.md). For the TC's threading model — which callbacks run on which thread, and what that means for any state you touch — see [CONCURRENCY.md](CONCURRENCY.md).

---

## 1. Overview

AOG-TaskController sits between two worlds:

```
   AgIO / AgValonia                 ISOBUS bus (CAN @ 250 kbps)
       (LAN/UDP)                    (sprayers, planters, TECU)
            │                                  │
            ▼                                  ▼
    ┌───────────────────────────────────────────────┐
    │           AOG-TaskController                  │
    │                                               │
    │  • UDP framing on subnet:8888 / .255:9999     │
    │  • ISO 11783 TaskController (Section Control) │
    │  • Optional ISO 11783-9 Tractor ECU (TECU)    │
    └───────────────────────────────────────────────┘
```

Three things to understand before reading the rest:

1. **All client traffic is UDP.** The TC speaks an AgIO-style framed UDP protocol on the LAN. There is no TCP, REST, or gRPC interface.
2. **The TC is the AgIO/AgValonia *peer*, not its client.** AgIO/AgValonia opens its sockets; the TC also opens its sockets; they exchange framed packets. Either can come up first.
3. **ISO 11783 is the source of truth for the CAN side.** The TC implements the ISOBUS Task Controller (Section Control, Generation 1, ISO 11783-10) using the [AgIsoStack++](https://github.com/Open-Agriculture/AgIsoStack-plus-plus) library. You do not need to know ISOBUS to *talk to the TC* — only to understand what it does on the bus.

---

## 2. UDP wire protocol

### 2.1 Packet framing

Every packet — both directions — uses the same frame:

```
 Offset  Size  Field        Notes
 0       2     Start         0x80 0x81 (big-endian sentinel)
 2       1     Source        Sender's logical address (see 2.2)
 3       1     PGN           Logical message id (see 2.5/2.6)
 4       1     Length        Number of payload bytes (N)
 5       N     Payload       N bytes, depends on PGN
 5+N     1     Checksum      Sum of bytes [Source .. last payload byte], mod 256
```

Total wire size is `N + 6` bytes. The maximum payload is currently 250 bytes (limited by the TC's 512-byte receive buffer). Most PGNs are ≤10 bytes; the outlier is `0xD6` (GPS/IMU data, see §2.5), which needs at least 39 bytes today.

**Checksum**: the TC currently does **not** validate inbound checksums (the verification code is present but commented out in `udp_connections.cpp`). Clients **should still compute and include a correct checksum** so that future TC versions, or third-party listeners, can validate.

### 2.2 Source addresses

Source byte identifies the logical sender of a frame. The conventions used today:

| Source | Logical sender |
|---|---|
| `0x7C` (124) | **GPS data from AgIO's NMEA parser** — sends PGN `0xD6` only (see §2.5). Not a separate device: AgIO builds this frame from the receiver's NMEA sentences. |
| `0x7F` (127) | **AgIO / AgValonia** (the GUI/host application) |
| `0x80` (128) | **AOG-TaskController** itself |

Other addresses appear on the ISOBUS side but are not used in UDP frames.

### 2.3 Ports and binding

The TC opens two UDP sockets:

| Socket | Bind | Purpose |
|---|---|---|
| Main | `<LAN-IP>:8888` | All operational traffic to/from AgIO/AgValonia |
| Address detection | `0.0.0.0:8888` | Listens for the subnet-detection PGN so AgIO can tell the TC what subnet to use |

Both sockets have `SO_REUSEADDR` set (needed on Linux so the same port can host the specific-IP bind and the wildcard bind simultaneously).

The TC sends to **`<subnet>.255:9999`** as a broadcast. The "subnet" is configured via `settings.json` and can be overridden at runtime by the subnet-detection PGN.

### 2.4 NIC selection

On startup the TC enumerates network interfaces and picks the first IPv4 address whose first three octets match `settings.subnet`:

- **Linux/macOS**: via `getifaddrs(3)`.
- **Windows**: via the hostname-based Boost.Asio resolver.

If no NIC matches, the TC falls back to loopback (`127.0.0.1`) — useful for local testing but means broadcast to LAN won't work.

### 2.5 PGNs inbound (client → TC)

All PGNs sent **by AgIO/AgValonia to the TC** use source `0x7F`, except `0xD6` (GPS/IMU data), which AgIO's NMEA parser sends with source `0x7C`.

| PGN | Name | Length | Payload |
|---|---|---|---|
| `0xC9` (201) | Subnet detection | 5 | `[0xC9, 0xC9, IP0, IP1, IP2]` |
| `0xD6` (214) | GPS/IMU data | variable (≥39 used) | Only byte 38 (fix quality) is parsed today; rest of frame is currently unused. Source `0x7C`. |
| `0xE5` (229) | Section states (64 sections) | 8 | Bitfield: bit `8·j + i` of byte `j` is section `(8j + i)` ON/OFF |
| `0xF1` (241) | Section control mode | 1 | `[mode]` where `1` = enabled, `0` = disabled |
| `0xF2` (242) | Process data | 6 or 7 | `[DDI_lo, DDI_hi, val0, val1, val2, val3]` — DDI is little-endian `uint16`; value is little-endian `int32`. Rate setpoints add `[channel]` (see below) |
| `0xF4` (244) | Rate channel info | 3 + name | `[channel, DDI_lo, DDI_hi, name...]` — from RateController; see "Rate channel info" below |

#### `0xC9` — Subnet detection

Tells the TC which `/24` subnet AgIO/AgValonia lives on. The first two payload bytes are `0xC9 0xC9` (a magic to disambiguate from other PGNs that share the source). The next three bytes are the first three octets of AgIO's IP.

On receipt the TC sets `settings.subnet = [IP0, IP1, IP2]`, closes the main socket, re-runs NIC enumeration, and rebinds. Useful for plug-and-play scenarios where the host may move between subnets.

#### `0xD6` — GPS/IMU data

Built by AgIO's NMEA parser from the GPS receiver's NMEA sentences (serial and UDP input both go through it), source `0x7C` (not `0x7F`). No hardware sends this PGN directly.

AgIO always sends it to AOG over loopback, but only sends it to the module network on 8888 when the AgIO profile setting `setUDP_isSendNMEAToUDP` is on. That setting defaults to off and has had no UI since Feb 2024, so on a default AgIO install the TC never receives `0xD6`. This is tracked upstream in [AgOpenGPS#1231](https://github.com/AgOpenGPS-Official/AgOpenGPS/issues/1231).

The TC only reads byte 38: the fix-quality code (`0`=invalid, `1`=GPS, `2`=DGPS, `3`=PPS, `4`=RTK Fixed, `5`=RTK Float, `6`=Estimated, `7`=Manual, `8`=Simulated — the NMEA 2000 GNSS Method values DDI 514 uses). Values `0`–`8` are forwarded unchanged to implements as DDI 514 (GNSSQuality) — see §5.2.

Fallbacks when there is no usable fix-quality value:

| Case | Sent as DDI 514 | Why |
|---|---|---|
| AgIO reports a value above `8` | `1` | Not a defined GNSS Method; treated as the weakest real GNSS fix. |
| No `0xD6` received since the TC started | `1` | The default AgIO setup doesn't forward it to 8888 (see above), and Simulator mode doesn't produce it at all. `0` is avoided here because some implements gate TRACK/section control on GNSS quality being non-zero. |
| `0xD6` was received, but none in the last 2 s | `0` | The NMEA stream itself stopped (e.g. the receiver was unplugged), so the TC no longer claims a fix. This also happens when switching from a real receiver to Simulator mode mid-session, until the TC is restarted. |

#### `0xE5` — Section states

Reports the *actual* state of up to 64 sections. 8 bytes = 64 bits, one bit per section. The TC forwards these to the connected ISOBUS implement via the appropriate condensed work-state DDIs (DDI 160/161/290).

#### `0xF1` — Section control mode

`1` = automatic (TC drives the implement). `0` = manual (operator drives). The TC logs and propagates this to the implement.

#### `0xF2` — Process data

Wraps a single ISO 11783 DDI/value pair. The TC currently dispatches on these DDIs:

| DDI (decimal) | Name | TC behavior |
|---|---|---|
| `397` | Actual speed (mm/s) | Stored. If TECU enabled, broadcast as Ground/Wheel/Machine-selected speed (PGN 65256) + NMEA2000 SOG. Drives forward/reverse direction. Also produces J1939 PGN 65256 every 100 ms. |
| `597` | Total distance (mm) | Stored and displayed on the VT Status page. If TECU is enabled, also populated into Speed Messages distance fields. |
| Guidance line deviation | XTE (mm) | Converted to metres. Broadcast as NMEA2000 XTE (PGN 0x1F903) at 1 Hz. |
| `1`, `6`, `11` | Setpoint volume (mm³/m²), mass (mg/m²) or count (/m²) per area application rate | Rate setpoint for a rate channel (see below). Not counted as AOG traffic. |
| `158` | Prescription control state | RateController's prescription request for a rate channel (see below): `0` = VR off, `1` = ask again. Not counted as AOG traffic. |

**Rate setpoints (DDI 1/6/11).** Sent by RateController while its VR (Rx) button is on, one message per product, every 1 s and on change, in the DDI's ISO unit (e.g. 1 L/ha = 100 mm³/m² = DDI 1 value `10000` at 0.01 mm³/m² per bit). An optional 7th byte is the **rate channel**, `1`–`5` (RateController's product + 1); without it, channel 1. Other channel numbers are ignored.

- When a DDOP is activated, the TC lists every element with a settable DDI 1, 6 or 11 (log tag `[Rate]`) and matches each to a channel (see "Rate channel info"). For each it also finds DDI 158 (prescription control state), 287 (setpoint prescription mode, settable) and 288 (actual prescription mode) on the element or its nearest parent, and the default rate (DDI 3 / 8 / 13, a property, or process data the TC asks for). 158 and 288 are asked for on change.
- A setpoint goes only to elements on the channel's DDI that take RateController's rate (STARTING or ACTIVE below), when it changes and then every 1 s. Negative values are sent as 0.

**Prescription request (DDI 158).** `[0x9E, 0x00][int32 0 or 1][channel]`. RateController sends `0` every 1 s per product while VR is off (only a change acts), and `1` once for "Ask again".

**Channel states, per element:**

| State | How it gets there | What the TC does |
|---|---|---|
| OFF | Start; RC sends 158 = 0 (VR off); the element leaves its channel | 158 = 0 (confirmed); no 0 setpoint, so the implement uses its own rate. An element without 158 is sent 0 (stop) instead |
| STARTING | A setpoint arrives for an element that is OFF or in FALLBACK; a new implement / DDOP; "Ask again" | 287 = 1 (if settable), 158 = 1, then the setpoint straight after; confirm |
| ACTIVE | The implement reports 158 = 1. An element without 158 (or reporting 3, not installed) is ACTIVE at once | Setpoints as above |
| REFUSED | The implement reports 158 = 0 or 2 while STARTING or ACTIVE, or never replies | Setpoints stop. Logged (`[Rate] Element 20 "Rate 1": implement turned prescription control off`) and shown as an alarm hardware message. **Never asked again by the TC**; only RateController's "Ask again" restarts it |
| FALLBACK | Nothing from RateController for the channel for 3 s (no setpoint, no 158) while STARTING or ACTIVE | With a default rate: 287 = 2 and the default rate. Without: 158 = 0 (hand back). Without either: 0. Alarm hardware message. The next setpoint starts it again |

**Confirming 158.** A lost command changes nothing, so the implement would report nothing. About 1 s after each 158 command without the new state reported, the TC requests 158; if that gets no reply within 1 s, it resends the command (for ON: 287, 158 and the setpoint). After 3 commands without a reply the element is REFUSED (for ON) with **no reply** set (`0xF5` flags bit 5), logged `no reply to prescription control state`. For OFF the TC stops anyway. "No reply" clears when the element reports 158 or a new request starts.

The sender must send continuously (RateController sends every second), not only on change: 3 s of silence is a lost RateController.

Unknown DDIs are silently ignored (PGN 0xF2 is the generic process-data channel — the TC will gain more DDIs over time).

#### `0xF4` — Rate channel info

Sent by RateController (source `0x7F`) for each rate channel while any of its products is an ISOBUS product (whatever VR is, so a channel keeps its elements and its actuals with VR off): on a change and every 2 s. When its last ISOBUS product stops, every channel is sent once with DDI 0.

| Byte | Field | Notes |
|---|---|---|
| 0 | Channel | `1`–`5` (RateController's product + 1) |
| 1–2 | DDI | The channel's type: `1` volume, `6` mass, `11` count per area. `0` = channel not in use |
| 3.. | Name | RateController's product name, UTF-8, 0–24 bytes, not NUL-terminated |

**Matching (interim, automatic).** Each rate element goes on the channel of the same type (DDI). Several elements may share a channel (e.g. a split bin). An element with no channel of its type, or with two or more channels of its type (e.g. seed and fertiliser both by mass), is on no channel and gets no setpoint: two channels of one type need an assignment, which isn't available yet. Without channel info (an older sender), a channel's type is taken from its setpoints.

Matching runs when a DDOP is activated and whenever a channel's type changes. An element that leaves a channel goes OFF (158 = 0; without 158, 0 once); one that joins a channel with a current setpoint (VR on) goes STARTING at once. Every result is logged (`[Rate] Element 7 "Bin 1" (DDI 6) -> channel 1 "Fert"`) and shown on AgOpenGPS's map as a hardware message (`0xDD`, info).

### 2.6 PGNs outbound (TC → client)

All PGNs sent **by the TC to AgIO/AgValonia** use source `0x80`.

| PGN | Name | Length | Frequency | Payload |
|---|---|---|---|---|
| `0xDD` (221) | Hardware message | 2 + T | On event | `[duration, colour, text0, text1, ...]` — `T` = UTF-8 text bytes |
| `0xF0` (240) | Section heartbeat / state | 2 + ⌈N/8⌉ | 100 ms | `[mode, num_sections, byte0, byte1, ...]` |
| `0xF5` (245) | Rate channel actuals, to RateController | 16 | 1 s per channel | See "Rate channel actuals" below. Sent to RateController's address, not broadcast |

#### `0xF5` — Rate channel actuals

What the implement reports for each RateController rate channel (see `0xF4`), so RateController records the applied rate and quantity rather than its target. Sent every second for each channel RateController has announced (`0xF4` with DDI ≠ 0), with elements `0` when none are matched (so RateController can tell "TC there, nothing matched" from "no TC"), and once more after a channel loses its elements. Sent **to the address and port the last `0xF4` came from** (RateController's own socket), not to the `.255:9999` broadcast.

| Byte | Field | Notes |
|---|---|---|
| 0 | Channel | `1`–`5` |
| 1–2 | Actual rate DDI | `2` volume, `7` mass, `12` count per area (the setpoint DDI + 1) |
| 3–6 | Rate | `int32`, raw in that DDI's unit: mean of the channel's elements that reported in the last 3 s |
| 7–8 | Total DDI | `351` (mL), `352` (g) or `82` (count); `0` = no total |
| 9–12 | Total | `int32`: sum of the elements' application totals in that unit (a total on a shared parent element counted once; DDI 80 (L) and 81 (kg) are converted to mL and g) |
| 13 | Elements | Elements matched to the channel |
| 14 | Flags | bit 0 rate valid, bit 1 total valid, bits 2–3 DDI 158 as reported (0 off, 1 on, 2 error, 3 not installed; with several elements, 1 only if every element with 158 reports 1, else the lowest other value), bit 4 channel has 158, bit 5 no reply to 158 (any element) |
| 15 | Prescription mode | DDI 288 as the first element with it reports it (1 TC rate, 2 manual, 3 peer, 4/5 max/min override); `0` = unknown or not present |

**Subscriptions.** For each rate element the TC looks for its actual rate DDI (on the element) and an application total of the same kind (DDI 351 or 80 for volume, 352 or 81 for mass, 82 for count; on the element, else the nearest parent element that has one). Each is asked for every 1 s by time-interval measurement command if the DDOP offers that trigger, else on change, else the TC requests the value every second. Each element's result is logged (`[Rate] Element 20 "Rate 1": actual DDI 2 every 1 s, total DDI 351 on element 20 every 1 s`).

#### `0xDD` — Hardware message

Displays a text banner across the top of the AgOpenGPS map so the operator can see TC state that would otherwise only reach the console log. Payload:

- byte 0: `duration` — see the caveat below
- byte 1: `colour` — `0` renders a Salmon background (alert), any other value renders Bisque (info). The TC sends `1` for info.
- bytes 2..: the message text, UTF-8, **not** NUL-terminated

The frame's `Length` field is therefore `text_bytes + 2`, and AOG reads exactly `Length - 2` bytes starting at wire offset 7. The TC caps text at 60 bytes and truncates on a UTF-8 character boundary — splitting a multi-byte sequence would render as U+FFFD, and the display label clips long text anyway.

**`duration` is frames, not seconds.** AOG stores `duration × 10` into a counter that it decrements once per OpenGL render tick, so the banner lasts `duration × 10` redraws — roughly `duration` seconds at AOG's nominal 10 Hz, but it stretches or shrinks with frame rate. **A condition that persists must be re-sent**; do not send once with a large duration and expect the banner to hold. The operator can also dismiss a banner early by clicking it.

**AgOpenGPS ignores the source byte for this PGN** — it dispatches on the PGN byte alone. The TC still sends `0x80` per §2.2.

**Display is opt-in on the AOG side.** AgOpenGPS drops these frames unless *Config → Data → Hardware Messages* is enabled; the setting defaults to **off**. A correct implementation looks like a no-op until it is switched on, which is the first thing to check when testing. There is no corresponding TC-side setting — the TC always sends.

Messages the TC currently emits:

| Text | Colour | Duration | Trigger |
|---|---|---|---|
| `Implement: <name> (<n> sections, <w> m)` | Info | 5 | An implement with sections registers. The `, <w> m` clause is omitted when the DDOP has no usable geometry. |
| `Implement lost: <name>` | Alert | 10 | A previously seen implement disappears from the client list. |
| `TC address conflict: preferred address in use` | Alert | 20 | Another control function claiming `NAME::Function::TaskController` holds address `0xF7`. Re-sent every 15 s while true. |
| `TC address conflict resolved` | Info | 5 | The above clears. |
| `TECU failed to claim address 240` | Alert | 10 | Once at startup, when the TECU is enabled but could not claim its fixed address. |

The conflict messages surface on the operator's screen what the TC already writes to its console log; the console warning remains and keeps its own 30 s throttle.

#### `0xF0` — Section heartbeat / state

Sent every 100 ms for each connected ISOBUS implement that has sections. The payload is:

- byte 0: `1` if section control is enabled (auto), `0` if disabled (manual)
- byte 1: `num_sections` (the implement's section count)
- bytes 2..: bitfield of *actual* section ON/OFF states (1 bit per section, LSB-first within each byte)

If no implement is currently connected (or none have sections), and `aogHeartbeatEnabled` is `true` in `settings.json`, the TC still sends `0xF0` with `num_sections = 0` and no bitfield — a pure "I'm alive" beacon so AgIO can light up its ISOBUS indicator.

> **Compatibility note:** AgOpenGPS releases **before v6.8.2 beta 5** do not handle the `num_sections = 0` heartbeat correctly. Set `aogHeartbeatEnabled: false` in `settings.json` if pairing with an older AOG. AgValonia and AOG ≥ v6.8.2 beta 5 are fine.

---

## 3. Settings reference

The TC reads `settings.json` from a per-user config directory:

| OS | Path |
|---|---|
| Windows | `%APPDATA%\AOG-TaskController\settings.json` |
| Linux | `$XDG_CONFIG_HOME/AOG-TaskController/settings.json` (or `~/.config/AOG-TaskController/settings.json` if `XDG_CONFIG_HOME` is unset) |
| macOS | `~/Library/Application Support/AOG-TaskController/settings.json` |

```json
{
    "subnet": [192, 168, 5],
    "tecuEnabled": true,
    "nmeaSendEnabled": true,
    "aogHeartbeatEnabled": true,
    "vtEnabled": true,
    "tcVersion": 3,
    "languageCode": "en",
    "countryCode": "US"
}
```

| Key | Type | Default | Description |
|---|---|---|---|
| `subnet` | `int[3]` | `[192, 168, 5]` | First three octets of the LAN AgIO/AgValonia lives on. Used for NIC selection and broadcast destination. |
| `tecuEnabled` | `bool` | `true` | If `true`, the TC also impersonates a Tractor ECU on the CAN bus (claims address 240, broadcasts Speed Messages/NMEA2000, announces Class 1 BasicTractorECUServer). Set `false` when the tractor already has a TECU. |
| `nmeaSendEnabled` | `bool` | `true` | Enable cyclic NMEA2000 COG/SOG transmission. Requires `tecuEnabled`; the VT disables this control when no TECU interface exists. |
| `aogHeartbeatEnabled` | `bool` | `true` | Send `0xF0` heartbeat to AgIO/AgValonia every 100 ms even with no implement. Disable for AOG < v6.8.2 beta 5. |
| `vtEnabled` | `bool` | `true` | Register the Virtual Terminal client and display the TC UI when a VT is present. |
| `tcVersion` | `integer` | `3` | Task Controller version code (`0`=DIS, `1`=FDIS.1, `2`=First Edition, `3`=Second Edition Draft, `4`=Second Published Edition). |
| `languageCode` | `string` | `"en"` | Two-character language code advertised through the ISOBUS language interface. |
| `countryCode` | `string` | `"US"` | Two-character country code advertised through the ISOBUS language interface. |

Unknown keys are ignored. Settings updates are written to a temporary file and atomically renamed over `settings.json`; the previous file is preserved if writing fails.

---

## 4. Command-line reference

```
AOG-TaskController [options]
```

| Flag | Default | Description |
|---|---|---|
| `--help` | — | Print usage and exit. |
| `--version` | — | Print git-describe version (with `-dirty` suffix on a dirty tree) and exit. |
| `--can_adapter=<name>` | none | CAN driver. One of: `peak-pcan`, `innomaker-usb2can`, `rusoku-toucan`, `sys-tec-usb2can` (all Windows), `socketcan` (Linux). **Required** — the TC will not start without a driver. |
| `--can_channel=<id>` | — | Driver-specific channel. Numeric (`1`, `2`, ...) for Windows USB adapters; interface name (`can0`, `vcan0`) for SocketCAN. |
| `--log_level=<lvl>` | — | One of `debug`, `info`, `warning`, `error`, `critical`. Filters AgIsoStack log output. |
| `--log2file` | off | Also write all output to `<config>/logs/AOG-TaskController_YYYY-M-D_H-M.log`. |

---

## 5. ISOBUS / CAN side overview

This section is for context. The TC implements the bus side according to ISO 11783 — that standard, plus the AgIsoStack documentation, is the authoritative reference.

### 5.1 What the TC presents on the bus

Two control functions, both claiming addresses via standard J1939-81 address claim (with the 250 ms post-claim quiet period enforced):

| CF | NAME function | Address | Notes |
|---|---|---|---|
| Task Controller | `TaskController` (function code 61) | Preferred `247` (ISO 11783-10 MappingComputer). Walks if claimed. | Always present. |
| Tractor ECU | `TractorECU` (function code 132) | Fixed `240` (non-arbitrary-address-capable per ISO 11783-9). | Only present if `tecuEnabled: true`. |

Common NAME fields: Industry Group `2` (Agricultural), Device Class `0`, Manufacturer Code `1407`, Identity `20`. Override these in `app.cpp` if you fork.

### 5.2 What the TC sends on the bus

| PGN | Cadence | Producer | Purpose |
|---|---|---|---|
| `0xCB00` (Process Data) | 2 s | TC | ISO 11783-10 B.8.1 Task Controller Status. Status byte bit 1 = task totals active. |
| `0x1F903` (NMEA2000 XTE) | 1 Hz | TC | Cross-track error, derived from AOG's guidance-line deviation PGN. |
| `0xFEE8` (PGN 65256 Speed/Direction) | 100 ms | TECU | Ground/Wheel/Machine-selected speed + machine direction, J1939 format. Only when TECU enabled. |
| `0xFC8E` (Control Function Functionalities) | At claim + periodic | TECU | Announces Class 1 BasicTractorECUServer (no options). |
| NMEA2000 COG/SOG | Periodic | TECU | Optional course/speed over ground. |
| GNSS Quality (DDI 514, via `0xCB00` Process Data) | 250 ms | TC | GPS fix quality from AgIO (PGN `0xD6`, see §2.5), sent to each client whose DDOP declares DDI 514 as settable. Falls back to `1` if `0xD6` has never arrived, and to `0` once it stops arriving (see §2.5). |

The TC also receives all ISOBUS Process Data (PGN 0xCB00) and Section Control commands from connected implements.

### 5.3 What the TC handles inbound from implements

- **Device Descriptor Object Pool (DDOP)** uploads from clients (stored per client).
- **Condensed actual work-state DDIs** (160, 161, 290, plus the extended range 16001–16016 per the standard): mapped into the per-client section model and forwarded to AgIO/AgValonia as PGN `0xF0`.
- **Section control state DDI**: tracked per client.
- **Process data acknowledges (PDACK)**: logged.

### 5.4 ISOBUS feature scope

| Capability | Value |
|---|---|
| ISO 11783-10 version | 2 (Second Edition) |
| Generation | 1 (TC-SC) |
| Max booms | 1 |
| Max sections | 64 |
| Supported DDIs | 160 / 161 / 290 (condensed section setpoint and actual states), plus speed/distance/guidance DDIs from the tractor side |

### 5.5 Virtual Terminal UI

The roughly 12 KB VT object pool is embedded in the executable, so deployment does not require a separate `AOG_TC.iop` file. The committed `src/AOG_TC.iop` remains its build-time source of truth, while `src/AOG_TC.iop.h` contains the ISO-Designer-generated object IDs and authored geometry.

The pool was authored for a 480-pixel data mask and an 80-pixel softkey designator. The client asks AgIsoStack to scale both before initialization, and includes that scaling contract in the VT cache version so a terminal cannot reuse a pool cached by an older unscaled build. The UI defines five virtual navigation softkeys; terminals with fewer than five physical softkeys must support paging.

At connection, the TC logs the VT version, screen dimensions, softkey dimensions, and virtual/physical softkey counts. It warns when fewer than five virtual softkeys are available. If a VT address is detected but the client has not connected after 30 seconds, it logs the reported capabilities and recovery guidance. Clear the terminal's stored/cached object pools first when diagnosing an upload failure, because stale pools and full non-volatile pool storage can prevent an otherwise compatible upload.

---

## 6. Example clients

These examples talk to a TC listening on subnet `192.168.5.x`. Adjust the broadcast address to your LAN.

### 6.1 Python — minimal heartbeat sender + section reader

```python
"""
Sends a subnet-detection packet, then prints every PGN 0xF0 heartbeat
that the TC broadcasts. Useful for confirming the TC is alive on your LAN.
"""
import socket
import struct
import threading

TC_HOST_BCAST = ("192.168.5.255", 8888)
LISTEN_PORT = 9999
SRC_AGIO = 0x7F
PGN_SUBNET_DETECT = 0xC9
PGN_SECTION_HEARTBEAT = 0xF0

def frame(src: int, pgn: int, payload: bytes) -> bytes:
    header = bytes([0x80, 0x81, src, pgn, len(payload)])
    crc = sum(header[2:] + payload) & 0xFF
    return header + payload + bytes([crc])

def announce_subnet():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    # IP0 IP1 IP2 = first three octets of OUR (AgIO/AgValonia) IP
    payload = bytes([0xC9, 0xC9, 192, 168, 5])
    s.sendto(frame(SRC_AGIO, PGN_SUBNET_DETECT, payload), TC_HOST_BCAST)
    s.close()

def listen():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", LISTEN_PORT))
    while True:
        data, addr = s.recvfrom(512)
        if len(data) < 6 or data[0] != 0x80 or data[1] != 0x81:
            continue
        src, pgn, length = data[2], data[3], data[4]
        payload = data[5:5 + length]
        if pgn == PGN_SECTION_HEARTBEAT:
            mode = "AUTO" if payload[0] == 1 else "MANUAL"
            n = payload[1]
            states = []
            for i in range(n):
                byte = payload[2 + (i // 8)]
                states.append("1" if byte & (1 << (i % 8)) else "0")
            print(f"{addr[0]}  heartbeat  mode={mode}  sections={n}  bits={''.join(states) or '(none)'}")

threading.Thread(target=listen, daemon=True).start()
announce_subnet()
import time; time.sleep(60)
```

### 6.2 C# — frame builder for AgValonia / .NET

```csharp
// Reusable AgIO/AgValonia frame builder. Targets .NET 8+.
// Drop into a System.Net.Sockets.UdpClient and you're done.

using System;

public static class TaskControllerFrame
{
    public const byte SrcAgio = 0x7F;
    public const byte SrcTc   = 0x80;

    public const byte PgnSubnetDetect      = 0xC9;
    public const byte PgnSectionStates     = 0xE5;
    public const byte PgnSectionControl    = 0xF1;
    public const byte PgnProcessData       = 0xF2;
    public const byte PgnSectionHeartbeat  = 0xF0;

    /// <summary>Build a complete framed packet ready for UdpClient.Send.</summary>
    public static byte[] Build(byte src, byte pgn, ReadOnlySpan<byte> payload)
    {
        if (payload.Length > 250)
            throw new ArgumentException("payload too large");

        var buf = new byte[6 + payload.Length];
        buf[0] = 0x80;
        buf[1] = 0x81;
        buf[2] = src;
        buf[3] = pgn;
        buf[4] = (byte)payload.Length;
        payload.CopyTo(buf.AsSpan(5));

        int sum = 0;
        for (int i = 2; i < 5 + payload.Length; i++) sum += buf[i];
        buf[5 + payload.Length] = (byte)(sum & 0xFF);
        return buf;
    }

    /// <summary>Tell the TC which subnet AgValonia lives on.</summary>
    public static byte[] SubnetDetect(byte ip0, byte ip1, byte ip2) =>
        Build(SrcAgio, PgnSubnetDetect, new byte[] { 0xC9, 0xC9, ip0, ip1, ip2 });

    /// <summary>Enable (true) or disable (false) automatic section control.</summary>
    public static byte[] SectionControlMode(bool enabled) =>
        Build(SrcAgio, PgnSectionControl, new byte[] { (byte)(enabled ? 1 : 0) });

    /// <summary>Set the actual ON/OFF state of up to 64 sections.</summary>
    public static byte[] SectionStates(ulong bitmap)
    {
        var b = new byte[8];
        for (int i = 0; i < 8; i++) b[i] = (byte)(bitmap >> (i * 8));
        return Build(SrcAgio, PgnSectionStates, b);
    }

    /// <summary>Send a single process-data (DDI, value) pair to the TC.</summary>
    public static byte[] ProcessData(ushort ddi, int value)
    {
        var p = new byte[6];
        p[0] = (byte)(ddi & 0xFF);
        p[1] = (byte)(ddi >> 8);
        p[2] = (byte)(value);
        p[3] = (byte)(value >> 8);
        p[4] = (byte)(value >> 16);
        p[5] = (byte)(value >> 24);
        return Build(SrcAgio, PgnProcessData, p);
    }
}
```

### 6.3 Quick command-line sanity check

```bash
# Listen for TC heartbeats with a one-liner (Linux/macOS):
nc -ul 9999 | xxd | head

# Send a subnet-detect packet with socat:
printf '\x80\x81\x7f\xc9\x05\xc9\xc9\xc0\xa8\x05\x4e' | socat - UDP-DATAGRAM:192.168.5.255:8888,broadcast
```

(The CRC byte `0x4e` is `0x7F + 0xC9 + 0x05 + 0xC9 + 0xC9 + 0xC0 + 0xA8 + 0x05` mod 256.)

---

## 7. Future direction

- The CRC verification path is in `udp_connections.cpp` but commented out. Clients are encouraged to send a correct checksum even though it is not enforced today.
- New DDIs are routinely added to PGN `0xF2`. The set in §2.5 is a snapshot — check `src/app.cpp` for the current dispatcher.
- macOS builds are not in CI yet, but the source compiles cleanly on macOS (with `CAN_DRIVER=MacCANPCAN`) and the config paths follow Apple conventions.
