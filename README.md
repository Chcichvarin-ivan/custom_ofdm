# custom_ofdm — Drone-to-Ground H.265 Video Link over OFDM

A complete digital video downlink for a drone: H.265 video, RaptorQ forward
error correction, and an 802.11a-style OFDM PHY on USRP B200 mini radios at
3.3 GHz. **Field-proven to 2 km** at 300–400 m altitude (10 W PA on the
aircraft, LNA on the ground) with real-time 720p video.

This is a heavily hardened fork of
[bmorgan5/fun_ofdm](https://github.com/bmorgan5/fun_ofdm), the 802.11a OFDM
PHY from the FUNLAB at the University of Washington (GPL v2). The PHY
architecture (frame detection, timing sync, FFT, equalization, Viterti/CRC
frame decode) is inherited; everything above it — and several critical fixes
inside it — is this project. The original README, targeting USRP N210
research use, is preserved as `README_upstream.md`.

---

## System architecture

```
DRONE (Radxa Rock 5B, ARM)                    GROUND (x86 Linux)
┌─────────────────────────────┐               ┌──────────────────────────────┐
│ USB camera (MJPEG)          │               │ B200 mini ── LNA ── patch ant│
│   └─ GStreamer decode       │               │   └─ fun_ofdm RX chain       │
│      └─ mpph265enc (HW)     │    3.3 GHz    │      └─ FEC decoder (sync-   │
│         └─ packetizer       │   ~~~~~~~~>   │         hardened RaptorQ)    │
│            └─ FEC encoder   │    OFDM       │         └─ frame reassembly  │
│               (RaptorQ)     │  QPSK 1/2     │            └─ avdec_h265     │
│               └─ fun_ofdm TX│   10 Msps     │               └─ display+TUI │
│                  └─ B200mini│               │                              │
│                     └─ 10W  │               │ Pluto+ (optional, libiio)    │
│                        PA   │               │   RX1/RX2 ─ pluto_rx ─┐      │
└─────────────────────────────┘               │   (diversity branches │ UDP  │
        10 MHz OCXO ref                       │    merge at FEC) <────┘      │
        on BOTH radios                        └──────────────────────────────┘
```

Every additional ground receiver (the Pluto+ channels, or any future branch)
runs its own complete receive chain and forwards decoded PHY packets over UDP
into the same FEC decoder — packets are self-identifying, duplicates are
ignored, so **whichever antenna's physics worked, its packets count**.

## What's changed versus upstream fun_ofdm

Fixes inside the PHY (`src/`):

* **Working CFO correction** (`timing_sync`). Upstream's frequency-offset
  estimation loop was dead code (`for(k = LTS1; k < LTS1; ...)`) — the
  receiver applied **no** carrier-frequency correction at all. Replaced with
  a correct Schmidl & Cox estimate over the two LTS symbols, plus a **clip
  gate** (never learn an estimate from a clipped preamble — a persistent
  garbage estimate corrupts every following frame) and a **watchdog** (reset
  to neutral after 250 ms without an LTS, so stale corrections can't outlive
  their signal).
* **Receive-power tap** (`frame_detector` → `rx_power_monitor.h`): publishes
  mean sample power and a near-clip fraction from samples the detector
  already touches. Feeds the AGC and the CFO clip gate. No hardware RSSI
  sensor anywhere — the B200 mini's `rssi` sensor is unreadable while
  streaming and is abandoned.

Everything above the PHY (`examples/`):

* **video_tx / video_rx** — the H.265 link itself: hardware encode on the
  RK3588 (with a required `videoconvert` copy working around an RGA
  file-descriptor leak that froze the encoder after ~40 s), packetization
  with keyframes every ~15 frames, keyframe-aware queue dropping, and
  loss-aware resync on the receive side (after any incomplete frame, drop
  until the next keyframe).
* **RaptorQ FEC** (`fec_encoder.h` / `fec_decoder.h`, needs libRaptorQ):
  K source symbols + R repair per generation. The decoder is
  **sync-hardened**: a large generation-id jump needs corroboration from
  multiple packets before the window moves (one corrupted header can no
  longer wedge the session), and a re-acquire watchdog self-heals a desynced
  window — including after a TX restart — in ~1.5 s instead of requiring an
  RX restart.
* **Software AGC v2** (`link_agc.h`): fast-attack / slow-release on the
  sample-power level. Emergency −6 dB on clipping (>1 % near-clip samples),
  proportional down-steps, gain rises only after *sustained* weakness so
  multipath fades never pump it, wide dead zone (−28…−14 dBFS). Designed for
  a moving platform; the old 1 dB/s loop could not track motion.
* **TUI** (`link_tui.h`): live link health — PHY packets/s, rejects, FEC
  generations decoded/dropped, repair-symbol usage, frames/fps, link and
  video bitrates, RX gain.
* **Diversity receiver** (`pluto_rx.cpp`, `iio_source.*`, `diversity_rx.h`):
  a standalone process driving a Pluto+ (AD9363, libiio, no UHD) with one
  or two RX channels, each through its own receive chain, forwarding PHY
  packets over UDP (localhost:5599) into video_rx's decoder. Additive by
  design: if it dies, the main link is untouched.
* **Optional TX interleaver** (`phy_spreader.h`): generation-id-keyed block
  interleaver spreading burst losses across `depth` generations
  (tolerance ≈ depth × R packets, latency ≈ depth × generation cadence).
  Integrate only if generations drop in clumps during motion.
* **Self-test suites** — see [Testing](#testing).

## Hardware

| | Drone (TX) | Ground (RX) |
|---|---|---|
| Computer | Radxa Rock 5B (RK3588) | x86 Linux |
| Radio | USRP B200 mini | USRP B200 mini (+ optional Pluto+) |
| RF chain | 10 W PA → **harmonic low-pass filter** → antenna | antenna → (band-pass filter) → LNA → radio |
| Reference | 10 MHz OCXO → external ref input | 10 MHz OCXO (same family) → external ref input |
| Camera | USB UVC, MJPEG | — |

**Clocking matters enormously.** Both radios run
`set_clock_source("external")` from a 10 MHz OCXO; verify `Ref locked: yes`
at startup on both ends. Disciplining both ends transformed link quality in
this project's history. The optional Pluto+ takes the same 10 MHz through
its external-clock input (EXCLK jumper to GND; either declare 10 MHz via the
u-boot `ad936x_custom_refclk` method or translate 10→40 MHz externally).

**⚠ Close-range RF safety.** With a 10 W PA and an LNA'd receiver, the RX
front end saturates on approach inside ~200–300 m and the **B200's damage
threshold (~+10 dBm) is approached inside ~50 m**. Never key the PA near the
ground station; power it down for takeoff/landing/close work; plan an LNA
bypass or step attenuator for close-range operation. A "10 W" PA is also a
~1–2 W *OFDM* amplifier (8–10 dB peak-to-average backoff) — find the drive
level by sweeping TX gain for maximum received packet rate, not maximum
power; the harmonic filter after it is mandatory.

## Dependencies

Upstream set: CMake ≥ 3.9, make, **UHD**, **FFTW3**, **Boost**, pthread.

Added by this project:

* **libRaptorQ** (+ Eigen3) — FEC. Point CMake at it with `LIBRAPTORQ_DIR`.
* **GStreamer 1.x** + `gstreamer-app` + libav (`avdec_h265`) — video.
  On the Radxa additionally the Rockchip MPP plugin (`mpph265enc`).
* **OpenCV** — RX display.
* **libiio** — only for the optional `pluto_rx` target.

## Build

Two machines, two builds. `src/` is the shared PHY library — any change
there requires rebuilding the library, not just the examples.
**`examples/fec_common.h` must be byte-identical on both machines**
(`md5sum` it) — it defines the FEC wire format.

```bash
git clone <this repo> && cd custom_ofdm
mkdir build && cd build
cmake -DWITH_FEC=ON -DWITH_TUI=ON -DWITH_AGC=ON \
      -DLIBRAPTORQ_DIR=/path/to/libRaptorQ/src ..
make
```

| CMake option | Meaning | Recommended |
|---|---|---|
| `WITH_FEC` | RaptorQ FEC (must match on TX and RX) | **ON** |
| `WITH_TUI` | terminal link-health display (RX) | ON |
| `WITH_AGC` | software AGC v2 (RX) | ON |
| `WITH_OVERLAY` | on-video HUD (RX) | optional |
| `WITH_DIAG` | legacy hardware-RSSI thread | **OFF** — dead end on B200 mini, implicated in instability |
| `WITH_PLUTO` | build `pluto_rx` (needs libiio) | ON on the ground box if using a Pluto+ |

Binaries land in `bin/` — run `./bin/video_rx`, not a stale copy from a
build directory.

## Configuration — the rules that bite

These constants live at the top of `video_tx.cpp` / `video_rx.cpp` and in
`fec_common.h`. Violating the first two produces **zero packets, silently**.

1. **`FREQ` identical on both ends** (this project: `3.3e9`, the licensed
   band). A 3.3/3.4 mismatch has burned this project twice.
2. **`SAMPLE_RATE` identical on both ends** (this project: `10e6`).
3. **Capacity budget.** 10 Msps QPSK 1/2 carries ≈ 2.4 Mbit/s. The encoder
   bitrate must satisfy `BPS × (1 + R/K) < capacity`. At K=32, R=16 (50 %
   overhead): `BPS ≈ 1.2–1.5 Mbit/s`. Oversubscribing doesn't degrade
   gracefully — the queue drops everything but keyframes and you get ~3 fps.
4. **FEC K/R.** K=32, **R=16** is the proven configuration (R=8 recovered
   71 % of generations in the field; R=16 recovered 92 %+). Burst tolerance
   per generation ≈ R packets ≈ 70 ms of outage at typical packet rates.
5. **AGC:** `set_window(-28.0, -14.0)` (dBFS), `set_gain_band(10, 70)`.
   Do **not** override `set_period_ms` — the attack/release timing assumes
   the 100 ms default.
6. Run the RX with `sudo` (or grant rtprio) for realtime scheduling.

## Running

```bash
# Ground:
sudo ./bin/video_rx                      # TUI + video window; 'q' quits
sudo ./bin/video_rx 2>rx.log             # capture AGC/diversity logs

# Drone:
./bin/video_tx

# Optional diversity branches (ground, after the Pluto+ clock is set up):
./bin/pluto_rx --uri ip:192.168.2.1 --gain 50 --channels 2
# stderr prints per-branch pkt/s and peak |sample| (lower --gain if ≥0.9);
# video_rx prints "[diversity] listening on 127.0.0.1:5599" and simply
# gains packets. Acid test: unplug the B200 antenna — video keeps flowing.
```

The TUI reads top-to-bottom as signal → PHY → FEC → application. A healthy
link: `Rejected` near zero, `gens lost` ≈ 0, `Repair use` well under 20 %,
`Link rate ≈ Video rate`.

## Testing

Every subsystem has a hermetic self-test — no radio needed. Run them after
touching anything, and **bench-test before flying** any change to `src/`.

| Suite | Covers |
|---|---|
| `cfo_selftest` | CFO estimate corrects a known offset; clip gate refuses clipped preambles; watchdog clears stale corrections |
| `agc_selftest` | closed-loop AGC: hold / clip-attack / step caps / anti-pumping on brief fades / gated release / rail anti-windup |
| `fec_window_selftest` | decoder sync-hardening: lone corrupted gen-id can't move the window; corroborated jumps do; watchdog heals wedges and TX restarts |
| `diversity_selftest` | UDP framing, garbage rejection, two-branch concurrency integrity, merge into the real decoder, overload throughput |
| `spreader_selftest` | interleaver: full + partial generations, burst spreading ≤ R per generation, idle flush |
| `ppdu_loopback_test` | TX→RX chain in memory, with AWGN sweep (Viterbi cliff) |

Each is one `.cpp` with a build one-liner in its header (RaptorQ is stubbed
where FEC math isn't the subject). Suites with radio APIs use instrumented
fakes so assertions cover *speed and step size*, not just final values.

## Known issues

* TUI prints doubled section headers and a stray `Link rate: 0 bps` in the
  FEC section — cosmetic merge artifact; the application-layer numbers are
  authoritative.
* Hardware RSSI shows `n/a` by design — the B200 mini's sensor is
  unreadable while streaming. The dBFS level from the software power
  monitor is the real measurement.
* The `pluto_rx` UDP transport assumes localhost (datagrams exceed a 1500
  MTU; loopback doesn't care).

## Troubleshooting (from this project's actual history)

* **Zero packets, `NO SIGNAL`, RSSI at noise floor** → `FREQ` or
  `SAMPLE_RATE` mismatch between TX and RX, or the TX isn't transmitting
  (its TX LED is off when no samples flow). Check the constants first —
  this is the most common total-failure cause.
* **AGC gain frozen / never moves** → the `frame_detector` power tap isn't
  in the built library: `grep -c RxPowerMonitor src/frame_detector.cpp`,
  then a *clean* rebuild (the tap is in `libfun_ofdm`, not the example).
* **Link healthy, then rots over minutes; RX restart fixes it** → you're
  running a pre-hardening `fec_decoder.h`. The sync-hardened decoder
  self-heals in ~1.5 s; verify with `fec_window_selftest`.
* **Link dies on close approach and doesn't recover until backed way off**
  → front-end saturation (LNA + strong signal). Software cannot fix it;
  bypass/attenuate the LNA or back off TX power.
* **3 fps, huge frames, link rate ≈ video rate** → encoder bitrate exceeds
  link capacity; redo the budget in rule 3 above.
* **`Ref locked: NO`** → that radio isn't seeing the 10 MHz reference —
  check the OCXO feed before anything else; an undisciplined pair is a
  different (worse) link.
* **Everything built but behavior didn't change** → stale binary or stale
  library: `rm -rf build`, rebuild, and run from `bin/`.

## Roadmap

Soft-decision Viterbi (~+2 dB); a slow back-channel for TX power control
and adaptive bitrate/MCS (turns the range cliff into a slope, and solves
close-approach overload at the source); coherent 2-branch MRC on the
Pluto+'s shared-LO channels; `phy_spreader` integration if motion testing
shows clumped generation loss.

## License & credits

GPL v2, inherited from
[fun_ofdm](https://github.com/bmorgan5/fun_ofdm) (FUNLAB, University of
Washington) — the OFDM PHY this project stands on. FEC by
[libRaptorQ](https://github.com/LucaFulchir/libRaptorQ). Everything else —
the video pipeline, FEC integration and sync hardening, the CFO fix, AGC v2,
the diversity architecture, and the test suites — was built and
field-debugged for this link.
