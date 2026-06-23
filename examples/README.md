# H.265 video link — unified, full-featured

The H.265 hardware-encode link merged with the full instrumentation stack:
FEC, terminal TUI, RSSI diagnostic, software AGC, and on-video overlay — each
an independent build flag. TX runs on the Radxa (hardware VPU encode); RX on
x86 (software HEVC decode) and carries all the instrumentation.

## Build flags (mix and match)

  -DWITH_FEC      RaptorQ FEC. STRONGLY recommended for H.265 (see note).
  -DWITH_TUI      Terminal link-health dashboard (RSSI, gain, FEC, rates).
  -DWITH_DIAG     RSSI + error-rate diagnostic thread (feeds TUI/overlay).
  -DWITH_AGC      Slow software AGC — keeps RSSI in window by nudging RX gain.
  -DWITH_OVERLAY  Draws RSSI / verdict / gain HUD onto the video.

All five are independent and compose. Verified: every combination compiles,
including all five at once.

## ⚠️ H.265 needs FEC

Measured: H.265 without FEC loses ~80% of frames at just 1.5% packet loss
(one lost packet corrupts video until the next keyframe). Build BOTH ends with
-DWITH_FEC. This is the link's stability foundation, not optional.

## Files

  video_tx.cpp        Radxa TX: camera -> mpph265enc (HARDWARE) -> appsink ->
                      packetize -> fun_ofdm. 720p @ 4 Mbit/s, keyframe ev. 15.
  video_rx.cpp        x86 RX: fun_ofdm -> reassemble -> appsrc -> avdec_h265
                      (SOFTWARE) -> overlay -> display. Loss-aware keyframe
                      resync. Carries FEC + TUI + DIAG + AGC + OVERLAY.
  video_packet.h      Shared 20-byte header (keyframe flag included). BOTH
                      sides include it so the wire format can't drift.
  link_stats.h        Thread-safe stats (counters + RSSI + verdict + gain).
  link_tui.h          Terminal dashboard (RF line shows RSSI, status, gain).
  link_overlay.h      On-video HUD.
  link_diagnostics.h  RSSI/error diagnostic thread.
  link_agc.h          Slow software AGC (target -60..-35 dBm, gain 10..60 dB,
                      1 dB steps @ 1 Hz, dead zone + clamp + anti-windup).
  fec_common/encoder/decoder.h   RaptorQ FEC (K=32, R=8).

## The AGC and UHD AGC are mutually exclusive

link_agc.h OWNS the RX gain. Do NOT also enable UHD's hardware AGC — they would
fight. The software AGC is slow on purpose (adjusts on the distance-change
timescale, ~1 Hz) so it never disturbs OFDM sync, unlike UHD's fast AGC which
chases the OFDM envelope and breaks the link.

## Dependencies

Radxa (TX): GStreamer + Rockchip plugins (mpph265enc), gstreamer-app, UHD,
            OpenCV. (+ libRaptorQ if WITH_FEC.)
x86 (RX):   GStreamer + gstreamer-app + gstreamer1.0-libav (avdec_h265), UHD,
            OpenCV. (+ libRaptorQ if WITH_FEC.)
            sudo apt install gstreamer1.0-libav libgstreamer-plugins-base1.0-dev

## Build (example — everything on)

  RX (x86):
    g++ -std=c++14 -O2 -pthread video_rx.cpp [fun_ofdm objs/libs] \
        -DWITH_FEC -DWITH_TUI -DWITH_DIAG -DWITH_AGC -DWITH_OVERLAY \
        -I<libRaptorQ>/src -include cstdint \
        $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0 opencv4 uhd)

  TX (Radxa):
    g++ -std=c++14 -O2 -pthread video_tx.cpp [fun_ofdm objs/libs] \
        -DWITH_FEC -DWITH_TUI \
        -I<libRaptorQ>/src -include cstdint \
        $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0 uhd)

  (RX needs opencv4; TX doesn't display so it doesn't.)
  In CMake, add gstreamer-1.0 + gstreamer-app-1.0 to both targets, and uhd to
  both (the AGC/diag on RX need UHD headers; the TX already links UHD).

## Prerequisites carried over from earlier work

- get_multi_usrp() accessor on usrp.h / receiver_chain.h / receiver.h — DIAG
  and AGC both need it (to read RSSI and set gain). You confirmed this is in
  (the diagnostic worked).
- FEC headers must MATCH between TX and RX. md5sum fec_common.h fec_encoder.h
  fec_decoder.h on both — all three must be identical, or FEC produces broken
  generations on an otherwise-working link. Clean-rebuild both after any change.
- Verify the TX keyframe property name: gst-inspect-1.0 mpph265enc | grep -i gop

## What's verified vs. tested on hardware

VERIFIED in sandbox:
  - Every build flag combination compiles (incl. all five at once), real GStreamer + UHD.
  - RX H.265 decode path RUN-tested (real stream decoded; loss-resync measured).
  - AGC control law unit-tested (converges, dead zone, clamps, no oscillation).

TESTED ON YOUR HARDWARE (can't emulate):
  - TX hardware VPU encode (mpph265enc).
  - AGC actually moving the B200's gain (logic verified; the set_rx_gain call
    on real hardware is yours to confirm).
  - End-to-end over the radio.

## Bring-up order

1. First confirm FEC works at all (the earlier blocker): md5sum the 3 FEC
   headers on both machines (must match), clean-rebuild both, and run the
   loopback test at 0% loss (must PASS 100%). Don't move on until FEC is proven.
2. Build RX (x86) with all flags. Run; it waits for packets and shows the TUI.
3. Build TX (Radxa) with -DWITH_FEC -DWITH_TUI. Verify encode pipeline starts.
4. Run TX. On the RX TUI watch: RSSI, the AGC gain tracking, FEC decode counts,
   and video appearing once the first intact keyframe arrives.
5. With AGC on: start stationary, confirm gain settles into the window and
   holds (no hunting). Then move the antenna away and watch gain rise.
