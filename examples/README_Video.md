# Webcam video over fun_ofdm

Two programs that stream webcam video over a pair of USRPs using the
[fun_ofdm](https://github.com/bmorgan5/fun_ofdm) 802.11a PHY, with every
packet **strictly 1900 bytes**.

- `video_tx.cpp` — captures from webcam, JPEG-encodes each frame,
  fragments into 1900-byte packets, sends via `transmitter::send_packet`.
- `video_rx.cpp` — registers a callback with `receiver`, reassembles
  JPEG frames, displays them with OpenCV.

## Packet format (1900 bytes)

| offset | size | field          | notes                                  |
|-------:|-----:|----------------|----------------------------------------|
|      0 |    4 | `magic`        | `0xDEADBEEF`, network byte order       |
|      4 |    4 | `frame_id`     | `uint32`, network byte order           |
|      8 |    2 | `chunk_index`  | `uint16`, 0-based                      |
|     10 |    2 | `total_chunks` | `uint16`                               |
|     12 |    2 | `payload_size` | `uint16`, actual bytes in this chunk   |
|     14 |    6 | reserved       | zeroed                                 |
|     20 | 1880 | payload        | zero-padded on the last chunk          |

Header = 20 B, payload = 1880 B, total = **1900 B exactly** for every
packet the transmitter emits. `video_tx` asserts `packet.size() ==
1900` before calling `send_packet`.

## Radio parameters

Both programs default to:
- `freq`        = 5.72 GHz
- `sample_rate` = 5 MHz
- `tx_gain`     = 30, `rx_gain` = 30
- `tx_amp`      = 0.5
- `phy_rate`    = `RATE_3_4_QAM16` (36 Mbps) — chosen as a reasonable
  compromise between throughput and robustness. Swap to
  `RATE_1_2_BPSK` if the link is noisy or `RATE_3_4_QAM64` for more
  bandwidth.

## Build

Install fun_ofdm first:

```bash
git clone https://github.com/bmorgan5/fun_ofdm.git
cd fun_ofdm && mkdir build && cd build && cmake .. && make && sudo make install && sudo ldconfig
```

Install OpenCV (Debian/Ubuntu): `sudo apt-get install libopencv-dev`.

Then either:

**Option A — drop into `fun_ofdm/examples/`** and add these lines to
`fun_ofdm/examples/CMakeLists.txt`:

```cmake
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(video_tx video_tx.cpp)
target_link_libraries(video_tx fun_ofdm ${OpenCV_LIBS})

add_executable(video_rx video_rx.cpp)
target_link_libraries(video_rx fun_ofdm ${OpenCV_LIBS})
```

Then rebuild the parent project.

**Option B — standalone**, using the included `CMakeLists.txt`:

```bash
mkdir build && cd build
cmake ..
make
```

## Run

On two machines (each with its own USRP), on the same channel:

```bash
# receiver first
sudo ./video_rx

# then transmitter
sudo ./video_tx
```

Press `q` or `ESC` in the video window to quit.

## Tuning notes

- Default frame size is 320×240 @ JPEG quality 50. At ~10–20 KB per
  frame that's 6–11 packets per frame. At 15 fps and 36 Mbps PHY you
  have plenty of headroom.
- If you see choppy video, lower `TARGET_FPS` / `JPEG_Q` / `FRAME_W` /
  `FRAME_H` in `video_tx.cpp`, or raise the PHY rate.
- The receiver's CRC check drops corrupted packets silently (this is
  done inside fun_ofdm's frame decoder), so missing chunks simply
  result in a dropped frame — the receiver will just show the next
  complete frame.
- Partial frames older than 1 s are garbage-collected.
