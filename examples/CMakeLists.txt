########################################################################
# Examples CMakeLists.txt — RaptorQ FEC + TUI + DIAG + AGC + OVERLAY,
# plus GStreamer for the H.265 video bridges.
########################################################################
#cmake_policy(SET CMP0167 OLD)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

########################################################################
# GStreamer — REQUIRED by the H.265 video_tx/video_rx (encode/decode
# pipelines). Without this block the compiler never gets the -I path to
# gst/gst.h even though the dev packages are installed.
########################################################################
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

########################################################################
# Include directories so cmake knows where .h files live
########################################################################
include_directories(${PROJECT_SOURCE_DIR}/src)

########################################################################
# Feature flags
#
#   cmake ..                                          # baseline
#   cmake -DWITH_TUI=ON ..                            # terminal stats
#   cmake -DWITH_FEC=ON -DLIBRAPTORQ_DIR=... ..       # FEC
#   cmake -DWITH_FEC=ON -DWITH_TUI=ON -DWITH_DIAG=ON -DWITH_AGC=ON \
#         -DWITH_OVERLAY=ON -DLIBRAPTORQ_DIR=... ..   # everything (RX)
#
# Shim headers (fec_*.h, link_*.h, video_packet.h) live in this examples/ dir.
########################################################################
option(WITH_FEC "Build video bridges with RaptorQ FEC" OFF)
option(WITH_TUI "Build video bridges with terminal link-health display" OFF)
option(WITH_OVERLAY "Build video_rx with on-video HUD overlay" OFF)
option(WITH_DIAG "Build video_rx with the RSSI/error diagnostic thread" OFF)
option(WITH_AGC "Build video_rx with the software AGC" OFF)
option(WITH_STATS "Build video_rx with the software STATS" OFF)

find_package(Threads REQUIRED)

if(WITH_FEC)
    set(LIBRAPTORQ_DIR "" CACHE PATH
            "Path to libRaptorQ/src (directory containing RaptorQ/)")

    if(NOT LIBRAPTORQ_DIR)
        find_path(LIBRAPTORQ_DIR RaptorQ/RaptorQ_v1_hdr.hpp
                PATHS /usr/local/include /usr/include)
    endif()
    if(NOT LIBRAPTORQ_DIR OR
            NOT EXISTS "${LIBRAPTORQ_DIR}/RaptorQ/RaptorQ_v1_hdr.hpp")
        message(FATAL_ERROR
                "WITH_FEC=ON but libRaptorQ not found. "
                "Pass -DLIBRAPTORQ_DIR=/path/to/libRaptorQ/src")
    endif()

    find_path(EIGEN3_INCLUDE_DIR Eigen/Core
            PATHS /usr/include/eigen3 /usr/local/include/eigen3
            "${LIBRAPTORQ_DIR}/../external/eigen3")
    if(NOT EIGEN3_INCLUDE_DIR)
        message(FATAL_ERROR "Eigen3 not found (needed by libRaptorQ). "
                "Install libeigen3-dev or point to bundled copy.")
    endif()

    message(STATUS "FEC enabled")
    message(STATUS "  libRaptorQ: ${LIBRAPTORQ_DIR}")
    message(STATUS "  Eigen3:     ${EIGEN3_INCLUDE_DIR}")
endif()
if(WITH_TUI)
    message(STATUS "TUI enabled")
endif()

########################################################################
# Original test executables
########################################################################
list(APPEND test_sim_srcs         test_sim.cpp)
list(APPEND test_tx_srcs          test_tx.cpp)
list(APPEND test_rx_srcs          test_rx.cpp)
list(APPEND test_transceiver_srcs simple_transceiver.cpp)

add_executable(sim         ${test_sim_srcs})
add_executable(transmitter ${test_tx_srcs})
add_executable(receiver    ${test_rx_srcs})
add_executable(transceiver ${test_transceiver_srcs})

target_link_libraries(sim         fun_ofdm)
target_link_libraries(transmitter fun_ofdm)
target_link_libraries(receiver    fun_ofdm)
target_link_libraries(transceiver fun_ofdm)

########################################################################
# Video bridges (H.265). GStreamer is ALWAYS needed; FEC/TUI/etc optional.
########################################################################
add_executable(video_tx video_tx.cpp)
add_executable(video_rx video_rx.cpp)

target_link_libraries(video_tx fun_ofdm ${OpenCV_LIBS})
target_link_libraries(video_rx fun_ofdm ${OpenCV_LIBS})

# GStreamer applies to BOTH bridges UNCONDITIONALLY (even a bare build uses it).
foreach(tgt video_tx video_rx)
    target_include_directories(${tgt} PRIVATE ${GST_INCLUDE_DIRS})
    target_link_libraries(${tgt} ${GST_LIBRARIES})
    target_compile_options(${tgt} PRIVATE ${GST_CFLAGS_OTHER})
endforeach()

# C++14 + threads + local headers whenever ANY instrumentation/FEC is on.
foreach(tgt video_tx video_rx)
    if(WITH_FEC OR WITH_TUI OR WITH_OVERLAY OR WITH_DIAG OR WITH_AGC)
        target_compile_features(${tgt} PRIVATE cxx_std_14)
        target_include_directories(${tgt} PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR})
        target_link_libraries(${tgt} Threads::Threads)
    endif()
    if(WITH_FEC)
        target_compile_definitions(${tgt} PRIVATE WITH_FEC)
        target_compile_options(${tgt} PRIVATE -include cstdint)
        target_include_directories(${tgt} PRIVATE
                ${LIBRAPTORQ_DIR}
                ${EIGEN3_INCLUDE_DIR})
    endif()
    if(WITH_TUI)
        target_compile_definitions(${tgt} PRIVATE WITH_TUI)
    endif()
endforeach()

# RX-only feature definitions.
if(WITH_OVERLAY)
    target_compile_definitions(video_rx PRIVATE WITH_OVERLAY)
endif()
if(WITH_DIAG)
    target_compile_definitions(video_rx PRIVATE WITH_DIAG)
endif()
if(WITH_AGC)
    target_compile_definitions(video_rx PRIVATE WITH_AGC)
endif()
