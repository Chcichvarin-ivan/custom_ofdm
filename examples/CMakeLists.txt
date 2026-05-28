########################################################################
# Examples CMakeLists.txt — with optional RaptorQ FEC for video bridges
########################################################################
cmake_policy(SET CMP0167 OLD)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

########################################################################
# Include directories so cmake knows where .h files live
########################################################################
include_directories(${PROJECT_SOURCE_DIR}/src)

########################################################################
# FEC configuration (libRaptorQ is header-only)
#
#   Enable:   cmake -DWITH_FEC=ON -DLIBRAPTORQ_DIR=/path/to/libRaptorQ/src ..
#   Disable:  cmake ..   (default)
#
# The FEC shim headers (fec_common.h, fec_encoder.h, fec_decoder.h) are
# expected in this examples/ directory alongside video_tx.cpp / video_rx.cpp.
########################################################################
option(WITH_FEC "Build video bridges with RaptorQ FEC" OFF)

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

    # libRaptorQ needs Eigen3. Prefer system, fall back to bundled copy.
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

    find_package(Threads REQUIRED)
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
# Video bridges (FEC optional)
########################################################################
add_executable(video_tx video_tx.cpp)
add_executable(video_rx video_rx.cpp)

target_link_libraries(video_tx fun_ofdm ${OpenCV_LIBS})
target_link_libraries(video_rx fun_ofdm ${OpenCV_LIBS})

if(WITH_FEC)
    foreach(tgt video_tx video_rx)
        # libRaptorQ header-only needs C++14 and -include cstdint for modern gcc
        target_compile_definitions(${tgt} PRIVATE WITH_FEC)
        target_compile_features(${tgt} PRIVATE cxx_std_14)
        target_compile_options(${tgt} PRIVATE -include cstdint)
        target_include_directories(${tgt} PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}     # for fec_*.h next to the sources
                ${LIBRAPTORQ_DIR}
                ${EIGEN3_INCLUDE_DIR})
        target_link_libraries(${tgt} Threads::Threads)
    endforeach()
endif()