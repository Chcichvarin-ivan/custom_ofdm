########################################################################
# Examples CMakeLists.txt
#
#
########################################################################
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
########################################################################
# Include directories so cmake knows where .h files live
########################################################################
include_directories (${PROJECT_SOURCE_DIR}/src)

########################################################################
# Example Sources
########################################################################
list(APPEND test_sim_srcs
    test_sim.cpp
)

list(APPEND test_tx_srcs
    test_tx.cpp
)

list(APPEND test_rx_srcs
    test_rx.cpp
)

list(APPEND test_transceiver_srcs
    simple_transceiver.cpp
)

########################################################################
# Create executables
########################################################################
add_executable(sim         ${test_sim_srcs})
add_executable(transmitter ${test_tx_srcs})
add_executable(receiver    ${test_rx_srcs})
add_executable(transceiver ${test_transceiver_srcs})
add_executable(video_tx video_tx.cpp)
target_link_libraries(video_tx fun_ofdm ${OpenCV_LIBS})
add_executable(video_rx video_rx.cpp)
target_link_libraries(video_rx fun_ofdm ${OpenCV_LIBS})

########################################################################
# Link executables to fun_ofdm library
########################################################################
target_link_libraries(sim         fun_ofdm)
target_link_libraries(transmitter fun_ofdm)
target_link_libraries(receiver    fun_ofdm)
target_link_libraries(transceiver fun_ofdm)

