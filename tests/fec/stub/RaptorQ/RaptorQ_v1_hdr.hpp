// TEST STUB of libRaptorQ — window-logic tests only. NOT the real library.
#pragma once
#include <cstddef>
#include <cstdint>
namespace RaptorQ__v1 {
enum class Block_Size { Block_32 };
enum class Dec_Report { COMPLETE };
enum class Error { NONE, NEED_DATA };
enum class Fill_With_Zeros { NO };
template <typename In, typename Out>
class Decoder {
public:
    Decoder(Block_Size, size_t, Dec_Report) {}
    bool add_symbol(In&, In&, uint32_t) { return true; }
    void end_of_input(Fill_With_Zeros) {}
    struct WaitRes { Error error; };
    WaitRes wait_sync() { return {Error::NEED_DATA}; }   // never decodes in tests
    struct DecRes { size_t written; };
    DecRes decode_bytes(Out&, Out&, int, int) { return {0}; }
};
} // namespace RaptorQ__v1
