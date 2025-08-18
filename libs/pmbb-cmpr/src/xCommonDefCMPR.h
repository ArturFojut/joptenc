/*
    SPDX-FileCopyrightText: 2019-2023 Jakub Stankowski <jakub.stankowski@put.poznan.pl>
    SPDX-License-Identifier: GPL-3.0-or-later
*/

#pragma once

#include "xCommonDefCORE.h"

namespace PMBB_NAMESPACE {

//===============================================================================================================================================================================================================
// Compile time settings
//===============================================================================================================================================================================================================
#define PMBB_CMPR_TRACE_LIKE_HM  0
#define PMBB_CMPR_TRACE_FLUSH    1
#define PMBB_CMPB_CABAC_BYTEBUFF 1

//===============================================================================================================================================================================================================
// Bitstream class assumes big endian
//===============================================================================================================================================================================================================
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#error Only little-endian targets are supported
#endif

#if X_PMBB_CPP20
static_assert(std::endian::native == std::endian::big);
#endif

//===============================================================================================================================================================================================================
// Leading zero count
//===============================================================================================================================================================================================================
#if X_PMBB_CPP20
static inline uint16 xLZCNT(uint16 Val) { return std::countl_zero(Val); }
static inline uint32 xLZCNT(uint32 Val) { return std::countl_zero(Val); }
static inline uint64 xLZCNT(uint64 Val) { return std::countl_zero(Val); }
#else //X_PMBB_CPP20
#if defined(X_PMBB_ARCH_AMD64)
#if X_SIMD_HAS_AVX
#if defined(X_PMBB_COMPILER_MSVC)
static inline uint16 xLZCNT(uint16 Val) { return __lzcnt16(Val); }
static inline uint32 xLZCNT(uint32 Val) { return __lzcnt  (Val); }
static inline uint64 xLZCNT(uint64 Val) { return __lzcnt64(Val); }
#elif defined(X_PMBB_COMPILER_GCC)
static inline uint16 xLZCNT(uint16 Val) { return __builtin_ia32_lzcnt_u16(Val); }
static inline uint32 xLZCNT(uint32 Val) { return __builtin_ia32_lzcnt_u32(Val); }
static inline uint64 xLZCNT(uint64 Val) { return __builtin_ia32_lzcnt_u64(Val); }
#elif defined(X_PMBB_COMPILER_CLANG)
static inline uint16 xLZCNT(uint16 Val) { return __lzcnt16(Val); }
static inline uint32 xLZCNT(uint32 Val) { return __lzcnt32(Val); }
static inline uint64 xLZCNT(uint64 Val) { return __lzcnt64(Val); }
#else
#error Unrecognized compiler
#endif
#else //X_SIMD_HAS_AVX
#if defined(X_PMBB_COMPILER_MSVC)
static inline uint16 xLZCNT(uint16 Val) { unsigned long Index; return (uint16)(_BitScanReverse  (&Index, Val) ? (15 - Index) : 16); }
static inline uint32 xLZCNT(uint32 Val) { unsigned long Index; return (uint32)(_BitScanReverse  (&Index, Val) ? (31 - Index) : 32); }
static inline uint64 xLZCNT(uint64 Val) { unsigned long Index; return (uint64)(_BitScanReverse64(&Index, Val) ? (63 - Index) : 64); }
#elif (defined(X_PMBB_COMPILER_GCC) || defined(X_PMBB_COMPILER_CLANG))
static inline uint16 xLZCNT(uint16 Val) { return Val ? 15-(uint16)__bsrd(Val) : 16; }
static inline uint32 xLZCNT(uint32 Val) { return Val ? 31-(uint32)__bsrd(Val) : 32; }
static inline uint64 xLZCNT(uint64 Val) { return Val ? 63-(uint64)__bsrq(Val) : 64; }
#else
#error Unrecognized compiler
#endif
#endif //X_SIMD_HAS_AVX
#endif //X_PMBB_ARCH_AMD64

#if defined(X_PMBB_ARCH_ARM64)
#if defined(X_PMBB_COMPILER_MSVC)
static inline uint16 xLZCNT(uint16 Val) { return _CountLeadingZeros  (Val) - 16; }
static inline uint32 xLZCNT(uint32 Val) { return _CountLeadingZeros  (Val); }
static inline uint64 xLZCNT(uint64 Val) { return _CountLeadingZeros64(Val); }
#elif defined(X_PMBB_COMPILER_GCC) || defined(X_PMBB_COMPILER_CLANG)
#if __has_builtin(__builtin_clzg)
static inline uint16 xLZCNT(uint16 Val) { return __builtin_clzg(Val); }
static inline uint32 xLZCNT(uint32 Val) { return __builtin_clzg(Val); }
static inline uint64 xLZCNT(uint64 Val) { return __builtin_clzg(Val); }
#else
static inline uint16 xLZCNT(uint16 Val) { return __builtin_clzl (Val) - 16; }
static inline uint32 xLZCNT(uint32 Val) { return __builtin_clzl (Val); }
static inline uint64 xLZCNT(uint64 Val) { return __builtin_clzll(Val); }
#endif
#else
#error Unrecognized compiler
#endif
#endif //X_PMBB_ARCH_ARM64
#endif //X_PMBB_CPP20

//===============================================================================================================================================================================================================
// Num significant bits (similar to xLog2, but returns 0 for Val==0, uses faster lzcnt
//===============================================================================================================================================================================================================
static inline uint16 xNumSignificantBits(uint16 Val) { return 16 - xLZCNT(Val); }
static inline uint32 xNumSignificantBits(uint32 Val) { return 32 - xLZCNT(Val); }
static inline uint64 xNumSignificantBits(uint64 Val) { return 32 - xLZCNT(Val); }

//===============================================================================================================================================================================================================
// Byte swap
//===============================================================================================================================================================================================================
#if X_PMBB_CPP20
static inline uint16 xSwapBytes16(uint16 Value) { return std::byteswap(Value); }
static inline  int16 xSwapBytes16( int16 Value) { return std::byteswap(Value); }
static inline uint32 xSwapBytes32(uint32 Value) { return std::byteswap(Value); }
static inline  int32 xSwapBytes32( int32 Value) { return std::byteswap(Value); }
static inline uint64 xSwapBytes64(uint64 Value) { return std::byteswap(Value); }
static inline  int64 xSwapBytes64( int64 Value) { return std::byteswap(Value); }
#else //X_PMBB_CPP20
  #if X_PMBB_COMPILER_MSVC
static inline uint16 xSwapBytes16(uint16 Value) { return _byteswap_ushort(Value); }
static inline  int16 xSwapBytes16( int16 Value) { return _byteswap_ushort(Value); }
static inline uint32 xSwapBytes32(uint32 Value) { return _byteswap_ulong (Value); }
static inline  int32 xSwapBytes32( int32 Value) { return _byteswap_ulong (Value); }
static inline uint64 xSwapBytes64(uint64 Value) { return _byteswap_uint64(Value); }
static inline  int64 xSwapBytes64( int64 Value) { return _byteswap_uint64(Value); }
  #elif (X_PMBB_COMPILER_GCC || X_PMBB_COMPILER_CLANG)
static inline uint16 xSwapBytes16(uint16 Value) { return __builtin_bswap16(Value); }
static inline  int16 xSwapBytes16( int16 Value) { return __builtin_bswap16(Value); }
static inline uint32 xSwapBytes32(uint32 Value) { return __builtin_bswap32(Value); }
static inline  int32 xSwapBytes32( int32 Value) { return __builtin_bswap32(Value); }
static inline uint64 xSwapBytes64(uint64 Value) { return __builtin_bswap64(Value); }
static inline  int64 xSwapBytes64( int64 Value) { return __builtin_bswap64(Value); }
  #else
    #error Unrecognized compiler
  #endif
#endif //X_PMBB_CPP20

//===============================================================================================================================================================================================================

} //end of namespace PMBB
