/*
    SPDX-FileCopyrightText: 2019-2023 Jakub Stankowski <jakub.stankowski@put.poznan.pl>
    SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once
#include "xCommonDefJPEG.h"
#include "xJFIF.h"
#include "xJPEG_Huffman.h"
#include "xBitstream.h"

namespace PMBB_NAMESPACE::JPEG {

//=====================================================================================================================================================================================

class xEntropyCommon
{
public:
  using tLDCs = std::array<int16, xJPEG_Constants::c_MaxComponents>;
  static constexpr tLDCs c_InitLastDCs = { 0, 0, 0, 0 };

#if X_SIMD_CAN_USE_AVX512
#define X_CAN_USE_AVX512 1
  static int32 findLastNonZeroAVX512(const int16* ScanCoeff);
#else //X_SIMD_CAN_USE_AVX512
#define X_CAN_USE_AVX512 0
#endif //X_SIMD_CAN_USE_AVX512

#if X_SIMD_CAN_USE_AVX
#define X_CAN_USE_AVX 1
  static int32 findLastNonZeroAVX(const int16* ScanCoeff);
#else //X_SIMD_CAN_USE_AVX
#define X_CAN_USE_AVX 0
#endif //X_SIMD_CAN_USE_AVX

#if X_SIMD_CAN_USE_SSE
#define X_CAN_USE_SSE 1
  static int32 findLastNonZeroSSE(const int16* ScanCoeff);
#else //X_SIMD_CAN_USE_SSE
#define X_CAN_USE_SSE 0
#endif //X_SIMD_CAN_USE_SSE
    
  static int32 findLastNonZeroSTD(const int16* ScanCoeff);

public:
#if X_CAN_USE_AVX512
  static inline int32 findLastNonZero(const int16* ScanCoeff) { return findLastNonZeroAVX512(ScanCoeff); }
#elif X_CAN_USE_AVX
  static inline int32 findLastNonZero(const int16* ScanCoeff) { return findLastNonZeroAVX   (ScanCoeff); }
#elif X_CAN_USE_SSE
  static inline int32 findLastNonZero(const int16* ScanCoeff) { return findLastNonZeroSSE   (ScanCoeff); }
#else
  static inline int32 findLastNonZero(const int16* ScanCoeff) { return findLastNonZeroSTD   (ScanCoeff); }
#endif

#undef X_CAN_USE_AVX512
#undef X_CAN_USE_AVX
#undef X_CAN_USE_SSE
};

//=====================================================================================================================================================================================

class xEntropyDecoder : public xEntropyCommon
{
protected:
  xHuffDecoder*    m_HuffDecoderDC[xJPEG_Constants::c_MaxHuffTabs];
  xHuffDecoder*    m_HuffDecoderAC[xJPEG_Constants::c_MaxHuffTabs];
  xBitstreamReader m_Bitstream;
  tLDCs            m_LastDC;

public:
  xEntropyDecoder () { memset(m_HuffDecoderDC, 0, sizeof(m_HuffDecoderDC)); memset(m_HuffDecoderAC, 0, sizeof(m_HuffDecoderAC)); }
  ~xEntropyDecoder() { UnInit(); }
  bool Init  (std::vector<xJFIF::xHuffTable>& HuffTables);
  void UnInit();

  void StartSlice (xByteBuffer* ByteBuffer);
  void FinishSlice();
  void DecodeBlock(int16* ScanCoeff, eCmp Cmp, int32 HuffTableIdDC, int32 HuffTableIdAC);
};

//=====================================================================================================================================================================================

class xEntropyEncoder : public xEntropyCommon
{
protected:
  xHuffEncoderDC*  m_HuffEncoderDC[xJPEG_Constants::c_MaxHuffTabs];
  xHuffEncoderAC*  m_HuffEncoderAC[xJPEG_Constants::c_MaxHuffTabs];
  xBitstreamWriter m_Bitstream;
  tLDCs            m_LastDC;

public:
  xEntropyEncoder () { memset(m_HuffEncoderDC, 0, sizeof(m_HuffEncoderDC)); memset(m_HuffEncoderAC, 0, sizeof(m_HuffEncoderAC)); }
  ~xEntropyEncoder() { UnInit(); }
  bool  Init  (std::vector<xJFIF::xHuffTable>& HuffTables);
  void  UnInit();

  void  StartSlice (xByteBuffer* ByteBuffer);
  void  FinishSlice();
  void  StartChunk (xByteBuffer* ByteBuffer, const tLDCs& LastDCs);
  int32 FinishChunk();
  void  EncodeBlock(const int16* ScanCoeff, eCmp Cmp, int32 HuffTableIdDC, int32 HuffTableIdAC);
};

//=====================================================================================================================================================================================

class xEntropyEncoderDefault : public xEntropyCommon
{
protected:
  xBitstreamWriter m_Bitstream;
  tLDCs            m_LastDC;

public:
  xEntropyEncoderDefault () { }
  ~xEntropyEncoderDefault() { }
  bool  Init  (std::vector<xJFIF::xHuffTable>& /*HuffTables*/) { return true; }
  void  UnInit() {}

  void  StartSlice (xByteBuffer* ByteBuffer);
  void  FinishSlice();
  void  EncodeBlock(int16* ScanCoeff, eCmp Cmp) { Cmp == eCmp::LM ? xEncodeBlockL(ScanCoeff) : xEncodeBlockC(ScanCoeff, Cmp); }

protected:
  void  xEncodeBlockL(const int16* ScanCoeff          );
  void  xEncodeBlockC(const int16* ScanCoeff, eCmp Cmp);
};

//=====================================================================================================================================================================================

class xEntropyEstimator : public xEntropyCommon
{
protected:
  xHuffEstimatorDC* m_HuffEstimatorDC[xJPEG_Constants::c_MaxHuffTabs];
  xHuffEstimatorAC* m_HuffEstimatorAC[xJPEG_Constants::c_MaxHuffTabs];

public:
  xEntropyEstimator () { memset(m_HuffEstimatorDC, 0, sizeof(m_HuffEstimatorDC)); memset(m_HuffEstimatorAC, 0, sizeof(m_HuffEstimatorAC)); }
  ~xEntropyEstimator() { UnInit(); }
  bool  Init  (std::vector<xJFIF::xHuffTable>& HuffTables);
  void  UnInit();

  int32 EstimateBlock  (const int16* ScanCoeff, int32 LastDC, int32 HuffTableIdDC, int32 HuffTableIdAC) const;
  int32 EstimateBlockDC(const int16* ScanCoeff, int32 LastDC, int32 HuffTableIdDC                     ) const;
  int32 EstimateBlockAC(const int16* ScanCoeff,                                    int32 HuffTableIdAC) const;
};

//=====================================================================================================================================================================================

class xEntropyEstimatorDefault : public xEntropyCommon
{
public:
  xEntropyEstimatorDefault () { }
  ~xEntropyEstimatorDefault() { UnInit(); }
  bool  Init  (std::vector<xJFIF::xHuffTable>& /*HuffTables*/) { return true; };
  void  UnInit() {};

  static int32 EstimateBlock(const int16* ScanCoeff, int32 LastDC, eCmp Cmp);
protected:
  static int32 xEstimateBlockL(const int16* ScanCoeff, int32 LastDC);
  static int32 xEstimateBlockC(const int16* ScanCoeff, int32 LastDC);
};

//=====================================================================================================================================================================================

class xEntropyCounter : public xEntropyCommon
{
protected:
  xHuffCounterDC* m_HuffCounterDC[xJPEG_Constants::c_MaxHuffTabs];
  xHuffCounterAC* m_HuffCounterAC[xJPEG_Constants::c_MaxHuffTabs];

public:
  xEntropyCounter () { memset(m_HuffCounterDC, 0, sizeof(m_HuffCounterDC)); memset(m_HuffCounterAC, 0, sizeof(m_HuffCounterAC)); }
  ~xEntropyCounter() { UnInit(); }
  bool  Init  (std::vector<xJFIF::xHuffTable>& HuffTables);
  void  UnInit();

  void  ZeroCounters();
  void  CountBlock  (const int16* ScanCoeff, int32 LastDC, int32 HuffTableIdDC, int32 HuffTableIdAC);
  void  AddCounters (const xEntropyCounter& Other);

  const uint32* getSymbolCountDC(int32 HuffTableIdDC) { return m_HuffCounterDC[HuffTableIdDC]->getSymbolCount(); }
  const uint32* getSymbolCountAC(int32 HuffTableIdAC) { return m_HuffCounterAC[HuffTableIdAC]->getSymbolCount(); }

  const uint32* getSymbolCount(xJFIF::xHuffTable::eHuffClass HuffClass, int32 HuffTableId)
  {
    if(HuffClass == xJFIF::xHuffTable::eHuffClass::DC) { return m_HuffCounterDC[HuffTableId]->getSymbolCount(); }
    if(HuffClass == xJFIF::xHuffTable::eHuffClass::AC) { return m_HuffCounterAC[HuffTableId]->getSymbolCount(); }
    return nullptr;
  }
};

//=====================================================================================================================================================================================

} //end of namespace PMBB::JPEG