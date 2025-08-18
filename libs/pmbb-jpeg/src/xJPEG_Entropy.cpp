/*
    SPDX-FileCopyrightText: 2020-2024 Jakub Stankowski <jakub.stankowski@put.poznan.pl>
    SPDX-License-Identifier: BSD-3-Clause
*/
#include "xJPEG_Entropy.h"
#include "xJPEG_HuffmanDefault.h"

namespace PMBB_NAMESPACE::JPEG {

//=====================================================================================================================================================================================

#if X_SIMD_CAN_USE_AVX512
int32 xEntropyCommon::findLastNonZeroAVX512(const int16* ScanCoeff)
{
  //begin with 2nd half
  uint32 MaskV1 = _mm512_cmpneq_epi16_mask(_mm512_loadu_si512((__m512i*)(ScanCoeff + 32)), _mm512_setzero_si512());
  if(MaskV1) { return (63 - xLZCNT(MaskV1)); }
  //continue with 1st half
  uint32 MaskV0 = _mm512_cmpneq_epi16_mask(_mm512_loadu_si512((__m512i*)(ScanCoeff     )), _mm512_setzero_si512());
  if(MaskV0) { return (31 - xLZCNT(MaskV0)); }
  //empty block but treeat DC as always existing
  return 0;
}
#endif //X_SIMD_CAN_USE_AVX512

#if X_SIMD_CAN_USE_AVX
int32 xEntropyCommon::findLastNonZeroAVX(const int16* ScanCoeff)
{
  for(int32 i = 64 - 32; i >= 0; i -= 32)
  {
    __m256i CoeffsA = _mm256_loadu_si256((__m256i*) & ScanCoeff[i     ]);
    __m256i CoeffsB = _mm256_loadu_si256((__m256i*) & ScanCoeff[i + 16]);
    __m256i Coeffs  = _mm256_packs_epi16(CoeffsA, CoeffsB);
    __m256i MaskErV = _mm256_cmpeq_epi8 (Coeffs, _mm256_setzero_si256());    
    uint32  MaskEr  = (~_mm256_movemask_epi8(MaskErV)) & 0xFFFFFFFF;
    if(MaskEr)
    {
      uint32 Mask = ((MaskEr & 0xFF0000FF) | ((MaskEr & 0x00FF0000) >> 8) | ((MaskEr & 0x0000FF00) << 8)); //swap central bytes
      return (31 - (int32)xLZCNT(Mask)) + i;
    }
  }
  //empty block but treeat DC as always existing
  return 0;
}
#endif //X_SIMD_CAN_USE_AVX

#if X_SIMD_CAN_USE_SSE
int32 xEntropyCommon::findLastNonZeroSSE(const int16* ScanCoeff)
{
  for(int32 i = 64 - 16; i >= 0; i-=16)
  {
    __m128i CoeffsA = _mm_loadu_si128((__m128i*) & ScanCoeff[i    ]);
    __m128i CoeffsB = _mm_loadu_si128((__m128i*) & ScanCoeff[i + 8]);
    __m128i Coeffs  = _mm_packs_epi16(CoeffsA, CoeffsB);
    __m128i MaskV   = _mm_cmpeq_epi8 (Coeffs, _mm_setzero_si128());
    uint32  Mask    = (~_mm_movemask_epi8(MaskV)) & 0xFFFF;
    if(Mask) { return (31 - (int32)xLZCNT(Mask)) + i; }
  }
  //empty block but treeat DC as always existing
  return 0;
}
#endif //X_SIMD_CAN_USE_SSE

int32 xEntropyCommon::findLastNonZeroSTD(const int16* ScanCoeff)
{
  for(int32 i = 63; i >= 0; i--) { if(ScanCoeff[i] != 0) { return i; } }
  //empty block but treeat DC as always existing
  return 0;
}

//=====================================================================================================================================================================================

bool xEntropyDecoder::Init(std::vector<xJFIF::xHuffTable>& HuffTables)
{
  bool Result = true;
  for(xJFIF::xHuffTable& HuffTable : HuffTables)
  {
    xJFIF::xHuffTable::eHuffClass HuffTableClass = HuffTable.getClass();
    int32                         HuffTableId    = HuffTable.getIdx  ();
    switch(HuffTableClass)
    {
      case xJFIF::xHuffTable::eHuffClass::DC:
        if(m_HuffDecoderDC[HuffTableId] == nullptr) { m_HuffDecoderDC[HuffTableId] = new xHuffDecoder; }
        Result &= m_HuffDecoderDC[HuffTableId]->init(HuffTable);
        break;
      case xJFIF::xHuffTable::eHuffClass::AC:
        if(m_HuffDecoderAC[HuffTableId] == nullptr) { m_HuffDecoderAC[HuffTableId] = new xHuffDecoder; }
        Result &= m_HuffDecoderAC[HuffTableId]->init(HuffTable);
        break;
      default: Result = false; break;
    }
  }
  return Result;
}
void xEntropyDecoder::UnInit()
{
  for(int32 HuffTableId=0; HuffTableId < xJPEG_Constants::c_MaxHuffTabs; HuffTableId++)
  {
    if(m_HuffDecoderDC[HuffTableId] != nullptr) { delete m_HuffDecoderDC[HuffTableId]; m_HuffDecoderDC[HuffTableId] = nullptr; }
    if(m_HuffDecoderAC[HuffTableId] != nullptr) { delete m_HuffDecoderAC[HuffTableId]; m_HuffDecoderAC[HuffTableId] = nullptr; }
  }
}
void xEntropyDecoder::StartSlice(xByteBuffer* ByteBuffer)
{
  m_LastDC = c_InitLastDCs;
  m_Bitstream.bindByteBuffer(ByteBuffer);
  m_Bitstream.init();
}
void xEntropyDecoder::FinishSlice()
{
  m_Bitstream.readAlign();
  m_Bitstream.uninit();
  m_Bitstream.unbindByteBuffer();
}
void xEntropyDecoder::DecodeBlock(int16* ScanCoeff, eCmp Cmp, int32 HuffTableIdDC, int32 HuffTableIdAC)
{
  memset(ScanCoeff, 0, xJPEG_Constants::c_BlockArea * sizeof(int16));

  //DC coefficient
  int32 DeltaDC        = m_HuffDecoderDC[HuffTableIdDC]->readDC(&m_Bitstream);
  int16 DC             = (int16)(DeltaDC + (int32)m_LastDC[(int32)Cmp]);
  m_LastDC[(int32)Cmp] = (int16)DC;
  ScanCoeff[0] = DC;

  //AC coefficients
  xHuffDecoder* HD = m_HuffDecoderAC[HuffTableIdAC];
  for(int32 i=1; i < 64; i++)
  {
    int32 AC = HD->readPrefix(&m_Bitstream);
    int32 R = AC >> 4;
    int32 V = AC & 0x0F;

    if(V)
    {
      i += R;
      AC = HD->readSufix(&m_Bitstream, V);
      ScanCoeff[i] = (int16)AC;
    }
    else
    {
      if(R != 15) break;
      i += 15;
    }
  }
}

//=====================================================================================================================================================================================

bool xEntropyEncoder::Init(std::vector<xJFIF::xHuffTable>& HuffTables)
{
  bool Result = true;
  for(xJFIF::xHuffTable& HuffTable : HuffTables)
  {
    xJFIF::xHuffTable::eHuffClass HuffTableClass = HuffTable.getClass();
    int32                         HuffTableId    = HuffTable.getIdx  ();
    switch(HuffTableClass)
    {
      case xJFIF::xHuffTable::eHuffClass::DC:
        if(m_HuffEncoderDC[HuffTableId] == nullptr) { m_HuffEncoderDC[HuffTableId] = new xHuffEncoderDC; }
        Result &= m_HuffEncoderDC[HuffTableId]->init(HuffTable);
        break;
      case xJFIF::xHuffTable::eHuffClass::AC:
        if(m_HuffEncoderAC[HuffTableId] == nullptr) { m_HuffEncoderAC[HuffTableId] = new xHuffEncoderAC; }
        Result &= m_HuffEncoderAC[HuffTableId]->init(HuffTable);
        break;
      default: Result = false; break;
    }
  }
  return Result;
}
void xEntropyEncoder::UnInit()
{
  for(int32 HuffTableId=0; HuffTableId < xJPEG_Constants::c_MaxHuffTabs; HuffTableId++)
  {
    if(m_HuffEncoderDC[HuffTableId] != nullptr) { delete m_HuffEncoderDC[HuffTableId]; m_HuffEncoderDC[HuffTableId] = nullptr; }
    if(m_HuffEncoderAC[HuffTableId] != nullptr) { delete m_HuffEncoderAC[HuffTableId]; m_HuffEncoderAC[HuffTableId] = nullptr; }
  }
}
void xEntropyEncoder::StartSlice(xByteBuffer* ByteBuffer)
{
  m_LastDC = c_InitLastDCs;
  m_Bitstream.bindByteBuffer(ByteBuffer);
  m_Bitstream.init();
}
void xEntropyEncoder::FinishSlice()
{
  m_Bitstream.writeAlign(1);
  m_Bitstream.uninit();
  m_Bitstream.unbindByteBuffer();
}
void xEntropyEncoder::StartChunk(xByteBuffer* ByteBuffer, const tLDCs& LastDCs)
{
  m_LastDC = LastDCs;
  m_Bitstream.bindByteBuffer(ByteBuffer);
  m_Bitstream.init();
}
int32 xEntropyEncoder::FinishChunk()
{
  int32 NumAlignmentBits = m_Bitstream.writeAlign(0);
  m_Bitstream.uninit();
  m_Bitstream.unbindByteBuffer();
  return NumAlignmentBits;
}
void xEntropyEncoder::EncodeBlock(const int16* ScanCoeff, eCmp Cmp, int32 HuffTableIdDC, int32 HuffTableIdAC)
{
  //DC coefficient
  int32 DC             = ScanCoeff[0];
  int32 DeltaDC        = DC - (int32)m_LastDC[(int32)Cmp];
  m_LastDC[(int32)Cmp] = (int16)DC;

  int32 SignMaskDC = DeltaDC >> 31;                       // make a mask of the sign bit
  int32 AbsDeltaDC = (DeltaDC ^ SignMaskDC) - SignMaskDC; // toggle the bits and add one if value is negative
  int32 NumBitsDC  = xNumSignificantBits((uint32)AbsDeltaDC);
  int32 RemainDC   = (DeltaDC + SignMaskDC) & ((1 << NumBitsDC) - 1);  // subtract one if value was negative and mask off any extra bits in code
  m_HuffEncoderDC[HuffTableIdDC]->writeDC(&m_Bitstream, NumBitsDC, RemainDC);

  //AC coefficients
  xHuffEncoderAC* HE = m_HuffEncoderAC[HuffTableIdAC];
  int32 LastNonZero = findLastNonZero(ScanCoeff);
  int32 RunLength   = 0;
  for(int32 i=1; i <= LastNonZero; i++)
  {
    int32 AC = ScanCoeff[i];

    //nothing to encode
    if(AC == 0) { RunLength++; continue; }

    //if run length > 15, must emit special run-length-16 codes (0xF0)
    while (RunLength > 15) { HE->writeZRL(&m_Bitstream); RunLength -= 16; }

    //Emit Huffman symbol for run length / number of bits
    int32 SignMaskAC = AC >> 31;                       // make a mask of the sign bit
    int32 AbsAC      = (AC ^ SignMaskAC) - SignMaskAC; // toggle the bits and add one if value is negative
    int32 NumBitsAC  = xNumSignificantBits((uint32)AbsAC);
    int32 CodeAC     = (RunLength << 4) + NumBitsAC;
    int32 RemainAC   = (AC + SignMaskAC) & ((1 << NumBitsAC) - 1);  // subtract one if value was negative and mask off any extra bits in code
    HE->writeAC(&m_Bitstream, CodeAC, NumBitsAC, RemainAC);

    //reset run length
    RunLength = 0;
  }
  //If the last coef(s) were zero, emit an end-of-block code
  if (LastNonZero < 63) { HE->writeEOB(&m_Bitstream); }
}

//=====================================================================================================================================================================================

void xEntropyEncoderDefault::StartSlice(xByteBuffer* ByteBuffer)
{
  m_LastDC = c_InitLastDCs;
  m_Bitstream.bindByteBuffer(ByteBuffer);
  m_Bitstream.init();
}
void xEntropyEncoderDefault::FinishSlice()
{
  m_Bitstream.writeAlign(1);
  m_Bitstream.uninit();
  m_Bitstream.unbindByteBuffer();
}
//void xEntropyEncoderDefault::EncodeBlock(int16* ScanCoeff, eCmp Cmp)
//{
//  //DC coefficient
//  int32 DC             = ScanCoeff[0];
//  int32 DeltaDC        = DC - (int32)m_LastDC[(int32)Cmp];
//  m_LastDC[(int32)Cmp] = (int16)DC;
//
//  int32 SignMaskDC = DeltaDC >> 31;                       // make a mask of the sign bit
//  int32 AbsDeltaDC = (DeltaDC ^ SignMaskDC) - SignMaskDC; // toggle the bits and add one if value is negative
//  int32 NumBitsDC  = xNumBits(AbsDeltaDC);
//  int32 RemainDC   = (DeltaDC + SignMaskDC) & ((1 << NumBitsDC) - 1);  // subtract one if value was negative and mask off any extra bits in code
//  if(Cmp == eCmp::LM) { xHuffDefaultEncoder::writeLumaDC  (&m_Bitstream, NumBitsDC, RemainDC); }
//  else                { xHuffDefaultEncoder::writeChromaDC(&m_Bitstream, NumBitsDC, RemainDC); }
//
//  //AC coefficients
//  int32 LastNonZero = findLastNonZero(ScanCoeff);
//  int32 RunLength   = 0;
//  for(int32 i=1; i <= LastNonZero; i++)
//  {
//    int32 AC = ScanCoeff[i];
//
//    //nothing to encode
//    if(AC == 0) { RunLength++; continue; }
//
//    //if run length > 15, must emit special run-length-16 codes (0xF0)
//    while (RunLength > 15)
//    { 
//      if(Cmp == eCmp::LM) { xHuffDefaultEncoder::writeLumaZRL  (&m_Bitstream); }
//      else                { xHuffDefaultEncoder::writeChromaZRL(&m_Bitstream); }
//      RunLength -= 16;
//    }
//
//    //Emit Huffman symbol for run length / number of bits
//    int32 SignMaskAC = AC >> 31;                       // make a mask of the sign bit
//    int32 AbsAC      = (AC ^ SignMaskAC) - SignMaskAC; // toggle the bits and add one if value is negative
//    int32 NumBitsAC  = xNumBits(AbsAC);
//    int32 CodeAC     = (RunLength << 4) + NumBitsAC;
//    int32 RemainAC   = (AC + SignMaskAC) & ((1 << NumBitsAC) - 1);  // subtract one if value was negative and mask off any extra bits in code
//    if(Cmp == eCmp::LM) { xHuffDefaultEncoder::writeLumaAC  (&m_Bitstream, RunLength, NumBitsAC, RemainAC); }
//    else                { xHuffDefaultEncoder::writeChromaAC(&m_Bitstream, RunLength, NumBitsAC, RemainAC); }
//
//    //reset run length
//    RunLength = 0;
//  }
//  //If the last coef(s) were zero, emit an end-of-block code
//  if (LastNonZero < 63)
//  {
//    if(Cmp == eCmp::LM) { xHuffDefaultEncoder::writeLumaEOB  (&m_Bitstream); }
//    else                { xHuffDefaultEncoder::writeChromaEOB(&m_Bitstream); }
//  }
//}

void xEntropyEncoderDefault::xEncodeBlockL(const int16* ScanCoeff)
{
  //DC coefficient
  int32 DC                  = ScanCoeff[0];
  int32 DeltaDC             = DC - (int32)m_LastDC[(int32)eCmp::LM];
  m_LastDC[(int32)eCmp::LM] = (int16)DC;

  int32 SignMaskDC = DeltaDC >> 31;                       // make a mask of the sign bit
  int32 AbsDeltaDC = (DeltaDC ^ SignMaskDC) - SignMaskDC; // toggle the bits and add one if value is negative
  int32 NumBitsDC  = xNumSignificantBits((uint32)AbsDeltaDC);
  int32 RemainDC   = (DeltaDC + SignMaskDC) & ((1 << NumBitsDC) - 1);  // subtract one if value was negative and mask off any extra bits in code
  xHuffDefaultEncoder::writeLumaDC(&m_Bitstream, NumBitsDC, RemainDC);

  //AC coefficients
  int32 LastNonZero = findLastNonZero(ScanCoeff);
  int32 RunLength   = 0;
  for(int32 i=1; i <= LastNonZero; i++)
  {
    int32 AC = ScanCoeff[i];

    //nothing to encode
    if(AC == 0) { RunLength++; continue; }

    //if run length > 15, must emit special run-length-16 codes (0xF0)
    while (RunLength > 15) { xHuffDefaultEncoder::writeLumaZRL(&m_Bitstream); RunLength -= 16; }

    //Emit Huffman symbol for run length / number of bits
    int32 SignMaskAC = AC >> 31;                       // make a mask of the sign bit
    int32 AbsAC      = (AC ^ SignMaskAC) - SignMaskAC; // toggle the bits and add one if value is negative
    int32 NumBitsAC  = xNumSignificantBits((uint32)AbsAC);
    int32 RemainAC   = (AC + SignMaskAC) & ((1 << NumBitsAC) - 1);  // subtract one if value was negative and mask off any extra bits in code
    xHuffDefaultEncoder::writeLumaAC(&m_Bitstream, RunLength, NumBitsAC, RemainAC);

    //reset run length
    RunLength = 0;
  }
  //If the last coef(s) were zero - emit an end-of-block code
  if(LastNonZero < 63) { xHuffDefaultEncoder::writeLumaEOB(&m_Bitstream); }
}
void xEntropyEncoderDefault::xEncodeBlockC(const int16* ScanCoeff, eCmp Cmp)
{
  //DC coefficient
  int32 DC             = ScanCoeff[0];
  int32 DeltaDC        = DC - (int32)m_LastDC[(int32)Cmp];
  m_LastDC[(int32)Cmp] = (int16)DC;

  int32 SignMaskDC = DeltaDC >> 31;                       // make a mask of the sign bit
  int32 AbsDeltaDC = (DeltaDC ^ SignMaskDC) - SignMaskDC; // toggle the bits and add one if value is negative
  int32 NumBitsDC  = xNumSignificantBits((uint32)AbsDeltaDC);
  int32 RemainDC   = (DeltaDC + SignMaskDC) & ((1 << NumBitsDC) - 1);  // subtract one if value was negative and mask off any extra bits in code
  xHuffDefaultEncoder::writeChromaDC(&m_Bitstream, NumBitsDC, RemainDC);

  //AC coefficients
  int32 LastNonZero = findLastNonZero(ScanCoeff);
  int32 RunLength   = 0;
  for(int32 i=1; i <= LastNonZero; i++)
  {
    int32 AC = ScanCoeff[i];

    //nothing to encode
    if(AC == 0) { RunLength++; continue; }

    //if run length > 15, must emit special run-length-16 codes (0xF0)
    while (RunLength > 15) { xHuffDefaultEncoder::writeChromaZRL(&m_Bitstream); RunLength -= 16; }

    //Emit Huffman symbol for run length / number of bits
    int32 SignMaskAC = AC >> 31;                       // make a mask of the sign bit
    int32 AbsAC      = (AC ^ SignMaskAC) - SignMaskAC; // toggle the bits and add one if value is negative
    int32 NumBitsAC  = xNumSignificantBits((uint32)AbsAC);
    int32 RemainAC   = (AC + SignMaskAC) & ((1 << NumBitsAC) - 1);  // subtract one if value was negative and mask off any extra bits in code
    xHuffDefaultEncoder::writeChromaAC(&m_Bitstream, RunLength, NumBitsAC, RemainAC);

    //reset run length
    RunLength = 0;
  }
  //If the last coef(s) were zero, emit an end-of-block code
  if (LastNonZero < 63) { xHuffDefaultEncoder::writeChromaEOB(&m_Bitstream); }
}


//=====================================================================================================================================================================================

bool xEntropyEstimator::Init(std::vector<xJFIF::xHuffTable>& HuffTables)
{
  bool Result = true;
  for(xJFIF::xHuffTable& HuffTable : HuffTables)
  {
    xJFIF::xHuffTable::eHuffClass HuffTableClass = HuffTable.getClass();
    int32                         HuffTableId    = HuffTable.getIdx  ();
    switch(HuffTableClass)
    {
      case xJFIF::xHuffTable::eHuffClass::DC:
        if(m_HuffEstimatorDC[HuffTableId] == nullptr) { m_HuffEstimatorDC[HuffTableId] = new xHuffEstimatorDC; }
        Result &= m_HuffEstimatorDC[HuffTableId]->init(HuffTable);
        break;
      case xJFIF::xHuffTable::eHuffClass::AC:
        if(m_HuffEstimatorAC[HuffTableId] == nullptr) { m_HuffEstimatorAC[HuffTableId] = new xHuffEstimatorAC; }
        Result &= m_HuffEstimatorAC[HuffTableId]->init(HuffTable);
        break;
      default: Result = false; break;
    }
  }
  return Result;
}
void xEntropyEstimator::UnInit()
{
  for(int32 HuffTableId=0; HuffTableId < xJPEG_Constants::c_MaxHuffTabs; HuffTableId++)
  {
    if(m_HuffEstimatorDC[HuffTableId] != nullptr) { delete m_HuffEstimatorDC[HuffTableId]; m_HuffEstimatorDC[HuffTableId] = nullptr; }
    if(m_HuffEstimatorAC[HuffTableId] != nullptr) { delete m_HuffEstimatorAC[HuffTableId]; m_HuffEstimatorAC[HuffTableId] = nullptr; }
  }
}
int32 xEntropyEstimator::EstimateBlock(const int16* ScanCoeff, int32 LastDC, int32 HuffTableIdDC, int32 HuffTableIdAC) const
{
  //DC coefficient
  int32 DC          = ScanCoeff[0];
  int32 DeltaDC     = DC - LastDC;
  int32 SignMaskDC  = DeltaDC >> 31;                       // make a mask of the sign bit
  int32 AbsDeltaDC  = (DeltaDC ^ SignMaskDC) - SignMaskDC; // toggle the bits and add one if value is negative
  int32 NumBitsDC   = xNumSignificantBits((uint32)AbsDeltaDC);
  int32 CalcNumBits = m_HuffEstimatorDC[HuffTableIdDC]->calcDC(NumBitsDC);

  //AC coefficients
  xHuffEstimatorAC* HE = m_HuffEstimatorAC[HuffTableIdAC];
  int32 LastNonZero = findLastNonZero(ScanCoeff);
  int32 RunLength   = 0;
  for(int32 i=1; i <= LastNonZero; i++)
  {
    int32 AC = ScanCoeff[i];

    //nothing to encode
    if(AC == 0) { RunLength++; continue; }

    //if run length > 15, must emit special run-length-16 codes (0xF0)
    while (RunLength > 15) { CalcNumBits += HE->calcZRL(); RunLength -= 16; }

    //Emit Huffman symbol for run length / number of bits
    int32 SignMaskAC = AC >> 31;                       // make a mask of the sign bit
    int32 AbsAC      = (AC ^ SignMaskAC) - SignMaskAC; // toggle the bits and add one if value is negative
    int32 NumBitsAC  = xNumSignificantBits((uint32)AbsAC);
    int32 CodeAC     = (RunLength << 4) + NumBitsAC;
    CalcNumBits += HE->calcAC(CodeAC, NumBitsAC);

    //reset run length
    RunLength = 0;
  }
  //If the last coef(s) were zero, emit an end-of-block code
  if (LastNonZero < 63) { CalcNumBits += HE->calcEOB(); }

  return CalcNumBits;
}
int32 xEntropyEstimator::EstimateBlockDC(const int16* ScanCoeff, int32 LastDC, int32 HuffTableIdDC) const
{
  //DC coefficient
  int32 DC          = ScanCoeff[0];
  int32 DeltaDC     = DC - LastDC;
  int32 SignMaskDC  = DeltaDC >> 31;                       // make a mask of the sign bit
  int32 AbsDeltaDC  = (DeltaDC ^ SignMaskDC) - SignMaskDC; // toggle the bits and add one if value is negative
  int32 NumBitsDC   = xNumSignificantBits((uint32)AbsDeltaDC);
  int32 CalcNumBits = m_HuffEstimatorDC[HuffTableIdDC]->calcDC(NumBitsDC);
  return CalcNumBits;
}
int32 xEntropyEstimator::EstimateBlockAC(const int16* ScanCoeff, int32 HuffTableIdAC) const
{
  int32 CalcNumBits = 0;

  //AC coefficients
  xHuffEstimatorAC* HE = m_HuffEstimatorAC[HuffTableIdAC];
  int32 LastNonZero = findLastNonZero(ScanCoeff);
  int32 RunLength   = 0;
  for(int32 i=1; i <= LastNonZero; i++)
  {
    int32 AC = ScanCoeff[i];

    //nothing to encode
    if(AC == 0) { RunLength++; continue; }

    //if run length > 15, must emit special run-length-16 codes (0xF0)
    while (RunLength > 15) { CalcNumBits += HE->calcZRL(); RunLength -= 16; }

    //Emit Huffman symbol for run length / number of bits
    int32 SignMaskAC = AC >> 31;                       // make a mask of the sign bit
    int32 AbsAC      = (AC ^ SignMaskAC) - SignMaskAC; // toggle the bits and add one if value is negative
    int32 NumBitsAC  = xNumSignificantBits((uint32)AbsAC);
    int32 CodeAC     = (RunLength << 4) + NumBitsAC;
    CalcNumBits += HE->calcAC(CodeAC, NumBitsAC);

    //reset run length
    RunLength = 0;
  }
  //If the last coef(s) were zero, emit an end-of-block code
  if (LastNonZero < 63) { CalcNumBits += HE->calcEOB(); }

  return CalcNumBits;
}

//=====================================================================================================================================================================================
// xEntropyEstimatorDefault
//=====================================================================================================================================================================================

int32 xEntropyEstimatorDefault::EstimateBlock(const int16* ScanCoeff, int32 LastDC, eCmp Cmp)
{
  if(Cmp == eCmp::LM) { return xEstimateBlockL(ScanCoeff, ScanCoeff[0] - LastDC); }
  else                { return xEstimateBlockC(ScanCoeff, ScanCoeff[0] - LastDC); }
}
int32 xEntropyEstimatorDefault::xEstimateBlockL(const int16* ScanCoeff, int32 DeltaDC)
{
  //DC coefficient
  int32 SignMaskDC = DeltaDC >> 31;                       // make a mask of the sign bit
  int32 AbsDeltaDC = (DeltaDC ^ SignMaskDC) - SignMaskDC; // toggle the bits and add one if value is negative
  int32 NumBitsDC  = xNumSignificantBits((uint32)(uint32)AbsDeltaDC);
  int32 CalcNumBits = xHuffDefaultEstimator::calcLumaDC(NumBitsDC);

  //AC coefficients
  int32 LastNonZero = findLastNonZero(ScanCoeff);
  int32 RunLength   = 0;  
  for(int32 i=1; i <= LastNonZero; i++)
  {
    int32 AC = ScanCoeff[i];

    //nothing to encode
    if(AC == 0) { RunLength++; continue; }

    //if run length > 15, must emit special run-length-16 codes (0xF0)
    while (RunLength > 15) { CalcNumBits += xHuffDefaultEstimator::calcLumaZRL(); RunLength -= 16; }

    //Emit Huffman symbol for run length / number of bits
    int32 SignMaskAC = AC >> 31;                       // make a mask of the sign bit
    int32 AbsAC      = (AC ^ SignMaskAC) - SignMaskAC; // toggle the bits and add one if value is negative
    int32 NumBitsAC  = xNumSignificantBits((uint32)AbsAC);
    CalcNumBits += xHuffDefaultEstimator::calcLumaAC(RunLength, NumBitsAC);

    //reset run length
    RunLength = 0;
  }
  //If the last coef(s) were zero, emit an end-of-block code
  if (LastNonZero < 63) { CalcNumBits += xHuffDefaultEstimator::calcLumaEOB(); }

  return CalcNumBits;
}
int32 xEntropyEstimatorDefault::xEstimateBlockC(const int16* ScanCoeff, int32 DeltaDC)
{
  //DC coefficient
  int32 SignMaskDC = DeltaDC >> 31;                       // make a mask of the sign bit
  int32 AbsDeltaDC = (DeltaDC ^ SignMaskDC) - SignMaskDC; // toggle the bits and add one if value is negative
  int32 NumBitsDC  = xNumSignificantBits((uint32)AbsDeltaDC);
  int32 CalcNumBits = xHuffDefaultEstimator::calcChromaDC(NumBitsDC);

  //AC coefficients
  int32 LastNonZero = findLastNonZero(ScanCoeff);
  int32 RunLength   = 0;
  for(int32 i=1; i <= LastNonZero; i++)
  {
    int32 AC = ScanCoeff[i];

    //nothing to encode
    if(AC == 0) { RunLength++; continue; }

    //if run length > 15, must emit special run-length-16 codes (0xF0)
    while (RunLength > 15) { CalcNumBits += xHuffDefaultEstimator::calcChromaZRL(); RunLength -= 16; }

    //Emit Huffman symbol for run length / number of bits
    int32 SignMaskAC = AC >> 31;                       // make a mask of the sign bit
    int32 AbsAC      = (AC ^ SignMaskAC) - SignMaskAC; // toggle the bits and add one if value is negative
    int32 NumBitsAC  = xNumSignificantBits((uint32)AbsAC);
    CalcNumBits += xHuffDefaultEstimator::calcChromaAC(RunLength, NumBitsAC);

    //reset run length
    RunLength = 0;
  }
  //If the last coef(s) were zero, emit an end-of-block code
  if (LastNonZero < 63) { CalcNumBits += xHuffDefaultEstimator::calcChromaEOB(); }

  return CalcNumBits;
}

//=====================================================================================================================================================================================
// xEntropyCounter
//=====================================================================================================================================================================================
bool xEntropyCounter::Init(std::vector<xJFIF::xHuffTable>& HuffTables)
{
  bool Result = true;
  for(const xJFIF::xHuffTable& HuffTable : HuffTables)
  {
    xJFIF::xHuffTable::eHuffClass HuffTableClass = HuffTable.getClass();
    int32                         HuffTableId    = HuffTable.getIdx  ();
    switch(HuffTableClass)
    {
      case xJFIF::xHuffTable::eHuffClass::DC:
        if(m_HuffCounterDC[HuffTableId] == nullptr) { m_HuffCounterDC[HuffTableId] = new xHuffCounterDC; }
        Result &= m_HuffCounterDC[HuffTableId]->init();
        break;
      case xJFIF::xHuffTable::eHuffClass::AC:
        if(m_HuffCounterAC[HuffTableId] == nullptr) { m_HuffCounterAC[HuffTableId] = new xHuffCounterAC; }
        Result &= m_HuffCounterAC[HuffTableId]->init();
        break;
      default: Result = false; break;
    }
  }
  return Result;
}
void xEntropyCounter::UnInit()
{
  for(int32 HuffTableId=0; HuffTableId < xJPEG_Constants::c_MaxHuffTabs; HuffTableId++)
  {
    if(m_HuffCounterDC[HuffTableId] != nullptr) { delete m_HuffCounterDC[HuffTableId]; m_HuffCounterDC[HuffTableId] = nullptr; }
    if(m_HuffCounterAC[HuffTableId] != nullptr) { delete m_HuffCounterAC[HuffTableId]; m_HuffCounterAC[HuffTableId] = nullptr; }
  }
}
void xEntropyCounter::ZeroCounters()
{
  for(int32 i = 0; i < xJPEG_Constants::c_MaxHuffTabs; i++)
  {
    if(m_HuffCounterDC[i] != nullptr) { m_HuffCounterDC[i]->init(); }
    if(m_HuffCounterAC[i] != nullptr) { m_HuffCounterAC[i]->init(); }
  }
}
void xEntropyCounter::CountBlock(const int16* ScanCoeff, int32 LastDC, int32 HuffTableIdDC, int32 HuffTableIdAC)
{
  //DC coefficient
  int32 DC         = ScanCoeff[0];
  int32 DeltaDC    = DC - LastDC;
  int32 SignMaskDC = DeltaDC >> 31;                       // make a mask of the sign bit
  int32 AbsDeltaDC = (DeltaDC ^ SignMaskDC) - SignMaskDC; // toggle the bits and add one if value is negative
  int32 NumBitsDC  = xNumSignificantBits((uint32)AbsDeltaDC);
  m_HuffCounterDC[HuffTableIdDC]->countDC(NumBitsDC);

  //AC coefficients
  xHuffCounterAC* HE = m_HuffCounterAC[HuffTableIdAC];
  int32 LastNonZero = findLastNonZero(ScanCoeff);
  int32 RunLength   = 0;
  for(int32 i=1; i <= LastNonZero; i++)
  {
    int32 AC = ScanCoeff[i];

    //nothing to encode
    if(AC == 0) { RunLength++; continue; }

    //if run length > 15, must emit special run-length-16 codes (0xF0)
    while (RunLength > 15) { HE->countZRL(); RunLength -= 16; }

    //Emit Huffman symbol for run length / number of bits
    int32 SignMaskAC = AC >> 31;                       // make a mask of the sign bit
    int32 AbsAC      = (AC ^ SignMaskAC) - SignMaskAC; // toggle the bits and add one if value is negative
    int32 NumBitsAC  = xNumSignificantBits((uint32)AbsAC);
    int32 CodeAC     = (RunLength << 4) + NumBitsAC;
    HE->countAC(CodeAC);

    //reset run length
    RunLength = 0;
  }
  //If the last coef(s) were zero, emit an end-of-block code
  if (LastNonZero < 63) { HE->countEOB(); }
}
void xEntropyCounter::AddCounters(const xEntropyCounter& Other)
{
  for(int32 i = 0; i < xJPEG_Constants::c_MaxHuffTabs; i++)
  {
    if(m_HuffCounterDC[i] != nullptr) { m_HuffCounterDC[i]->acc(Other.m_HuffCounterDC[i]); }
    if(m_HuffCounterAC[i] != nullptr) { m_HuffCounterAC[i]->acc(Other.m_HuffCounterAC[i]); }
  }
}

//=====================================================================================================================================================================================

} //end of namespace PMBB::JPEG