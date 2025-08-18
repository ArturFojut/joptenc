/*
    SPDX-FileCopyrightText: 2020-2024 Jakub Stankowski <jakub.stankowski@put.poznan.pl>
    SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once
#include "xCommonDefJPEG.h"
#include "xJFIF.h"
#include "xBitstream.h"
#include <map>
#include <queue>

#define X_PMBB_JPEG_MULTI_LEVEL_LOOKAHEAD 0

namespace PMBB_NAMESPACE::JPEG {

 //=====================================================================================================================================================================================

class xHuffCommon
{
public:
  static bool xInitHuffTables   (uint8* HuffLen, uint32* HuffCode, const xJFIF::xHuffTable& HuffTable);
  static void xAvoidZeroLenCodes(uint8* HuffLen, int32 TableSize);

  static int32 xFillTmpLengths(uint8 * Lenghts, const xJFIF::xHuffTable::tCodeL& TabCodeLengths);
  static int32 xFillTmpCodes  (uint32* Codes  , const uint8* Lenghts, int32 NumLengths);
};

//=====================================================================================================================================================================================

class xHuffDecoder : public xHuffCommon
{
protected:
#if X_PMBB_JPEG_MULTI_LEVEL_LOOKAHEAD
  static const int32 c_LookAhead1st =  8; //fixed size
  static const int32 c_LookAhead2nd = 16; //fixed size
#else
  static const int32 c_LookAhead    = 10; //can be up to 16
#endif

  uint8  m_CodeSymbols[256];
  int32  m_MaxCode    [18 ]; //largest code of length k (-1 if none)
  int32  m_ValOffset  [18 ]; //huffval[] offset for codes of length k
#if X_PMBB_JPEG_MULTI_LEVEL_LOOKAHEAD
  uint16 m_Lookup1st  [1<<c_LookAhead1st];
  uint32 m_Lookup2nd  [1<<c_LookAhead2nd];
#else
  uint32  m_Lookup    [1<<c_LookAhead   ];
#endif

public:
  bool init(const xJFIF::xHuffTable& HuffTable)
  {     
    //return xCreateDerrivedDecoder(m_CodeSymbols, m_MaxCode, m_ValOffset, m_Lookup, HuffTable);
    return xInitTables(HuffTable);
  }
  int32 readPrefix(xBitstreamReader* Bitstream);
  int32 readSufix(xBitstreamReader* Bitstream, int32 NumBits)
  {
    int32 R = Bitstream->readBits(NumBits);
    int32 Value = R + (((R - (1 << (NumBits - 1))) >> 31) & ((((uint32)-1) << NumBits) + 1));
    return Value;
  }
  int32 readDC(xBitstreamReader* Bitstream)
  {
    int32 DC = readPrefix(Bitstream);
    if(DC) { DC = readSufix(Bitstream, DC); }
    return DC;
  }

protected:
  bool xInitTables(const xJFIF::xHuffTable& HuffTable);

};

//=====================================================================================================================================================================================

class xHuffEncoderDC : public xHuffCommon
{
protected:
  uint32 m_HuffCode[xJPEG_Constants::c_MaxNumCodeSymbolsDC];
  uint8  m_HuffLen [xJPEG_Constants::c_MaxNumCodeSymbolsDC];

public:
  bool init    (const xJFIF::xHuffTable& HuffTable) { if(HuffTable.getClass() != xJFIF::xHuffTable::eHuffClass::DC) { return false; } return xInitHuffTables(m_HuffLen, m_HuffCode, HuffTable); }
  void writeDC (xBitstreamWriter* Bitstream, int32 NumBits, uint32 Remainder) { Bitstream->writeBits(m_HuffCode[NumBits], m_HuffLen[NumBits]); if(NumBits) { Bitstream->writeBits(Remainder, NumBits); } }
};

class xHuffEncoderAC : public xHuffCommon
{
protected:
  uint32 m_HuffCode[xJPEG_Constants::c_MaxNumCodeSymbolsAC];
  uint8  m_HuffLen [xJPEG_Constants::c_MaxNumCodeSymbolsAC];

public:
  bool init    (xJFIF::xHuffTable& HuffTable) { if(HuffTable.getClass() != xJFIF::xHuffTable::eHuffClass::AC) { return false; } return xInitHuffTables(m_HuffLen, m_HuffCode, HuffTable); }
  void writeAC (xBitstreamWriter* Bitstream, int32 Code, int32 NumBits, uint32 Remainder) { Bitstream->writeBits(m_HuffCode[Code], m_HuffLen[Code]); Bitstream->writeBits(Remainder, NumBits); }
  void writeZRL(xBitstreamWriter* Bitstream) { Bitstream->writeBits(m_HuffCode[0xF0], m_HuffLen[0xF0]); }
  void writeEOB(xBitstreamWriter* Bitstream) { Bitstream->writeBits(m_HuffCode[0x00], m_HuffLen[0x00]); }
};

//=====================================================================================================================================================================================

class xHuffEstimatorDC: public xHuffCommon
{
protected:
  uint8 m_HuffLen [xJPEG_Constants::c_MaxNumCodeSymbolsDC];

public:
  bool  init  (const xJFIF::xHuffTable& HuffTable);
  int32 calcDC(int32 NumBits) const { return m_HuffLen[NumBits] + NumBits; }
};

class xHuffEstimatorAC : public xHuffCommon
{
protected:
  uint8  m_HuffLen [xJPEG_Constants::c_MaxNumCodeSymbolsAC];

public:
  bool  init   (const xJFIF::xHuffTable& HuffTable);
  int32 calcAC (int32 Code, int32 NumBits) const { return m_HuffLen[Code] + NumBits; }
  int32 calcZRL() const { return m_HuffLen[0xF0]; }
  int32 calcEOB() const { return m_HuffLen[0x00]; }
};

//=====================================================================================================================================================================================

class xHuffCounterDC
{
protected:
  static constexpr int32 c_NCS = xJPEG_Constants::c_MaxNumCodeSymbolsDC;
  uint32 m_SymbolCount[c_NCS];

public:
  bool init   (          ) { memset(m_SymbolCount, 0, c_NCS * sizeof(uint32)); return true; }
  void countDC(int32 Code) { m_SymbolCount[Code]++; }
  void acc    (const xHuffCounterDC* Other) { for(int32 i = 0; i < c_NCS; i++) { m_SymbolCount[i] += Other->m_SymbolCount[i]; } }
  const uint32* getSymbolCount() const { return m_SymbolCount; }
};

class xHuffCounterAC
{
protected:
  static constexpr int32 c_NCS = xJPEG_Constants::c_MaxNumCodeSymbolsAC;
  uint32 m_SymbolCount[c_NCS];

public:
  bool init    (          ) { memset(m_SymbolCount, 0, c_NCS * sizeof(uint32)); return true; }
  void countAC (int32 Code) { m_SymbolCount[Code]++; }
  void countZRL(          ) { m_SymbolCount[0xF0]++; }
  void countEOB(          ) { m_SymbolCount[0x00]++; }
  void acc     (const xHuffCounterAC* Other) { for(int32 i = 0; i < c_NCS; i++) { m_SymbolCount[i] += Other->m_SymbolCount[i]; } }
  const uint32* getSymbolCount() const { return m_SymbolCount; }
};

//=====================================================================================================================================================================================

class xHuffmanTabBuilder
{
protected:
  class xHuffTree
  {
  public:
    int32 m_Symbol = NOT_VALID;
    int64 m_Count  = NOT_VALID;

    xHuffTree* m_Left  = nullptr;
    xHuffTree* m_Right = nullptr;

    xHuffTree(int16 Symbol, int32 Count) { m_Symbol = Symbol; m_Count = Count; m_Left = nullptr; m_Right = nullptr; }
    xHuffTree(xHuffTree* L, xHuffTree* R) { m_Symbol = NOT_VALID; m_Count = L->m_Count + R->m_Count; m_Left = L; m_Right = R; }

    ~xHuffTree()
    {
      if(m_Left  != nullptr) { delete m_Left ; }
      if(m_Right != nullptr) { delete m_Right; }
    }
  };

  struct Comparator { bool operator()(const xHuffTree* L, const xHuffTree* R) const 
  {
    if(L->m_Count == R->m_Count)
    {
      return L->m_Symbol < R->m_Symbol;
    }
    return L->m_Count > R->m_Count;
  } };

public:
  static void buildLengthTable(uint8* LengthTable, const uint32* SymbolCount, int32 Size);

protected:
  static void  xCalcCodeLengths  (uint8* CodeLengths, xHuffTree* Node, int32 Length);
  static flt64 xCalcAvgCodeLength(const uint8* CodeLength, const uint32* SymbolCount, int32 Size);

public:
  static flt64 calcAvgCodeLength (const xJFIF::xHuffTable& HuffTable, const uint32* SymbolCount);
};

//=====================================================================================================================================================================================

} //end of namespace PMBB::JPEG