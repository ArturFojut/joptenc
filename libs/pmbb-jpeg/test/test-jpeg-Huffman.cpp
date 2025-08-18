/*
    SPDX-FileCopyrightText: 2019-2023 Jakub Stankowski <jakub.stankowski@put.poznan.pl>
    SPDX-License-Identifier: BSD-3-Clause
*/

#if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_WARNINGS)
#define _CRT_SECURE_NO_WARNINGS
#endif

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include <functional>
#include <utility>
#include <array>
#include "xTestUtils.h"
#include "xTimeUtils.h"
#include "xMemory.h"
#include "xCommonDefJPEG.h"
#include "xJPEG_Entropy.h"

using namespace PMBB_NAMESPACE;
using namespace PMBB_NAMESPACE::JPEG;

//===============================================================================================================================================================================================================

class xHuffEncoderDC_Test : public xHuffEncoderDC
{
public:
  const uint32* getHuffCode() const { return m_HuffCode; }
  const uint8*  getHuffLen () const { return m_HuffLen ; }
};

class xHuffEncoderAC_Test : public xHuffEncoderAC
{
public:
  const uint32* getHuffCode() const { return m_HuffCode; }
  const uint8*  getHuffLen () const { return m_HuffLen ; }
};

//===============================================================================================================================================================================================================

void testHuffmanInitialization(xJFIF::xHuffTable::eHuffClass HuffClass, eCmp Component)
{
  xJFIF::xHuffTable HT;
  HT.InitDefault(0, HuffClass, Component);

  const uint8*  HuffLenDefault  = nullptr;
  const uint16* HuffCodeDefault = nullptr;
  if(HuffClass == xJFIF::xHuffTable::eHuffClass::DC && Component == eCmp::LM) { HuffLenDefault = xJPEG_Constants::c_HuffLenDefaultLumaDC  ; HuffCodeDefault = xJPEG_Constants::c_HuffCodeDefaultLumaDC  ; }
  if(HuffClass == xJFIF::xHuffTable::eHuffClass::AC && Component == eCmp::LM) { HuffLenDefault = xJPEG_Constants::c_HuffLenDefaultLumaAC  ; HuffCodeDefault = xJPEG_Constants::c_HuffCodeDefaultLumaAC  ; }
  if(HuffClass == xJFIF::xHuffTable::eHuffClass::DC && Component != eCmp::LM) { HuffLenDefault = xJPEG_Constants::c_HuffLenDefaultChromaDC; HuffCodeDefault = xJPEG_Constants::c_HuffCodeDefaultChromaDC; }
  if(HuffClass == xJFIF::xHuffTable::eHuffClass::AC && Component != eCmp::LM) { HuffLenDefault = xJPEG_Constants::c_HuffLenDefaultChromaAC; HuffCodeDefault = xJPEG_Constants::c_HuffCodeDefaultChromaAC; }
  REQUIRE(HuffLenDefault  != nullptr);
  REQUIRE(HuffCodeDefault != nullptr);

  if(HuffClass == xJFIF::xHuffTable::eHuffClass::DC)
  {
    xHuffEncoderDC_Test HE;
    HE.init(HT);

    for(int32 i = 0; i < xJPEG_Constants::c_MaxNumCodeSymbolsDC; i++)
    {
      uint8 TstHuffLen = HE.getHuffLen()[i];
      uint8 RefHuffLen = HuffLenDefault[i];
      CHECK(TstHuffLen == RefHuffLen);
      uint32 TstHuffCode = HE.getHuffCode()[i];
      uint32 RefHuffCode = HuffCodeDefault[i];
      CHECK(TstHuffCode == RefHuffCode);
    }
  }

  if(HuffClass == xJFIF::xHuffTable::eHuffClass::AC)
  {
    xHuffEncoderAC_Test HE;
    HE.init(HT);

    for(int32 i = 0; i < xJPEG_Constants::c_MaxNumCodeSymbolsAC; i++)
    {
      uint8 TstHuffLen = HE.getHuffLen()[i];
      uint8 RefHuffLen = HuffLenDefault[i];
      CHECK(TstHuffLen == RefHuffLen);
      uint32 TstHuffCode = HE.getHuffCode()[i];
      uint32 RefHuffCode = HuffCodeDefault[i];
      CHECK(TstHuffCode == RefHuffCode);
    }
  }
}

//===============================================================================================================================================================================================================

void testCustomHuffmanTableGeneration(const uint8* HuffLenghts, const uint8* CodeLengths, xJFIF::xHuffTable::eHuffClass HuffClass)
{
  const int32 MaxNumCodeSymbols = xJFIF::xHuffTable::getMaxNumCodeSymbols(HuffClass);

  //generate simulated symbol conts based on code length
  std::vector<uint32> SymbolCount(MaxNumCodeSymbols);
  for(int32 i = 0; i < MaxNumCodeSymbols; i++)
  {
    int32  CodeLen  = HuffLenghts[i];
    uint32 EstCount = CodeLen != 0 ? 1<<(18 - CodeLen) : 0; // same as 2^18 * 2^(-CodeLen)
    SymbolCount[i]  = EstCount;
  }

  //design custom code table 
  std::vector<uint8> LengthTable(MaxNumCodeSymbols + 1);
  xHuffmanTabBuilder::buildLengthTable(LengthTable.data(), SymbolCount.data(), MaxNumCodeSymbols);
  xJFIF::xHuffTable HT;
  HT.InitCustom(0, HuffClass, LengthTable.data());

  //check if DHT code length entries are the same (checking DTH code sumbol is useless - they can differ)
  for(int32 i = 0; i < xJPEG_Constants::c_NumCodeLenghts; i++)
  {
    CHECK(HT.getCodeLengths()[i] == CodeLengths[i]);
  }

  //check if decoded Huffman code lenghts are same (checking decoded code sumbols is useless - they can differ)
  if(HuffClass == xJFIF::xHuffTable::eHuffClass::DC)
  {
    xHuffEncoderDC_Test HE;
    HE.init(HT);

    for(int32 i = 0; i < MaxNumCodeSymbols; i++)
    {
      uint8 TstHuffLen = HE.getHuffLen()[i];
      uint8 RefHuffLen = HuffLenghts[i];
      CHECK(TstHuffLen == RefHuffLen);
    }
  }  
  if(HuffClass == xJFIF::xHuffTable::eHuffClass::AC)
  {
    xHuffEncoderAC_Test HE;
    HE.init(HT);

    for(int32 i = 0; i < MaxNumCodeSymbols; i++)
    {
      uint8 TstHuffLen = HE.getHuffLen()[i];
      uint8 RefHuffLen = HuffLenghts[i];
      CHECK(TstHuffLen == RefHuffLen);
    }
  }
}

//===============================================================================================================================================================================================================

TEST_CASE("testHuffmanInitialization")
{
  testHuffmanInitialization(xJFIF::xHuffTable::eHuffClass::DC, eCmp::LM);
  testHuffmanInitialization(xJFIF::xHuffTable::eHuffClass::AC, eCmp::LM);
  testHuffmanInitialization(xJFIF::xHuffTable::eHuffClass::DC, eCmp::CB);
  testHuffmanInitialization(xJFIF::xHuffTable::eHuffClass::AC, eCmp::CB);
}

TEST_CASE("testCustomHuffmanTableGeneration")
{
  testCustomHuffmanTableGeneration(xJPEG_Constants::c_HuffLenDefaultLumaDC  , xJPEG_Constants::m_CodeLengthLumaDC  , xJFIF::xHuffTable::eHuffClass::DC);
  testCustomHuffmanTableGeneration(xJPEG_Constants::c_HuffLenDefaultLumaAC  , xJPEG_Constants::m_CodeLengthLumaAC  , xJFIF::xHuffTable::eHuffClass::AC);
  testCustomHuffmanTableGeneration(xJPEG_Constants::c_HuffLenDefaultChromaDC, xJPEG_Constants::m_CodeLengthChromaDC, xJFIF::xHuffTable::eHuffClass::DC);
  testCustomHuffmanTableGeneration(xJPEG_Constants::c_HuffLenDefaultChromaAC, xJPEG_Constants::m_CodeLengthChromaAC, xJFIF::xHuffTable::eHuffClass::AC);
}

//===============================================================================================================================================================================================================
