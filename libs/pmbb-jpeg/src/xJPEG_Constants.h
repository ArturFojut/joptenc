/*
    SPDX-FileCopyrightText: 2020-2024 Jakub Stankowski <jakub.stankowski@put.poznan.pl>
    SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once
#include "xCommonDefJPEG.h"

namespace PMBB_NAMESPACE::JPEG {

//===============================================================================================================================================================================================================

class xJPEG_Constants
{
public:
  static constexpr int32 c_MaxQuantTabs  = 4;
  static constexpr int32 c_MaxHuffTabs   = 4;
  static constexpr int32 c_MaxComponents = 4;

  static constexpr int32 c_Log2BlockSize = 3;
  static constexpr int32 c_BlockSize     = 1 << c_Log2BlockSize;
  static constexpr int32 c_Log2BlockArea = 3 << 1;
  static constexpr int32 c_BlockArea     = c_BlockSize * c_BlockSize;

  static constexpr int32 c_NumCodeLenghts      = 16;
  static constexpr int32 c_MaxNumCodeSymbolsDC = 16;
  static constexpr int32 c_MaxNumCodeSymbolsAC = 256;

public:
  //scan tables
  static const uint8 m_ScanZigZag   [c_BlockArea];
  static const uint8 m_InvScanZigZag[c_BlockArea];

  //quantization tables in raster order
  static const uint8 m_QuantTabDefLumaR  [c_BlockArea];
  static const uint8 m_QuantTabDefChromaR[c_BlockArea];

  //quantization tables in zigzag order
  static const uint8 m_QuantTabDefLumaZ  [c_BlockArea];
  static const uint8 m_QuantTabDefChromaZ[c_BlockArea];

  //default huffman tables
  static const uint8 m_CodeLengthLumaDC  [ 16];
  static const uint8 m_CodeSymbolLumaDC  [ 12];
  static const uint8 m_CodeLengthLumaAC  [ 16];
  static const uint8 m_CodeSymbolLumaAC  [162];
  static const uint8 m_CodeLengthChromaDC[ 16];
  static const uint8 m_CodeSymbolChromaDC[ 12];
  static const uint8 m_CodeLengthChromaAC[ 16];
  static const uint8 m_CodeSymbolChromaAC[162];

  //default huffman codes 
  //DC code = NumBitsDC
  //AC code = (RunLength << 4) + NumBitsAC
  static const uint8  c_HuffLenDefaultLumaDC   [ 16];
  static const uint16 c_HuffCodeDefaultLumaDC  [ 16];
  static const uint8  c_HuffLenDefaultLumaAC   [256];
  static const uint16 c_HuffCodeDefaultLumaAC  [256];
  static const uint8  c_HuffLenDefaultChromaDC [ 16];
  static const uint16 c_HuffCodeDefaultChromaDC[ 16];
  static const uint8  c_HuffLenDefaultChromaAC [256];
  static const uint16 c_HuffCodeDefaultChromaAC[256];

public:
  static void  GenerateQuantTable    (uint8* QuantTable, eCmp Cmp, int32 Q, eQTLa QuantTabLayout);
  static void  GenerateQuantTableDef (uint8* QuantTable, eCmp Cmp, int32 Q);
  static void  GenerateQuantTableFlat(uint8* QuantTable, eCmp Cmp, int32 Q);
  static void  GenerateQuantTableSemi(uint8* QuantTable, eCmp Cmp, int32 Q);

  static int32 EstimateQ(uint8* QuantTableL, uint8* QuantTableC);
};

//===============================================================================================================================================================================================================

} //end of namespace PMBB_NAMESPACE::JPEG
