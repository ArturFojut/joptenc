/*
    SPDX-FileCopyrightText: 2020-2024 Jakub Stankowski <jakub.stankowski@put.poznan.pl>
    SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once
#include "xCommonDefJPEG.h"
#include "xJPEG_Constants.h"
#include "xTimeUtils.h"
#include "xVec.h"
#include "xJFIF.h"
#include "xThreadPool.h"

namespace PMBB_NAMESPACE::JPEG {

//=====================================================================================================================================================================================

class xCodecCommon
{
protected:
  static constexpr int32 c_NC   = xJPEG_Constants::c_MaxComponents;
  static constexpr int32 c_L2BS = xJPEG_Constants::c_Log2BlockSize;
  static constexpr int32 c_BS   = xJPEG_Constants::c_BlockSize;
  static constexpr int32 c_L2BA = xJPEG_Constants::c_Log2BlockArea;
  static constexpr int32 c_BA   = xJPEG_Constants::c_BlockArea;

protected:
  int32V2 m_PictureSize     = { NOT_VALID, NOT_VALID };
  int32   m_PictureWidth    = NOT_VALID;
  int32   m_PictureHeight   = NOT_VALID;
  eCrF    m_ChromaFormat    = eCrF::INVALID;
  int32   m_NumOfComponents = NOT_VALID;
  int32   m_ProcessChroma   = false;

  std::array<int32, c_NC> m_SampFactorHor     = {0};
  std::array<int32, c_NC> m_SampFactorVer     = {0};
  std::array<int32, c_NC> m_ShiftHor          = {0};
  std::array<int32, c_NC> m_ShiftVer          = {0};
  std::array<int32, c_NC> m_Log2MCUsWidth     = {0};
  std::array<int32, c_NC> m_Log2MCUsHeight    = {0};
  std::array<int32, c_NC> m_CmpWidth          = {0}; //real size of image m_ExternalWidth
  std::array<int32, c_NC> m_CmpHeight         = {0};
  std::array<int32, c_NC> m_ScanlineHeight    = {0};
  std::array<int32, c_NC> m_MCUsMulWidth      = {0}; //size of image + padding (full MCU) m_BufferWidth
  std::array<int32, c_NC> m_MCUsMulHeight     = {0};
  std::array<int32, c_NC> m_MCUsMulArea       = {0};
  std::array<int32, c_NC> m_NumBlocksInWidth  = {0};
  std::array<int32, c_NC> m_NumBlocksInHeight = {0};
  std::array<int32, c_NC> m_NumBlocksInArea   = {0};
  std::array<int32, c_NC> m_NumBlocksInMCU    = {0};  

  int32   m_NumMCUsInWidth  = NOT_VALID;
  int32   m_NumMCUsInHeight = NOT_VALID;
  int32   m_NumMCUsInArea   = NOT_VALID;
  int32   m_NumMCUsInSlice  = NOT_VALID;
  int32   m_NumOfSlices     = 1;

  //operation
  int32   m_VerboseLevel    = NOT_VALID;

  //profiling
protected:
  bool   m_GatherTimeStats   = false;
  uint64 m_Ticks___Picture   = 0;
  int64  m_TotalPictureIters = 0;
  int64  m_TotalSliceIters   = 0;

public:
  void  setVerboseLevel(int32 VerboseLevel)       { m_VerboseLevel = VerboseLevel; }
  int32 getVerboseLevel(                  ) const { return m_VerboseLevel;         }

  void  setGatherTimeStats(bool GatherTimeStats)       { m_GatherTimeStats = GatherTimeStats; }
  bool  getGatherTimeStats(                    ) const { return m_GatherTimeStats;            }

  static int32 calcNumMCUsInWidth(int32V2 PictureSize, eCrF ChromaFormat);

protected:
  void initCodecCommon(int32V2 PictureSize, eCrF ChromaFormat);

  static inline void loadEntireBlock(uint16* restrict Dst, const uint16* Src, int32 SrcStride)
  {
    for(int32 y = 0; y < c_BS; y++)
    {
      ::memcpy(Dst, Src, c_BS * sizeof(uint16));
      Src += SrcStride; Dst += c_BS;
    }
  }

  static inline void storeEntireBlock(uint16* restrict Dst, const uint16* Src, int32 DstStride)
  {
    for(int32 y = 0; y < c_BS; y++)
    {
      ::memcpy(Dst, Src, c_BS * sizeof(uint16));
      Src += c_BS; Dst += DstStride;
    }
  }

  static inline void zeroEntireBlock(uint16* restrict Dst)
  {
    memset(Dst, 0, c_BA * sizeof(int16));
  }

  static inline void loadExtendBlock(uint16* restrict Dst, const uint16* Src, int32 SrcStride, int32 AvailableWidth, int32 AvailableHeight)
  {
    int32 y = 0;
    for(; y < AvailableHeight; y++)
    {
      int32 x = 0;
      for(; x < AvailableWidth; x++) { Dst[x] = Src[x]                 ; }
      for(; x < c_BS          ; x++) { Dst[x] = Dst[AvailableWidth - 1]; }
      Dst += c_BS;
      Src += SrcStride;
    }
    Src = Dst - c_BS;
    for(; y < c_BS; y++)
    {
      memcpy(Dst, Src, c_BS * sizeof(uint16));
      Dst += c_BS;
    }
  }

  static inline void storePartialBlock(uint16* restrict Dst, const uint16* Src, int32 DstStride, int32 AvailableWidth, int32 AvailableHeight)
  {    
    for(int32 y = 0; y < AvailableHeight; y++)
    {
      ::memcpy(Dst, Src, AvailableWidth * sizeof(uint16));
      Src += c_BS; Dst += DstStride;
    }
  }
};

//=====================================================================================================================================================================================

class xCodecImplCommon : public xCodecCommon
{
public:
  using tQTV = std::vector<xJFIF::xQuantTable>;
  using tDHV = std::vector<xJFIF::xHuffTable >;
protected:
  //Markers
  xJFIF::xAPP0 m_APP0;
  tQTV         m_QTs; //quantization tables
  int32        m_RestartInterval = 0;
  xJFIF::xSOF0 m_SOF0;
  tDHV         m_HTs; //huffman tables 
  xJFIF::xSOS  m_SOS;

  //Threading
  tThPI m_ThPI; //thread pool interface
};

//=====================================================================================================================================================================================

} //end of namespace PMBB::JPEG