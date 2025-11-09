/*
    SPDX-FileCopyrightText: 2020-2024 Jakub Stankowski <jakub.stankowski@put.poznan.pl>
    SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once
#include "xCommonDefJPEG.h"
#include "xJPEG_CodecCommon.h"
#include "xJFIF.h"
#include "xPicYUV.h"
#include "xJPEG_Quant.h"
#include "xJPEG_Entropy.h"
#include <array>

namespace PMBB_NAMESPACE::JPEG {

//=====================================================================================================================================================================================

class xAdvancedEncoder : public xCodecImplCommon
{
public:
  using tDistBits = std::tuple<int64V4, int64V4>;

protected:
  int32 m_Quality;

  //encoder behaviour
  bool    m_EmitAPP0      = true;
  bool    m_EmitQuantTabs = true;
  bool    m_EmitHuffTabs  = true;
  //optimization
  bool    m_UseRDOQ           = false;
  bool    m_OptQuantLuma      = false;
  bool    m_OptQuantChroma    = false;  
  bool    m_ProcessZeroCoeffs = false;
  bool    m_OptHuffTables     = false;
  int32   m_NumOptPassesBlock = 0;
  int32   m_NumOptPassesPic   = 0;
  bool    m_UseDctSsd         = false;

  //lambdas
  flt64V4 m_Lambda            = { 1.0, 1.0, 1.0, 1.0 };

  //Tools
  xQuantizerSet     m_QuantMain;
  xQuantizerSet     m_QuantAuxD;
  xQuantizerSet     m_QuantAuxI;  

  xEntropyEstimator            m_EntropyEst;
  std::vector<xEntropyCounter> m_EntropyCnts;
  std::vector<xEntropyEncoder> m_EntropyEncs;  
 
  //Buffers
  xPicYUV* m_PicYCbCr444 = nullptr;
  xPicYUV* m_PicYCbCr4XX = nullptr;

  int16*  m_CmpCoeffsTransOrg[c_NC];
  int16*  m_CmpCoeffsTransRec[c_NC];
  int16*  m_CmpCoeffsScan    [c_NC];
  int16*  m_CmpCoeffsScanAux [c_NC];
  int16*  m_CmpCoeffsScanOpt [c_NC];
  xPicYUV m_PicRec;

  std::vector<xByteBuffer> m_EncBuffers  ;
  std::vector<xByteBuffer> m_SliceBuffers;
  std::vector<int32      > m_NumAlignmentBits;
  xByteBuffer              m_CombineBuffer;

  //Profiling
  uint64 m_Ticks_Transform = 0;
  uint64 m_Ticks_QuantScan = 0;
  uint64 m_Ticks____Lambda = 0;
  uint64 m_Ticks__OptQuant = 0;
  uint64 m_Ticks___OptHuff = 0;
  uint64 m_TicksEntropyEnc = 0;
  uint64 m_Ticks____Tables = 0;
  uint64 m_Ticks__WriteOut = 0;

public:
  void create (int32V2 PictureSize, eCrF ChromaFormat, xThreadPool* ThreadPool = nullptr);
  void destroy();

  void initBaseMarkers();
  void initQuant    (int32 Quality, eQTLa QuantTabLayout);
  void initEntropy  (int32 RestartInterval);
  void setMarkerEmit(bool EmitAPP0, bool EmitQuantTabs, bool EmitHuffmanTabs);
  void setQuantOpt  (bool OptimizeLuma, bool OptimizeChroma, bool ProcessZeroCoeffs);
  void setHuffOpt   (bool OptimizeHuffmanTables);
  void setOptPass   (int32 NumBlockOptPasses, int32 NumPicOptPasses);
  void setDctSsd    (bool UseDctSsd);
  
  void encode(const xPicYUV* InputPicture, xByteBuffer* OutputBuffer);

  //tDistBits calcDistBits(const xPicYUV* Picture); unused

  std::string formatAndResetStats(const std::string Prefix, flt64 TicksPerMiliSec);
  std::string formatStatsFile(flt64 TicksPerMiliSec);

protected:
  void    xEncodePicture  (xByteBuffer* Buffer, const xPicYUV* Picture);
  int64V4 xCalcPicSSDs    (const xPicYUV* Tst, const xPicYUV* Ref);

  void xFwdTransformPic(int16* CoeffsTransV[], const xPicYUV* Picture);
  void xFwdTransformSlc(int16* CoeffsTransV[], const uint16* CmpPtrV[], const int32 CmpStrideV[], int32 MCU_IdxBeg, int32 MCU_IdxEnd);
  void xFwdTransformMCU(int16* CoeffsTransV[], const uint16* CmpPtrV[], const int32 CmpStrideV[], int32 MCU_Idx);
  void xInvTransformPic(xPicYUV* Picture, const int16* CoeffsTransV[]);
  void xInvTransformSlc(uint16* CmpPtrV[], const int32 CmpStrideV[], const int16* CoeffsTransV[], int32 MCU_IdxBeg, int32 MCU_IdxEnd);
  void xInvTransformMCU(uint16* CmpPtrV[], const int32 CmpStrideV[], const int16* CoeffsTransV[], int32 MCU_Idx);

  void        xFwdQuantScanPic(int16* CoeffScanV [], const int16* CoeffTransV[], const xQuantizerSet& Quant);
  static void xFwdQuantScanRng(int16* CoeffScan    , const int16* CoeffTrans   , const xQuantizer& Quant, int32 BlockIdxBeg, int32 BlockIdxEnd);
  void        xInvScanQuantPic(int16* CoeffTransV[], const int16* CoeffScanV[] , const xQuantizerSet& Quant);
  static void xInvScanQuantRng(int16* CoeffTrans   , const int16* CoeffScan    , const xQuantizer& Quant, int32 BlockIdxBeg, int32 BlockIdxEnd);

  int64V4 xHuffEstPic(const int16* CoeffsScanV[]);
  int64V4 xHuffEstSlc(const int16* CoeffsScanV[], int32 MCU_IdxFirst, int32 MCU_IdxLast);
  int64V4 xHuffEstMCU(const int16* CoeffsScanV[], int32 MCU_Idx);

  void   xEstimateLambda(const xPicYUV* Picture);

  void   xOptQuantPic(int16* OptCoeffsScanV[], const int16* CoeffsScanV[], const xPicYUV* Picture);
  void   xOptQuantPicDCT(int16* OptCoeffsScanV[], const int16* CoeffsScanV[], const int16* CoeffsTransOrgV[]);
  void   xOptQuantSlc(int16* OptCoeffsScanV[], const int16* CoeffsScanV[], const xPicYUV* Picture, int32 MCU_IdxFirst, int32 MCU_IdxLast);
  void   xOptQuantSlcDCT(int16* OptCoeffsScanV[], const int16* CoeffsScanV[], const int16* CoeffsTransOrgV[], int32 MCU_IdxFirst, int32 MCU_IdxLast);
  void   xOptQuantMCU(int16* OptCoeffsScanV[], const int16* CoeffsScanV[], const uint16* CmpPtrV[], const int32 CmpStrideV[], int32 MCU_Idx);
  void   xOptQuantMCUdct(int16* OptCoeffsScanV[], const int16* CoeffsScanV[], const int16* CoeffsTransOrgV[], int32 MCU_Idx);
  void   xOptQuantBLK(int16* OptCoeffScan, const int16* CoeffsScan, const uint16* SamplesOrg, eCmp CmpId, int32 LastDC);
  void   xOptQuantBLKdct(int16* OptCoeffScan, const int16* CoeffsScan, const int16* CoeffsTransOrg, eCmp CmpId, int32 LastDC);
  uint64 xCalcDistBLK(const int16* ScanCoeffs, const uint16* SamplesOrg, int32 QuantTabId);
  uint64 xCalcDistBLKdct(const int16* ScanCoeffs, const int16* CoeffsTransOrg, int32 QuantTabId);

  void   xOptHuffPic(const int16* CoeffsScanV[]);
  void   xCntPic(const int16* CoeffsScanV[]);
  void   xCntRng(const int16* CoeffsScan, int32 CounterIdx, int32 HuffTabIdDC, int32 HuffTabIdAC, int32 BlockIdxBeg, int32 BlockIdxEnd, int32 BlocksPerSlice);

  void   xHuffEncPic(const int16* CoeffsScanV[]);
  void   xHuffEncSlc(const int16* CoeffsScanV[], int32 SliceIdx, int32 MCU_IdxFirst, int32 MCU_IdxLast);
  void   xHuffEncChk(const int16* CoeffsScanV[], int32 ChunkIdx, int32 MCU_IdxFirst, int32 MCU_IdxLast);
  void   xHuffEncMCU(const int16* CoeffsScanV[], int32 SliceIdx, int32 MCU_Idx);

  void xWritePic(xByteBuffer* OutputBuffer);
};

//=====================================================================================================================================================================================

} //end of namespace PMBB::JPEG