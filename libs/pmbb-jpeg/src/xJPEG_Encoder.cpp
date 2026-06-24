/*
    SPDX-FileCopyrightText: 2020-2024 Jakub Stankowski <jakub.stankowski@put.poznan.pl>
    SPDX-License-Identifier: BSD-3-Clause
*/
#include "xJPEG_Encoder.h"
#include "xJPEG_Transform.h"
#include "xJPEG_TransformConstants.h"
#include "xJPEG_Scan.h"
#include "xJPEG_Entropy.h"
#include "xMemory.h"
#include "xPixelOps.h"
#include "xDistortion.h"

namespace PMBB_NAMESPACE::JPEG {

//=====================================================================================================================================================================================

void xAdvancedEncoder::create(int32V2 PictureSize, eCrF ChromaFormat, xThreadPool* ThreadPool)
{
  initCodecCommon(PictureSize, ChromaFormat);

  for(int32 CmpIdx = 0; CmpIdx < m_NumOfComponents; CmpIdx++)
  {
    const int32 Area = m_MCUsMulWidth[CmpIdx] * m_MCUsMulHeight[CmpIdx];
    m_CmpCoeffsTransOrg[CmpIdx] = (int16*)xMemory::xAlignedMallocPageAuto(Area * sizeof(int16));
    m_CmpCoeffsTransRec[CmpIdx] = (int16*)xMemory::xAlignedMallocPageAuto(Area * sizeof(int16));
    m_CmpCoeffsScan    [CmpIdx] = (int16*)xMemory::xAlignedMallocPageAuto(Area * sizeof(int16));
    m_CmpCoeffsScanAux [CmpIdx] = (int16*)xMemory::xAlignedMallocPageAuto(Area * sizeof(int16));
    m_CmpCoeffsScanOpt [CmpIdx] = (int16*)xMemory::xAlignedMallocPageAuto(Area * sizeof(int16));
  }

  m_PicRec.create(PictureSize, 8, ChromaFormat, 16);

  if(ThreadPool != nullptr)
  {
    m_ThPI.init(ThreadPool, m_PictureSize.getY(), m_PictureSize.getY());
  }
}
void xAdvancedEncoder::destroy()
{
  m_ThPI.uninit();

  for(int32 CmpIdx = 0; CmpIdx < m_NumOfComponents; CmpIdx++)
  {
    xMemory::xAlignedFreeNull(m_CmpCoeffsTransOrg[CmpIdx]);
    xMemory::xAlignedFreeNull(m_CmpCoeffsTransRec[CmpIdx]);
    xMemory::xAlignedFreeNull(m_CmpCoeffsScan    [CmpIdx]);
    xMemory::xAlignedFreeNull(m_CmpCoeffsScanAux [CmpIdx]);
    xMemory::xAlignedFreeNull(m_CmpCoeffsScanOpt [CmpIdx]);
  }

  //m_EntropyBuffer.destroy();
  m_PicRec.destroy();
}
void xAdvancedEncoder::initBaseMarkers()
{
  //init markers
  m_APP0.InitDefault();
  m_SOF0.Init(m_PictureHeight, m_PictureWidth, 8, m_ChromaFormat, 2);
  m_SOS .Init(m_SOF0.getNumComponents(), 0, 1);
}
void xAdvancedEncoder::initQuant(int32 Quality, eQTLa QuantTabLayout)
{
  m_Quality = Quality;

  //init markers
  m_QTs.resize(2);
  m_QTs[0].Init(0, eCmp::LM, Quality, QuantTabLayout);
  m_QTs[1].Init(1, eCmp::CB, Quality, QuantTabLayout); //any chroma so use CB

  if(m_VerboseLevel >= 6)
  {
    std::string Dump = fmt::format("QuantizationTablesMain Num={}\n", m_QTs.size());
    for(int32 i = 0; i < (int32)m_QTs.size(); i++) { Dump += fmt::format("  QuantTab_{:d}\n", i) + m_QTs[i].Format("    "); }
    fmt::print("{}", Dump);
  }

  //init toolbox
  m_QuantMain.Init(m_QTs);
  m_QuantAuxD.Init(0, eCmp::LM, Quality - 2, QuantTabLayout);
  m_QuantAuxD.Init(1, eCmp::CB, Quality - 2, QuantTabLayout); //any chroma so use CB
  m_QuantAuxI.Init(0, eCmp::LM, Quality + 2, QuantTabLayout);
  m_QuantAuxI.Init(1, eCmp::CB, Quality + 2, QuantTabLayout); //any chroma so use CB

  if(m_VerboseLevel >= 6)
  {
    std::string Dump = fmt::format("QuantizerTablesMain\n");
    for(int32 i = 0; i < (int32)m_QTs.size(); i++)
    {
      Dump += fmt::format("  Table_{}\n", i) + m_QuantMain.getQuantizer(i).FormatCoeffs("    ");
    }
    Dump += fmt::format("QuantizerTablesAux\n");
    for(int32 i = 0; i < (int32)m_QTs.size(); i++)
    {
      Dump += fmt::format("  Table_{}\n", i) + m_QuantAuxD.getQuantizer(i).FormatCoeffs("    ");
    }
    fmt::print("{}", Dump);
  }
}
void xAdvancedEncoder::initEntropy(int32 RestartInterval)
{
  //init markers
  m_RestartInterval = RestartInterval;
  m_HTs.resize(4);
  m_HTs[0].InitDefault(0, xJFIF::xHuffTable::eHuffClass::DC, eCmp::LM);
  m_HTs[1].InitDefault(0, xJFIF::xHuffTable::eHuffClass::AC, eCmp::LM);
  m_HTs[2].InitDefault(1, xJFIF::xHuffTable::eHuffClass::DC, eCmp::CB); //any chroma so use CB
  m_HTs[3].InitDefault(1, xJFIF::xHuffTable::eHuffClass::AC, eCmp::CB); //any chroma so use CB

  //init toolbox
  m_NumOfSlices    = RestartInterval > 0 ? (int32)std::ceil((flt64)m_NumMCUsInArea / (flt64)RestartInterval) : 1;
  m_NumMCUsInSlice = RestartInterval != 0 ? RestartInterval : m_NumMCUsInArea;
  int32 NumBlocksInMCU       = std::accumulate(m_NumBlocksInMCU.cbegin(), m_NumBlocksInMCU.cend(), 0);
  int32 MaxEncodedSliceSize  = m_NumMCUsInSlice * NumBlocksInMCU * c_BA * 2;
  int32 MaxEncodedMCURowSize = m_NumMCUsInWidth * NumBlocksInMCU * c_BA * 2;
  int32 NumCounters          = m_ThPI.isActive() ? m_ThPI.getNumThreads() : 1;
  bool  EncodeMCU_Rows       = RestartInterval == 0 && m_ThPI.isActive();
  int32 NumEncoders          = !EncodeMCU_Rows ? m_NumOfSlices       : m_NumMCUsInHeight   ;
  int32 EncBufferSize        = !EncodeMCU_Rows ? MaxEncodedSliceSize : MaxEncodedMCURowSize;

  m_EntropyCnts.resize(NumCounters);

  m_EntropyEncs = std::vector<xEntropyEncoder>(NumEncoders);
  m_EncBuffers  .resize(NumEncoders);
  m_SliceBuffers.resize(NumEncoders);

  if(EncodeMCU_Rows)
  {
    m_NumAlignmentBits.resize(m_NumMCUsInHeight, 0);
    const int32 CombineBufferSize = m_NumMCUsInArea * NumBlocksInMCU * c_BA * 2;
    m_CombineBuffer.create(CombineBufferSize);
  }

  m_EntropyEst.Init(m_HTs);
  for(int32 i = 0; i < (int32)m_EntropyEncs .size(); i++) { m_EntropyEncs [i].Init(m_HTs); }
  for(int32 i = 0; i < (int32)m_EncBuffers  .size(); i++) { m_EncBuffers  [i].resize(EncBufferSize); }
  for(int32 i = 0; i < (int32)m_SliceBuffers.size(); i++) { m_SliceBuffers[i].resize(EncBufferSize); }
}
void xAdvancedEncoder::setMarkerEmit(bool EmitAPP0, bool EmitQuantTabs, bool EmitHuffmanTabs)
{
  m_EmitAPP0      = EmitAPP0       ;
  m_EmitQuantTabs = EmitQuantTabs  ;
  m_EmitHuffTabs  = EmitHuffmanTabs;
}
void xAdvancedEncoder::setQuantOpt(bool OptimizeLuma, bool OptimizeChroma, bool ProcessZeroCoeffs)
{ 
  m_UseRDOQ           = (OptimizeLuma || OptimizeChroma);
  m_OptQuantLuma      = OptimizeLuma;
  m_OptQuantChroma    = OptimizeChroma;
  m_ProcessZeroCoeffs = ProcessZeroCoeffs;  
}
void xAdvancedEncoder::setHuffOpt(bool OptimizeHuffmanTables)
{
  m_OptHuffTables = OptimizeHuffmanTables;
}
void xAdvancedEncoder::setOptPass(int32 NumBlockOptPasses, int32 NumPicOptPasses)
{
  m_NumOptPassesBlock = NumBlockOptPasses;
  m_NumOptPassesPic   = NumPicOptPasses  ;
}
void xAdvancedEncoder::setDctSsd(bool UseDctSsd)
{
  m_UseDctSsd = UseDctSsd;
}
void xAdvancedEncoder::setGreedyMultiPass(bool GreedyMultiPass)
{
  m_GreedyMultiPass = GreedyMultiPass;
}
void xAdvancedEncoder::setBeamSearch(bool BeamSearch, int32 BeamWidth, int32 BeamSteps) {
  m_BeamSearch = BeamSearch;
  m_BeamSteps = BeamSteps;
  m_BeamWidth = std::max(1, std::min(BeamWidth, c_MAX_BEAM_WIDTH));
}
void xAdvancedEncoder::encode(const xPicYUV* InputPicture, xByteBuffer* OutputBuffer)
{
  xEncodePicture(OutputBuffer, InputPicture);
}
//xAdvancedEncoder::tDistBits xAdvancedEncoder::calcDistBits(const xPicYUV* Picture)
//{
//  const int16* ConstCmpCoeffsTransOrg[] = { m_CmpCoeffsTransOrg[0], m_CmpCoeffsTransOrg[1], m_CmpCoeffsTransOrg[2], m_CmpCoeffsTransOrg[3] };
//  const int16* ConstCmpCoeffsTransRec[] = { m_CmpCoeffsTransRec[0], m_CmpCoeffsTransRec[1], m_CmpCoeffsTransRec[2], m_CmpCoeffsTransRec[3] };
//  const int16* ConstCmpCoeffsScan    [] = { m_CmpCoeffsScan    [0], m_CmpCoeffsScan    [1], m_CmpCoeffsScan    [2], m_CmpCoeffsScan    [3] };
//
//  xFwdTransformPic(m_CmpCoeffsTransOrg, Picture);
//  xFwdQuantScanPic(m_CmpCoeffsScan    , ConstCmpCoeffsTransOrg, m_QuantMain);
//  xInvScanQuantPic(m_CmpCoeffsTransRec, ConstCmpCoeffsScan    , m_QuantMain);
//  xInvTransformPic(&m_PicRec          , ConstCmpCoeffsTransRec);
//  int64V4 EstNumBits = xHuffEstPic (ConstCmpCoeffsScan);
//  int64V4 Distortion = xCalcPicSSDs(Picture, &m_PicRec);
//
//  return std::make_tuple(Distortion, EstNumBits);
//}
std::string xAdvancedEncoder::formatAndResetStats(const std::string Prefix, flt64 TicksPerMicroSec)
{
  if(!m_GatherTimeStats      ) { return "Time stats gathering is disabled!"; }
  if(m_TotalPictureIters == 0) { return "No time stats gathered!"; }

  flt64 InvDenom = TicksPerMicroSec == 0.0 ? (flt64)1.0 / ((flt64)m_TotalPictureIters) : (flt64)1.0 / ((flt64)m_TotalPictureIters * TicksPerMicroSec);

  std::string TimeStats; TimeStats.reserve(xMemory::c_MemSizePageBase);
  TimeStats += Prefix + fmt::format("Processing time {}\n", TicksPerMicroSec == 0.0 ? "[ticks]" : "[us]");

  TimeStats += Prefix + "  " + fmt::format("PicIters   = {}\n"    , m_TotalPictureIters);
  TimeStats += Prefix + "  " + fmt::format("Pic        = {:.2f}\n", m_Ticks___Picture * InvDenom);
  TimeStats += Prefix + "  " + fmt::format("Transform  = {:.2f}\n", m_Ticks_Transform * InvDenom);
  TimeStats += Prefix + "  " + fmt::format("QuantScan  = {:.2f}\n", m_Ticks_QuantScan * InvDenom);
  TimeStats += Prefix + "  " + fmt::format("LambdaEst  = {:.2f}\n", m_Ticks____Lambda * InvDenom);
  TimeStats += Prefix + "  " + fmt::format("OptQuant   = {:.2f}\n", m_Ticks__OptQuant * InvDenom);
  TimeStats += Prefix + "  " + fmt::format("OptHuff    = {:.2f}\n", m_Ticks___OptHuff * InvDenom);
  TimeStats += Prefix + "  " + fmt::format("EntropyEnc = {:.2f}\n", m_TicksEntropyEnc * InvDenom);
  TimeStats += Prefix + "  " + fmt::format("Tables     = {:.2f}\n", m_Ticks____Tables * InvDenom);
  TimeStats += Prefix + "  " + fmt::format("WriteOut   = {:.2f}\n", m_Ticks__WriteOut * InvDenom);
  
  m_Ticks_Transform = 0;
  m_Ticks_QuantScan = 0;
  m_Ticks____Lambda = 0;
  m_Ticks__OptQuant = 0;
  m_Ticks___OptHuff = 0;
  m_TicksEntropyEnc = 0;
  m_Ticks____Tables = 0;
  m_Ticks__WriteOut = 0;

  m_TotalPictureIters = 0;
  m_TotalSliceIters   = 0;

  return TimeStats;
}
std::string xAdvancedEncoder::formatStatsFile(flt64 TicksPerMicroSec)
{
  if (!m_GatherTimeStats) { return "Time stats gathering is disabled!"; }
  if (m_TotalPictureIters == 0) { return "No time stats gathered!"; }

  flt64 InvDenom = TicksPerMicroSec == 0.0 ? (flt64)1.0 / ((flt64)m_TotalPictureIters) : (flt64)1.0 / ((flt64)m_TotalPictureIters * TicksPerMicroSec);

  std::string TimeStats; TimeStats.reserve(xMemory::c_MemSizePageBase);

  TimeStats += fmt::format("OptQuant = {:.2f}\n", m_Ticks__OptQuant * InvDenom);

  return TimeStats;
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void xAdvancedEncoder::xEncodePicture(xByteBuffer* OutputBuffer, const xPicYUV* Picture)
{
  const int16* ConstCmpCoeffsTransOrg[] = { m_CmpCoeffsTransOrg[0], m_CmpCoeffsTransOrg[1], m_CmpCoeffsTransOrg[2], m_CmpCoeffsTransOrg[3] };
  const int16* ConstCmpCoeffsScan    [] = { m_CmpCoeffsScan    [0], m_CmpCoeffsScan    [1], m_CmpCoeffsScan    [2], m_CmpCoeffsScan    [3] };
  const int16* ConstCmpCoeffsScanOpt [] = { m_CmpCoeffsScanOpt [0], m_CmpCoeffsScanOpt [1], m_CmpCoeffsScanOpt [2], m_CmpCoeffsScanOpt [3] };

  if(m_OptHuffTables)
  {
    m_HTs[0].InitDefault(0, xJFIF::xHuffTable::eHuffClass::DC, eCmp::LM);
    m_HTs[1].InitDefault(0, xJFIF::xHuffTable::eHuffClass::AC, eCmp::LM);
    m_HTs[2].InitDefault(1, xJFIF::xHuffTable::eHuffClass::DC, eCmp::CB); //any chroma so use CB
    m_HTs[3].InitDefault(1, xJFIF::xHuffTable::eHuffClass::AC, eCmp::CB); //any chroma so use CB
  }

  uint64 TP0 = m_GatherTimeStats ? xTSC() : 0;

  xFwdTransformPic(m_CmpCoeffsTransOrg, Picture);

  uint64 TP1 = m_GatherTimeStats ? xTSC() : 0;

  xFwdQuantScanPic(m_CmpCoeffsScan, ConstCmpCoeffsTransOrg, m_QuantMain);

  uint64 TP2 = m_GatherTimeStats ? xTSC() : 0;

  if(m_UseRDOQ) { xEstimateLambda(Picture); }

  uint64 TP3 = m_GatherTimeStats ? xTSC() : 0;

  for(int32 n = 0; n < m_NumOptPassesPic; n++)
  {
    uint64 TPo0 = m_GatherTimeStats ? xTSC() : 0;

    if (m_UseRDOQ) {
      if (m_UseDctSsd) {
        xOptQuantPicDCT(m_CmpCoeffsScanOpt, ConstCmpCoeffsScan, ConstCmpCoeffsTransOrg);
      }
      else {
        xOptQuantPic(m_CmpCoeffsScanOpt, ConstCmpCoeffsScan, Picture);
      }
    }

    uint64 TPo1 = m_GatherTimeStats ? xTSC() : 0;

    if(m_OptHuffTables) { xOptHuffPic(m_UseRDOQ ? ConstCmpCoeffsScanOpt : ConstCmpCoeffsScan); }

    if(m_GatherTimeStats) { m_Ticks__OptQuant += TPo1 - TPo0; m_Ticks___OptHuff += xTSC() - TPo1; }
  }

  uint64 TP4 = m_GatherTimeStats ? xTSC() : 0;

  xHuffEncPic(m_UseRDOQ ? ConstCmpCoeffsScanOpt : ConstCmpCoeffsScan);

  uint64 TP5 = m_GatherTimeStats ? xTSC() : 0;

  xJFIF::WriteSOI(OutputBuffer);
  if(m_EmitAPP0       ) { xJFIF::WriteAPP0(OutputBuffer, m_APP0); }
  if(m_EmitQuantTabs  ) { xJFIF::WriteDQT(OutputBuffer, m_QTs); }
  if(m_RestartInterval) { xJFIF::WriteDRI(OutputBuffer, m_RestartInterval); }
  xJFIF::WriteSOF0(OutputBuffer, m_SOF0);
  if(m_EmitHuffTabs   ) { xJFIF::WriteDHT(OutputBuffer, m_HTs); }

  uint64 TP6 = m_GatherTimeStats ? xTSC() : 0;

  xJFIF::WriteSOS(OutputBuffer, m_SOS);
  xWritePic(OutputBuffer);
  xJFIF::WriteEOI(OutputBuffer);

  uint64 TP7 = m_GatherTimeStats ? xTSC() : 0;
  
  m_TotalPictureIters += 1;
  if(m_GatherTimeStats)
  {
    m_Ticks___Picture += TP6 - TP0;
    m_Ticks_Transform += TP1 - TP0;
    m_Ticks_QuantScan += TP2 - TP1;
    m_Ticks____Lambda += TP3 - TP2;
    m_TicksEntropyEnc += TP5 - TP4;
    m_Ticks____Tables += TP6 - TP5;
    m_Ticks__WriteOut += TP7 - TP6;
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

int64V4 xAdvancedEncoder::xCalcPicSSDs(const xPicYUV* Tst, const xPicYUV* Ref)
{
  int64V4 SSDs = xMakeVec4<int64>(0);
  for(int32 CmpIdx = 0; CmpIdx < m_NumOfComponents; CmpIdx++)
  {    
    m_ThPI.storeTask([&SSDs, &Tst, &Ref, CmpIdx] (int32 /*ThIdx*/) { eCmp c = (eCmp)CmpIdx; SSDs[CmpIdx] = xDistortion::CalcSSD(Tst->getAddr(c), Ref->getAddr(c), Tst->getStride(c), Ref->getStride(c), Tst->getWidth(c), Tst->getHeight(c)); });
  }
  m_ThPI.executeStoredTasks();

  return SSDs;
}
void xAdvancedEncoder::xFwdTransformPic(int16* CoeffsTransV[], const xPicYUV* Picture)
{
  const uint16* CmpPtrV   [] = {Picture->getAddr  (eCmp::LM), Picture->getAddr  (eCmp::CB), Picture->getAddr  (eCmp::CR), nullptr};
  const int32   CmpStrideV[] = {Picture->getStride(eCmp::LM), Picture->getStride(eCmp::CB), Picture->getStride(eCmp::CR),       0};

  for(int32 MCU_RowIdx = 0; MCU_RowIdx < m_NumMCUsInHeight; MCU_RowIdx++) //loop over MCUs rows
  {
    const int32 MCU_IdxBeg = MCU_RowIdx * m_NumMCUsInWidth;
    const int32 MCU_IdxEnd = MCU_IdxBeg + m_NumMCUsInWidth;
    m_ThPI.storeTask([this, &CoeffsTransV, &CmpPtrV, &CmpStrideV, MCU_IdxBeg, MCU_IdxEnd](int32 /*ThIdx*/) { xFwdTransformSlc(CoeffsTransV, CmpPtrV, CmpStrideV, MCU_IdxBeg, MCU_IdxEnd); });
  }
  m_ThPI.executeStoredTasks();
}
void xAdvancedEncoder::xFwdTransformSlc(int16* CoeffsTransV[], const uint16* CmpPtrV[], const int32 CmpStrideV[], int32 MCU_IdxBeg, int32 MCU_IdxEnd)
{
  //fmt::print("MCU_IdxBeg={}\n", MCU_IdxBeg);
  for(int32 MCU_Idx = MCU_IdxBeg; MCU_Idx < MCU_IdxEnd; MCU_Idx++) //loop over MCUs
  {
    xFwdTransformMCU(CoeffsTransV, CmpPtrV, CmpStrideV, MCU_Idx);
  }
}
void xAdvancedEncoder::xFwdTransformMCU(int16* CoeffsTransV[], const uint16* CmpPtrV[], const int32 CmpStrideV[], int32 MCU_Idx)
{
  //calculate MCU position
  int32 MCU_PosV = MCU_Idx / m_NumMCUsInWidth;
  int32 MCU_PosH = MCU_Idx % m_NumMCUsInWidth;

  //org samples buffer
  uint16 SamplesOrg[c_BA];

  //transform blocks
  for(int32 CmpIdx = 0; CmpIdx < m_NumOfComponents; CmpIdx++)
  {
    const int32 MCU_PelPosV = MCU_PosV << (2 + m_SampFactorVer[CmpIdx]);
    const int32 MCU_PelPosH = MCU_PosH << (2 + m_SampFactorHor[CmpIdx]);

    const uint16* CmpPtr    = CmpPtrV   [CmpIdx];
    const int32   CmpStride = CmpStrideV[CmpIdx];

    int32 BlockIdx = MCU_Idx * m_SampFactorVer[CmpIdx] * m_SampFactorHor[CmpIdx];
    for(int32 V = 0; V < m_SampFactorVer[CmpIdx]; V++)
    {
      const int32 BlockPosV = MCU_PelPosV + V * c_BS;
      const int32 BlockResV = m_CmpHeight[CmpIdx] - BlockPosV;
      for(int32 H = 0; H < m_SampFactorHor[CmpIdx]; H++)
      {        
        const int32   BlockPosH = MCU_PelPosH + H * c_BS;
        const int32   BlockResH = m_CmpWidth[CmpIdx] - BlockPosH;
        const uint16* BlockPtr  = CmpPtr + BlockPosV * CmpStride + BlockPosH;     
        if     (BlockResV >= 8 && BlockResH >= 8) { loadEntireBlock(SamplesOrg, BlockPtr, CmpStride); } //C++20 TODO use [[likely]]
        else if(BlockResV >  0 && BlockResH >  0) { loadExtendBlock(SamplesOrg, BlockPtr, CmpStride, xMin(BlockResH, c_BS), xMin(BlockResV, c_BS)); }
        else                                      { zeroEntireBlock(SamplesOrg); }

        const int32 CoeffTransOffset = BlockIdx << c_L2BA;
        xTransform::FwdTransformDCT_8x8(CoeffsTransV[CmpIdx] + CoeffTransOffset, SamplesOrg);
        CoeffsTransV[CmpIdx][CoeffTransOffset] -= xTransformConstants::c_FwdDcCorr; //DC correction - JPEG requires 128 to be subtracted from every input sample - could be done be DC -= 
        BlockIdx++;
      }
    }
  }
}
void xAdvancedEncoder::xInvTransformPic(xPicYUV* Picture, const int16* CoeffsTransV[])
{
        uint16* CmpPtrV   [] = {Picture->getAddr  (eCmp::LM), Picture->getAddr  (eCmp::CB), Picture->getAddr  (eCmp::CR), nullptr};
  const int32   CmpStrideV[] = {Picture->getStride(eCmp::LM), Picture->getStride(eCmp::CB), Picture->getStride(eCmp::CR),       0};

  for(int32 MCU_RowIdx = 0; MCU_RowIdx < m_NumMCUsInHeight; MCU_RowIdx++) //loop over MCUs rows
  {
    const int32 MCU_IdxBeg = MCU_RowIdx * m_NumMCUsInWidth;
    const int32 MCU_IdxEnd = MCU_IdxBeg + m_NumMCUsInWidth;
    m_ThPI.storeTask([this, &CmpPtrV, &CmpStrideV, &CoeffsTransV, MCU_IdxBeg, MCU_IdxEnd](int32 /*ThIdx*/) { xInvTransformSlc(CmpPtrV, CmpStrideV, CoeffsTransV, MCU_IdxBeg, MCU_IdxEnd); });
  }
  m_ThPI.executeStoredTasks();
}
void xAdvancedEncoder::xInvTransformSlc(uint16* CmpPtrV[], const int32 CmpStrideV[], const int16* CoeffsTransV[], int32 MCU_IdxBeg, int32 MCU_IdxEnd)
{
  for(int32 MCU_Idx = MCU_IdxBeg; MCU_Idx < MCU_IdxEnd; MCU_Idx++) //loop over MCUs
  {
    xInvTransformMCU(CmpPtrV, CmpStrideV, CoeffsTransV, MCU_Idx);
  }
}
void xAdvancedEncoder::xInvTransformMCU(uint16* CmpPtrV[], const int32 CmpStrideV[], const int16* CoeffsTransV[], int32 MCU_Idx)
{
  //calculate MCU position
  int32 MCU_PosV = MCU_Idx / m_NumMCUsInWidth;
  int32 MCU_PosH = MCU_Idx % m_NumMCUsInWidth;

  //org samples buffer
  int16  CoeffsTrans[c_BA];
  uint16 SamplesRec [c_BA];

  //transform blocks
  for(int32 CmpIdx = 0; CmpIdx < m_NumOfComponents; CmpIdx++)
  {
    const int32 MCU_PelPosV = MCU_PosV << (2 + m_SampFactorVer[CmpIdx]);
    const int32 MCU_PelPosH = MCU_PosH << (2 + m_SampFactorHor[CmpIdx]);

          uint16* CmpPtr    = CmpPtrV   [CmpIdx];
    const int32   CmpStride = CmpStrideV[CmpIdx];

    int32 BlockIdx = MCU_Idx * m_SampFactorVer[CmpIdx] * m_SampFactorHor[CmpIdx];
    for(int32 V = 0; V < m_SampFactorVer[CmpIdx]; V++)
    {
      const int32 BlockPosV = MCU_PelPosV + V * c_BS;
      const int32 BlockResV = m_CmpHeight[CmpIdx] - BlockPosV;
      for(int32 H = 0; H < m_SampFactorHor[CmpIdx]; H++)
      {
        const int32 BlockPosH = MCU_PelPosH + H * c_BS;
        const int32 BlockResH = m_CmpWidth[CmpIdx] - BlockPosH;
        uint16* restrict BlockPtr = CmpPtr + BlockPosV * CmpStride + BlockPosH;
        const int32 CoeffTransOffset = BlockIdx << c_L2BA;

        memcpy(CoeffsTrans, CoeffsTransV[CmpIdx] + CoeffTransOffset, c_BA * sizeof(int16));
        CoeffsTrans[0] += xTransformConstants::c_InvDcCorr; //DC correction - JPEG requires 128 to be subtracted from every input sample - could be done be DC -= 
        xTransform::InvTransformDCT_8x8(SamplesRec, CoeffsTrans);

        if     (BlockResV >= 8 && BlockResH >= 8) { storeEntireBlock (BlockPtr, SamplesRec, CmpStride); } //C++20 TODO use [[likely]]
        else if(BlockResV >  0 && BlockResH >  0) { storePartialBlock(BlockPtr, SamplesRec, CmpStride, xMin(BlockResH, c_BS), xMin(BlockResV, c_BS)); }
        else                                      { /* do nothing */ }

        BlockIdx++;
      }
    }
  }
}
void xAdvancedEncoder::xFwdQuantScanPic(int16* CoeffsScanV[], const int16* CoeffsTransV[], const xQuantizerSet& Quant)
{
  for(int32 CmpIdx = 0; CmpIdx < m_NumOfComponents; CmpIdx++)
  {
    const int32       QuantTabIdx       = m_SOF0.getQuantTableId(eCmp(CmpIdx));
    const int32       NumBlocksInWidth  = m_NumBlocksInWidth [CmpIdx];
    const int32       NumBlocksInHeight = m_NumBlocksInHeight[CmpIdx];
    const xQuantizer* Quantizer         = &Quant.getQuantizer(QuantTabIdx);

    int16*       CoeffsScan  = CoeffsScanV [CmpIdx];
    const int16* CoeffsTrans = CoeffsTransV[CmpIdx];

    for(int32 BlockRowIdx = 0; BlockRowIdx < NumBlocksInHeight; BlockRowIdx++) //loop over block rows
    {
      const int32 BlockIdxBeg = BlockRowIdx * NumBlocksInWidth;
      const int32 BlockIdxEnd = BlockIdxBeg + NumBlocksInWidth;

      m_ThPI.storeTask([CoeffsScan, CoeffsTrans, Quantizer, BlockIdxBeg, BlockIdxEnd](int32 /*ThIdx*/) { xFwdQuantScanRng(CoeffsScan, CoeffsTrans, *Quantizer, BlockIdxBeg, BlockIdxEnd); });      
    }
  }
  m_ThPI.executeStoredTasks();
}
void xAdvancedEncoder::xFwdQuantScanRng(int16* CoeffScan, const int16* CoeffTrans, const xQuantizer& Quant, int32 BlockIdxBeg, int32 BlockIdxEnd)
{
  int16 CoeffQuant[c_BA];

  for(int32 BlockIdx = BlockIdxBeg; BlockIdx < BlockIdxEnd; BlockIdx++)
  {
    const int32 BlockOffset = BlockIdx << c_L2BA;
    Quant.QuantScale(CoeffQuant, CoeffTrans + BlockOffset);
    xScan::Scan(CoeffScan + BlockOffset, CoeffQuant);
  }
}
void xAdvancedEncoder::xInvScanQuantPic(int16* CoeffsTransV[], const int16* CoeffsScanV[], const xQuantizerSet& Quant)
{
  for(int32 CmpIdx = 0; CmpIdx < m_NumOfComponents; CmpIdx++)
  {
    const int32       QuantTabIdx       = m_SOF0.getQuantTableId(eCmp(CmpIdx));
    const int32       NumBlocksInWidth  = m_NumBlocksInWidth [CmpIdx];
    const int32       NumBlocksInHeight = m_NumBlocksInHeight[CmpIdx];
    const xQuantizer* Quantizer         = &Quant.getQuantizer(QuantTabIdx);

    int16*       CoeffsTrans = CoeffsTransV[CmpIdx];
    const int16* CoeffsScan  = CoeffsScanV [CmpIdx];

    for(int32 BlockRowIdx = 0; BlockRowIdx < NumBlocksInHeight; BlockRowIdx++) //loop over block rows
    {
      const int32 BlockIdxBeg = BlockRowIdx * NumBlocksInWidth;
      const int32 BlockIdxEnd = BlockIdxBeg + NumBlocksInWidth;

      m_ThPI.storeTask([CoeffsTrans, CoeffsScan, Quantizer, BlockIdxBeg, BlockIdxEnd](int32 /*ThIdx*/) {xInvScanQuantRng(CoeffsTrans, CoeffsScan, *Quantizer, BlockIdxBeg, BlockIdxEnd); });
    }
  }
  m_ThPI.executeStoredTasks();
}
void xAdvancedEncoder::xInvScanQuantRng(int16* CoeffTrans, const int16* CoeffScan, const xQuantizer& Quant, int32 BlockIdxBeg, int32 BlockIdxEnd)
{
  int16 CoeffQuant[c_BA];

  for(int32 BlockIdx = BlockIdxBeg; BlockIdx < BlockIdxEnd; BlockIdx++)
  {
    const int32 BlockOffset = BlockIdx << c_L2BA;
    xScan::InvScan(CoeffQuant, CoeffScan + BlockOffset);
    Quant.InvScale(CoeffTrans + BlockOffset, CoeffQuant);
  }
}
int64V4 xAdvancedEncoder::xHuffEstPic(const int16* CoeffsScanV[])
{
  std::vector<int64V4> EstNumBits;
  if(m_RestartInterval == 0) //no division - however, process picture in MCU rows
  {
    EstNumBits.resize(m_NumMCUsInHeight);
    for(int32 MCU_RowIdx = 0; MCU_RowIdx < m_NumMCUsInHeight; MCU_RowIdx++) //loop over MCUs rows
    {
      const int32 MCU_IdxFirst = MCU_RowIdx   * m_NumMCUsInWidth    ;
      const int32 MCU_IdxLast  = MCU_IdxFirst + m_NumMCUsInWidth - 1;
      m_ThPI.storeTask([this, &EstNumBits, MCU_RowIdx, &CoeffsScanV, MCU_IdxFirst, MCU_IdxLast](int32 /*ThIdx*/) { EstNumBits[MCU_RowIdx] = xHuffEstSlc(CoeffsScanV, MCU_IdxFirst, MCU_IdxLast); });
    }
  }
  else //divide picture into independent slices
  {
    EstNumBits.resize(m_NumOfSlices);
    for(int32 SliceIdx = 0; SliceIdx < m_NumOfSlices; SliceIdx++) //loop over slices
    {
      const int32 MCU_IdxFirst = SliceIdx * m_RestartInterval;
      const int32 MCU_IdxLast  = xMin(m_NumMCUsInArea, MCU_IdxFirst + m_NumMCUsInSlice) - 1;
      m_ThPI.storeTask([this, &EstNumBits, SliceIdx, &CoeffsScanV, MCU_IdxFirst, MCU_IdxLast](int32 /*ThIdx*/) { EstNumBits[SliceIdx] = xHuffEstSlc(CoeffsScanV, MCU_IdxFirst, MCU_IdxLast); });
    }
  }
  m_ThPI.executeStoredTasks();

  int64V4 TotalEstNumBits = std::accumulate(EstNumBits.cbegin(), EstNumBits.cend(), xMakeVec4<int64>(0));
  return TotalEstNumBits;
}
int64V4 xAdvancedEncoder::xHuffEstSlc(const int16* CoeffsScanV[], int32 MCU_IdxFirst, int32 MCU_IdxLast)
{
  int64V4 EstNumBits = xMakeVec4<int64>(0);  
  for(int32 MCU_Idx = MCU_IdxFirst; MCU_Idx <= MCU_IdxLast; MCU_Idx++) //loop over MCUs
  {
    EstNumBits += xHuffEstMCU(CoeffsScanV, MCU_Idx);
  }
  return EstNumBits;
}
int64V4 xAdvancedEncoder::xHuffEstMCU(const int16* CoeffsScanV[], int32 MCU_Idx)
{
  int64V4 EstNumBits = xMakeVec4<int64>(0);

  //estimate blocks
  for(int32 CmpIdx = 0; CmpIdx < m_NumOfComponents; CmpIdx++)
  {
    int32 HuffTabIdDC = m_SOS.getHuffTableIdDC(eCmp(CmpIdx));
    int32 HuffTabIdAC = m_SOS.getHuffTableIdAC(eCmp(CmpIdx));
    int32 BlockIdx    = MCU_Idx * m_SampFactorVer[CmpIdx] * m_SampFactorHor[CmpIdx];
    
    for(int32 V = 0; V < m_SampFactorVer[CmpIdx]; V++)
    {
      for(int32 H = 0; H < m_SampFactorHor[CmpIdx]; H++)
      {        
        const int32 CoeffOffset = BlockIdx << c_L2BA;
        const bool  FirstInSlc  = BlockIdx == 0 || (m_RestartInterval > 0 && MCU_Idx % m_RestartInterval == 0);
        const int32 LastDC      = FirstInSlc ? 0 : CoeffsScanV[CmpIdx][CoeffOffset - c_BA];

        EstNumBits[CmpIdx] += m_EntropyEst.EstimateBlock(CoeffsScanV[CmpIdx] + CoeffOffset, LastDC, HuffTabIdDC, HuffTabIdAC);
        BlockIdx++;
      }
    }
  }
  return EstNumBits;
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void xAdvancedEncoder::xEstimateLambda(const xPicYUV* Picture)
{
  const int16* ConstCmpCoeffsTransOrg[] = { m_CmpCoeffsTransOrg[0], m_CmpCoeffsTransOrg[1], m_CmpCoeffsTransOrg[2], m_CmpCoeffsTransOrg[3] };
  const int16* ConstCmpCoeffsTransRec[] = { m_CmpCoeffsTransRec[0], m_CmpCoeffsTransRec[1], m_CmpCoeffsTransRec[2], m_CmpCoeffsTransRec[3] };
  const int16* ConstCmpCoeffsScan    [] = { m_CmpCoeffsScan    [0], m_CmpCoeffsScan    [1], m_CmpCoeffsScan    [2], m_CmpCoeffsScan    [3] };
  const int16* ConstCmpCoeffsScanAux [] = { m_CmpCoeffsScanAux [0], m_CmpCoeffsScanAux [1], m_CmpCoeffsScanAux [2], m_CmpCoeffsScanAux [3] };

  //base point
  xInvScanQuantPic(m_CmpCoeffsTransRec, ConstCmpCoeffsScan, m_QuantMain);
  xInvTransformPic(&m_PicRec          , ConstCmpCoeffsTransRec);
  int64V4 EstNumBitsMain = xHuffEstPic (ConstCmpCoeffsScan);
  int64V4 DistortionMain = xCalcPicSSDs(Picture, &m_PicRec);

  //lower point
  int64V4 EstNumBitsAuxD = { 0,0,0,0 };
  int64V4 DistortionAuxD = { 0,0,0,0 };
  if(m_Quality > 1)
  {
    xFwdQuantScanPic(m_CmpCoeffsScanAux , ConstCmpCoeffsTransOrg, m_QuantAuxD);
    xInvScanQuantPic(m_CmpCoeffsTransRec, ConstCmpCoeffsScanAux , m_QuantAuxD);
    xInvTransformPic(&m_PicRec          , ConstCmpCoeffsTransRec);
    EstNumBitsAuxD = xHuffEstPic (ConstCmpCoeffsScanAux);
    DistortionAuxD = xCalcPicSSDs(Picture, &m_PicRec);
  }
  
  //higher point
  int64V4 EstNumBitsAuxI = { 0,0,0,0 };
  int64V4 DistortionAuxI = { 0,0,0,0 };
  if(m_Quality < 100)
  {
    xFwdQuantScanPic(m_CmpCoeffsScanAux , ConstCmpCoeffsTransOrg, m_QuantAuxI);
    xInvScanQuantPic(m_CmpCoeffsTransRec, ConstCmpCoeffsScanAux , m_QuantAuxI);
    xInvTransformPic(&m_PicRec, ConstCmpCoeffsTransRec);
    EstNumBitsAuxI = xHuffEstPic(ConstCmpCoeffsScanAux);
    DistortionAuxI = xCalcPicSSDs(Picture, &m_PicRec);
  }

  //local lambda
  int64V4 DeltaEstNumBitsD = EstNumBitsMain - EstNumBitsAuxD;
  int64V4 DeltaDistortionD = DistortionMain - DistortionAuxD;
  int64V4 DeltaEstNumBitsI = EstNumBitsMain - EstNumBitsAuxI;
  int64V4 DeltaDistortionI = DistortionMain - DistortionAuxI;

  flt64V4 LambdaD = -(flt64V4)DeltaDistortionD / (flt64V4)DeltaEstNumBitsD;
  flt64V4 LambdaI = -(flt64V4)DeltaDistortionI / (flt64V4)DeltaEstNumBitsI;
  if     (m_Quality > 1 && m_Quality < 100) { m_Lambda = (LambdaD + LambdaI) / 2.0; }
  else if(m_Quality > 1                   ) { m_Lambda = LambdaD; }
  else if(m_Quality < 100                 ) { m_Lambda = LambdaI; }

  if(m_VerboseLevel >= 5)
  {
    std::string Dump = "LambdaEstimation\n";
    Dump += fmt::format("QuantMain EstNumBits={:d} {:d} {:d}    Distortion={:d} {:d} {:d}\n", EstNumBitsMain[0], EstNumBitsMain[1], EstNumBitsMain[2], DistortionMain[0], DistortionMain[1], DistortionMain[2]);
    Dump += fmt::format("QuantAuxD EstNumBits={:d} {:d} {:d}    Distortion={:d} {:d} {:d}\n", EstNumBitsAuxD[0], EstNumBitsAuxD[1], EstNumBitsAuxD[2], DistortionAuxD[0], DistortionAuxD[1], DistortionAuxD[2]);
    Dump += fmt::format("QuantAuxI EstNumBits={:d} {:d} {:d}    Distortion={:d} {:d} {:d}\n", EstNumBitsAuxI[0], EstNumBitsAuxI[1], EstNumBitsAuxI[2], DistortionAuxI[0], DistortionAuxI[1], DistortionAuxI[2]);
    Dump += fmt::format("LambdaD = {} {} {}\n", LambdaD[0], LambdaD[1], LambdaD[2]);
    Dump += fmt::format("LambdaI = {} {} {}\n", LambdaI[0], LambdaI[1], LambdaI[2]);
    Dump += fmt::format("Lambda  = {} {} {}\n", m_Lambda[0], m_Lambda[1], m_Lambda[2]);
    fmt::print("{}", Dump);
  }
}

void xAdvancedEncoder::xOptQuantPic(int16* OptCoeffsScanV[], const int16* CoeffsScanV[], const xPicYUV* Picture)
{
  if(m_RestartInterval == 0) //no division - encode entire picture at once
  {
    for(int32 MCU_RowIdx = 0; MCU_RowIdx < m_NumMCUsInHeight; MCU_RowIdx++) //loop over MCUs rows
    {
      const int32 MCU_IdxFirst = MCU_RowIdx   * m_NumMCUsInWidth    ;
      const int32 MCU_IdxLast  = MCU_IdxFirst + m_NumMCUsInWidth - 1;
      m_ThPI.storeTask([this, &OptCoeffsScanV, &CoeffsScanV, &Picture, MCU_IdxFirst, MCU_IdxLast](int32 /*ThIdx*/) { xOptQuantSlc(OptCoeffsScanV, CoeffsScanV, Picture, MCU_IdxFirst, MCU_IdxLast); });
    }
  }
  else //divide picture into independent slices
  {
    for(int32 SliceIdx = 0; SliceIdx < m_NumOfSlices; SliceIdx++)
    {
      const int32 MCU_IdxFirst = SliceIdx * m_RestartInterval;
      const int32 MCU_IdxLast  = xMin(m_NumMCUsInArea, MCU_IdxFirst + m_RestartInterval) - 1;
      m_ThPI.storeTask([this, &OptCoeffsScanV, &CoeffsScanV, &Picture, MCU_IdxFirst, MCU_IdxLast](int32 /*ThIdx*/) {xOptQuantSlc(OptCoeffsScanV, CoeffsScanV, Picture, MCU_IdxFirst, MCU_IdxLast); });
    }
  }
  m_ThPI.executeStoredTasks();

  if(!m_OptQuantLuma)
  {
    memcpy(m_CmpCoeffsScanOpt[(int32)eCmp::LM], m_CmpCoeffsScan[(int32)eCmp::LM], m_MCUsMulArea[(int32)eCmp::LM] * sizeof(int16));
  }
  if(!m_OptQuantChroma)
  {
    memcpy(m_CmpCoeffsScanOpt[(int32)eCmp::CB], m_CmpCoeffsScan[(int32)eCmp::CB], m_MCUsMulArea[(int32)eCmp::CB] * sizeof(int16));
    memcpy(m_CmpCoeffsScanOpt[(int32)eCmp::CR], m_CmpCoeffsScan[(int32)eCmp::CR], m_MCUsMulArea[(int32)eCmp::CR] * sizeof(int16));
  }
}
void xAdvancedEncoder::xOptQuantPicDCT(int16* OptCoeffsScanV[], const int16* CoeffsScanV[], const int16* CoeffsTransOrgV[])
{
  if (m_RestartInterval == 0) //no division - encode entire picture at once
  {
    for (int32 MCU_RowIdx = 0; MCU_RowIdx < m_NumMCUsInHeight; MCU_RowIdx++) //loop over MCUs rows
    {
      const int32 MCU_IdxFirst = MCU_RowIdx * m_NumMCUsInWidth;
      const int32 MCU_IdxLast = MCU_IdxFirst + m_NumMCUsInWidth - 1;
      m_ThPI.storeTask([this, &OptCoeffsScanV, &CoeffsScanV, &CoeffsTransOrgV, MCU_IdxFirst, MCU_IdxLast](int32 /*ThIdx*/) { xOptQuantSlcDCT(OptCoeffsScanV, CoeffsScanV, CoeffsTransOrgV, MCU_IdxFirst, MCU_IdxLast); });
    }
  }
  else //divide picture into independent slices
  {
    for (int32 SliceIdx = 0; SliceIdx < m_NumOfSlices; SliceIdx++)
    {
      const int32 MCU_IdxFirst = SliceIdx * m_RestartInterval;
      const int32 MCU_IdxLast = xMin(m_NumMCUsInArea, MCU_IdxFirst + m_RestartInterval) - 1;
      m_ThPI.storeTask([this, &OptCoeffsScanV, &CoeffsScanV, &CoeffsTransOrgV, MCU_IdxFirst, MCU_IdxLast](int32 /*ThIdx*/) {xOptQuantSlcDCT(OptCoeffsScanV, CoeffsScanV, CoeffsTransOrgV, MCU_IdxFirst, MCU_IdxLast); });
    }
  }
  m_ThPI.executeStoredTasks();

  if (!m_OptQuantLuma)
  {
    memcpy(m_CmpCoeffsScanOpt[(int32)eCmp::LM], m_CmpCoeffsScan[(int32)eCmp::LM], m_MCUsMulArea[(int32)eCmp::LM] * sizeof(int16));
  }
  if (!m_OptQuantChroma)
  {
    memcpy(m_CmpCoeffsScanOpt[(int32)eCmp::CB], m_CmpCoeffsScan[(int32)eCmp::CB], m_MCUsMulArea[(int32)eCmp::CB] * sizeof(int16));
    memcpy(m_CmpCoeffsScanOpt[(int32)eCmp::CR], m_CmpCoeffsScan[(int32)eCmp::CR], m_MCUsMulArea[(int32)eCmp::CR] * sizeof(int16));
  }
}
void xAdvancedEncoder::xOptQuantSlc(int16* OptCoeffsScanV[], const int16* CoeffsScanV[], const xPicYUV* Picture, int32 MCU_IdxFirst, int32 MCU_IdxLast)
{
  const uint16* CmpPtrV   [] = {Picture->getAddr  (eCmp::LM), Picture->getAddr  (eCmp::CB), Picture->getAddr  (eCmp::CR), nullptr};
  const int32   CmpStrideV[] = {Picture->getStride(eCmp::LM), Picture->getStride(eCmp::CB), Picture->getStride(eCmp::CR),       0};

  //loop over MCUs
  for(int32 MCU_Idx = MCU_IdxFirst; MCU_Idx <= MCU_IdxLast; MCU_Idx++)
  {
    xOptQuantMCU(OptCoeffsScanV, CoeffsScanV, CmpPtrV, CmpStrideV, MCU_Idx);
  }
}
void xAdvancedEncoder::xOptQuantSlcDCT(int16* OptCoeffsScanV[], const int16* CoeffsScanV[], const int16* CoeffsTransOrgV[], int32 MCU_IdxFirst, int32 MCU_IdxLast)
{
  //loop over MCUs
  for (int32 MCU_Idx = MCU_IdxFirst; MCU_Idx <= MCU_IdxLast; MCU_Idx++)
  {
    xOptQuantMCUdct(OptCoeffsScanV, CoeffsScanV, CoeffsTransOrgV, MCU_Idx);
  }
}
void xAdvancedEncoder::xOptQuantMCU(int16* OptCoeffsScanV[], const int16* CoeffsScanV[], const uint16* CmpPtrV[], const int32 CmpStrideV[], int32 MCU_Idx)
{
  //calculate MCU position
  int32 MCU_PosV = MCU_Idx / m_NumMCUsInWidth;
  int32 MCU_PosH = MCU_Idx % m_NumMCUsInWidth;

  //org samples buffer
  uint16 SamplesOrg[c_BA];

  //transform blocks
  for(int32 CmpIdx = 0; CmpIdx < m_NumOfComponents; CmpIdx++)
  {
    if(m_UseRDOQ && ((CmpIdx == (int32)eCmp::LM && m_OptQuantLuma) || (CmpIdx != (int32)eCmp::LM && m_OptQuantChroma)))
    {
      const int32 MCU_PelPosV = MCU_PosV << (2 + m_SampFactorVer[CmpIdx]);
      const int32 MCU_PelPosH = MCU_PosH << (2 + m_SampFactorHor[CmpIdx]);

      const uint16* CmpPtr    = CmpPtrV   [CmpIdx];
      const int32   CmpStride = CmpStrideV[CmpIdx];

      int32 BlockIdx = MCU_Idx * m_SampFactorVer[CmpIdx] * m_SampFactorHor[CmpIdx];
      for(int32 V = 0; V < m_SampFactorVer[CmpIdx]; V++)
      {
        const int32 BlockPosV = MCU_PelPosV + V * c_BS;
        const int32 BlockResV = m_CmpHeight[CmpIdx] - BlockPosV;

        for(int32 H = 0; H < m_SampFactorHor[CmpIdx]; H++)
        {
          const int32 BlockPosH = MCU_PelPosH + H * c_BS;
          const int32 BlockResH = m_CmpWidth[CmpIdx] - BlockPosH;

          const uint16* BlockPtr = CmpPtr + BlockPosV * CmpStride + BlockPosH;          

          if     (BlockResV >= 8 && BlockResH >= 8) { loadEntireBlock(SamplesOrg, BlockPtr, CmpStride); } //C++20 TODO use [[likely]]
          else if(BlockResV >  0 && BlockResH >  0) { loadExtendBlock(SamplesOrg, BlockPtr, CmpStride, xMin(BlockResH, c_BS), xMin(BlockResV, c_BS)); }
          else                                      { zeroEntireBlock(SamplesOrg); }

          const int32 CoeffOffset = BlockIdx << c_L2BA;
          const bool  FirstInSlc  = BlockIdx == 0 || (m_RestartInterval > 0 && MCU_Idx % m_RestartInterval == 0);
          const int32 LastDC      = FirstInSlc ? 0 : CoeffsScanV[CmpIdx][CoeffOffset - c_BA];
          if (m_BeamSearch)
            xOptQuantBLKBeam(OptCoeffsScanV[CmpIdx] + CoeffOffset, CoeffsScanV[CmpIdx] + CoeffOffset, SamplesOrg, eCmp(CmpIdx), LastDC);
          else if (m_GreedyMultiPass)
            xOptQuantBLKGreedyMP(OptCoeffsScanV[CmpIdx] + CoeffOffset, CoeffsScanV[CmpIdx] + CoeffOffset, SamplesOrg, eCmp(CmpIdx), LastDC);
          else
            xOptQuantBLK(OptCoeffsScanV[CmpIdx] + CoeffOffset, CoeffsScanV[CmpIdx] + CoeffOffset, SamplesOrg, eCmp(CmpIdx), LastDC);
          BlockIdx++;
        }
      }
    }
  }
}
void xAdvancedEncoder::xOptQuantMCUdct(int16* OptCoeffsScanV[], const int16* CoeffsScanV[], const int16* CoeffsTransOrgV[], int32 MCU_Idx)
{
  for (int32 CmpIdx = 0; CmpIdx < m_NumOfComponents; CmpIdx++)
  {
    if (m_UseRDOQ && ((CmpIdx == (int32)eCmp::LM && m_OptQuantLuma) || (CmpIdx != (int32)eCmp::LM && m_OptQuantChroma)))
    {
      int32 BlockIdx = MCU_Idx * m_SampFactorVer[CmpIdx] * m_SampFactorHor[CmpIdx];
      for (int32 V = 0; V < m_SampFactorVer[CmpIdx]; V++)
      {
        for (int32 H = 0; H < m_SampFactorHor[CmpIdx]; H++)
        {
          const int32 CoeffOffset = BlockIdx << c_L2BA;
          const bool  FirstInSlc = BlockIdx == 0 || (m_RestartInterval > 0 && MCU_Idx % m_RestartInterval == 0);
          const int32 LastDC = FirstInSlc ? 0 : CoeffsScanV[CmpIdx][CoeffOffset - c_BA];
          xOptQuantBLKdct(OptCoeffsScanV[CmpIdx] + CoeffOffset, CoeffsScanV[CmpIdx] + CoeffOffset, CoeffsTransOrgV[CmpIdx] + CoeffOffset, eCmp(CmpIdx), LastDC);
          BlockIdx++;
        }
      }
    }
  }
}
void xAdvancedEncoder::xOptQuantBLK(int16* OptCoeffScan, const int16* CoeffsScan, const uint16* SamplesOrg, eCmp CmpId, int32 LastDC)
{
  const int32 QuantTabId  = m_SOF0.getQuantTableId(CmpId);
  const int32 HuffTabIdDC = m_SOS.getHuffTableIdDC(CmpId);
  const int32 HuffTabIdAC = m_SOS.getHuffTableIdAC(CmpId);  
  const flt64 Lambda      = m_Lambda[(int32)CmpId];

  int32 LastNonZero = xEntropyCommon::findLastNonZero(CoeffsScan);
  if(LastNonZero == 0) { memcpy(OptCoeffScan, CoeffsScan, c_BA * sizeof(int16)); return; } //only DC - nothing to do here

  const int32 OrgBitsDC = m_EntropyEst.EstimateBlockDC(CoeffsScan, LastDC, HuffTabIdDC);
  const int32 OrgBitsAC = m_EntropyEst.EstimateBlockAC(CoeffsScan,         HuffTabIdAC);

  int32  BestBits = OrgBitsDC + OrgBitsAC;
  uint64 BestDist = xCalcDistBLK(CoeffsScan, SamplesOrg, QuantTabId);
  flt64  BestCost = (flt64)BestDist + Lambda * (flt64)BestBits;

  int16 TmpCoeffsScan[c_BA];
  memcpy(TmpCoeffsScan, CoeffsScan, c_BA * sizeof(int16));

  for (int32 PassIdx = 0; PassIdx < m_NumOptPassesBlock; PassIdx++)
  {    
    for (int32 i = LastNonZero; i >= 1; i--)
    {
      const int16 OrgCoeff  = TmpCoeffsScan[i];
      if (!m_ProcessZeroCoeffs && OrgCoeff == 0) { continue; }
      
      int16 BestCoeff = TmpCoeffsScan[i];
      //try zero
      if(OrgCoeff != 0)
      {
        TmpCoeffsScan[i] = 0;
        int32  CurrBits = OrgBitsDC + m_EntropyEst.EstimateBlockAC(TmpCoeffsScan, HuffTabIdAC);
        uint64 CurrDist = xCalcDistBLK(TmpCoeffsScan, SamplesOrg, QuantTabId);
        flt64  CurrCost = (flt64)CurrDist + Lambda * (flt64)CurrBits;
        if (CurrCost < BestCost)
        {
          BestBits  = CurrBits;
          BestCost  = CurrCost;
          BestCoeff = 0;
        }
      }

      //try +1
      if(OrgCoeff != -1)
      {
        TmpCoeffsScan[i] = OrgCoeff + 1;
        int32  CurrBits = OrgBitsDC + m_EntropyEst.EstimateBlockAC(TmpCoeffsScan, HuffTabIdAC);
        uint64 CurrDist = xCalcDistBLK(TmpCoeffsScan, SamplesOrg, QuantTabId);
        flt64  CurrCost = (flt64)CurrDist + Lambda * (flt64)CurrBits;
        if (CurrCost < BestCost)
        {
          BestBits = CurrBits;
          BestCost  = CurrCost;
          BestCoeff = OrgCoeff + 1;
        }
      }

      //try -1  
      if(OrgCoeff != 1)
      {
        TmpCoeffsScan[i] = OrgCoeff - 1;
        int32  CurrBits = OrgBitsDC + m_EntropyEst.EstimateBlockAC(TmpCoeffsScan, HuffTabIdAC);
        uint64 CurrDist = xCalcDistBLK(TmpCoeffsScan, SamplesOrg, QuantTabId);
        flt64  CurrCost = (flt64)CurrDist + Lambda * (flt64)CurrBits;
        if (CurrCost < BestCost)
        {
          BestBits = CurrBits;
          BestCost  = CurrCost;
          BestCoeff = OrgCoeff - 1;
        }
      }

      TmpCoeffsScan[i] = BestCoeff;
    }
  }

  //int32 TestBits = m_EntropyEst.EstimateBlock(TmpCoeffsScan, CmpId, HuffTabIdDC, HuffTabIdAC);
  memcpy(OptCoeffScan, TmpCoeffsScan, c_BA * sizeof(int16));
}
void xAdvancedEncoder::xOptQuantBLKGreedyMP(int16* OptCoeffScan, const int16* CoeffsScan, const uint16* SamplesOrg, eCmp CmpId, int32 LastDC)
{
  const int32 QuantTabId = m_SOF0.getQuantTableId(CmpId);
  const int32 HuffTabIdDC = m_SOS.getHuffTableIdDC(CmpId);
  const int32 HuffTabIdAC = m_SOS.getHuffTableIdAC(CmpId);
  const flt64 Lambda = m_Lambda[(int32)CmpId];

  int32 LastNonZero = xEntropyCommon::findLastNonZero(CoeffsScan);
  if (LastNonZero == 0) { memcpy(OptCoeffScan, CoeffsScan, c_BA * sizeof(int16)); return; } //only DC - nothing to do here

  const int32 OrgBitsDC = m_EntropyEst.EstimateBlockDC(CoeffsScan, LastDC, HuffTabIdDC);
  const int32 OrgBitsAC = m_EntropyEst.EstimateBlockAC(CoeffsScan, HuffTabIdAC);

  int32  BestBits = OrgBitsDC + OrgBitsAC;
  uint64 BestDist = xCalcDistBLK(CoeffsScan, SamplesOrg, QuantTabId);
  flt64  BestCost = (flt64)BestDist + Lambda * (flt64)BestBits;

  int16 TmpCoeffsScan[c_BA];
  memcpy(TmpCoeffsScan, CoeffsScan, c_BA * sizeof(int16));

  for (int32 PassIdx = 0; PassIdx < m_NumOptPassesBlock; PassIdx++)
  {
    int16 RoundBestIdx = -1;
    int16 RoundBestCoeff = 0;
    flt64 RoundBestCost = BestCost;

    for (int32 i = LastNonZero; i >= 1; i--)
    {
      const int16 OrgCoeff = TmpCoeffsScan[i];
      if (!m_ProcessZeroCoeffs && OrgCoeff == 0) { continue; }

      const int16 backup = TmpCoeffsScan[i];

      //try zero
      if (OrgCoeff != 0)
      {
        TmpCoeffsScan[i] = 0;
        int32  CurrBits = OrgBitsDC + m_EntropyEst.EstimateBlockAC(TmpCoeffsScan, HuffTabIdAC);
        uint64 CurrDist = xCalcDistBLK(TmpCoeffsScan, SamplesOrg, QuantTabId);
        flt64  CurrCost = (flt64)CurrDist + Lambda * (flt64)CurrBits;

        if (CurrCost < RoundBestCost)
        {
          RoundBestCost = CurrCost;
          RoundBestIdx = i;
          RoundBestCoeff = 0;
        }
      }

      //try +1
      if (OrgCoeff != -1)
      {
        TmpCoeffsScan[i] = OrgCoeff + 1;
        int32  CurrBits = OrgBitsDC + m_EntropyEst.EstimateBlockAC(TmpCoeffsScan, HuffTabIdAC);
        uint64 CurrDist = xCalcDistBLK(TmpCoeffsScan, SamplesOrg, QuantTabId);
        flt64  CurrCost = (flt64)CurrDist + Lambda * (flt64)CurrBits;

        if (CurrCost < RoundBestCost)
        {
          RoundBestCost = CurrCost;
          RoundBestIdx = i;
          RoundBestCoeff = OrgCoeff + 1;
        }
      }

      //try -1  
      if (OrgCoeff != 1)
      {
        TmpCoeffsScan[i] = OrgCoeff - 1;
        int32  CurrBits = OrgBitsDC + m_EntropyEst.EstimateBlockAC(TmpCoeffsScan, HuffTabIdAC);
        uint64 CurrDist = xCalcDistBLK(TmpCoeffsScan, SamplesOrg, QuantTabId);
        flt64  CurrCost = (flt64)CurrDist + Lambda * (flt64)CurrBits;

        if (CurrCost < RoundBestCost)
        {
          RoundBestCost = CurrCost;
          RoundBestIdx = i;
          RoundBestCoeff = OrgCoeff - 1;
        }
      }

      TmpCoeffsScan[i] = backup;
    }

    if (RoundBestIdx < 0)
    {
      break;
    }

    TmpCoeffsScan[RoundBestIdx] = RoundBestCoeff;
    BestCost = RoundBestCost;

  }

  memcpy(OptCoeffScan, TmpCoeffsScan, c_BA * sizeof(int16));
}
void xAdvancedEncoder::xOptQuantBLKBeam(int16* OptCoeffScan, const int16* CoeffsScan, const uint16* SamplesOrg, eCmp CmpId, int32 LastDC)
{
  const int32 QuantTabId = m_SOF0.getQuantTableId(CmpId);
  const int32 HuffTabIdDC = m_SOS.getHuffTableIdDC(CmpId);
  const int32 HuffTabIdAC = m_SOS.getHuffTableIdAC(CmpId);
  const flt64 Lambda = m_Lambda[(int32)CmpId];

  int32 LastNonZero = xEntropyCommon::findLastNonZero(CoeffsScan);
  if (LastNonZero == 0) { memcpy(OptCoeffScan, CoeffsScan, c_BA * sizeof(int16)); return; }

  const int32 OrgBitsDC = m_EntropyEst.EstimateBlockDC(CoeffsScan, LastDC, HuffTabIdDC);

  const int32 NumStates = m_BeamWidth;
  const int32 NumSteps = m_BeamSteps;

  struct xStateRDOQ { int16 coeffs[c_BA]; flt64 cost; };

  std::vector<xStateRDOQ> Beam;
  Beam.reserve(NumStates);

  xStateRDOQ s0;
  memcpy(s0.coeffs, CoeffsScan, c_BA * sizeof(int16));

  const int32 BitsAC0 = m_EntropyEst.EstimateBlockAC(s0.coeffs, HuffTabIdAC);
  const int32 Bits0 = OrgBitsDC + BitsAC0;
  const uint64 Dist0 = xCalcDistBLK(s0.coeffs, SamplesOrg, QuantTabId);
  s0.cost = (flt64)Dist0 + Lambda * (flt64)Bits0;
  Beam.push_back(s0);

  struct xLocalCandidate { flt64 cost; int32 idx; int16 val; };

  for (int32 step = 0; step < NumSteps; step++)
  {
    std::vector<xStateRDOQ> Candidates;
    Candidates.reserve((int32)Beam.size() * NumStates);

    for (const xStateRDOQ& currState : Beam)
    {
      xLocalCandidate LocalCandidates[c_MAX_BEAM_WIDTH];
      int32 numLocal = 0;

      int16 tmp[c_BA];
      memcpy(tmp, currState.coeffs, c_BA * sizeof(int16));

      auto EvaluateCoeffChange = [&](int32 idx, int16 val)
        {
          const int16 backup = tmp[idx];
          tmp[idx] = val;

          const int32 BitsAC = m_EntropyEst.EstimateBlockAC(tmp, HuffTabIdAC);
          const int32 Bits = OrgBitsDC + BitsAC;
          const uint64 Dist = xCalcDistBLK(tmp, SamplesOrg, QuantTabId);
          const flt64 c = (flt64)Dist + Lambda * (flt64)Bits;

          tmp[idx] = backup;

          if (numLocal < NumStates)
          {
            LocalCandidates[numLocal++] = { c, idx, val };
          }
          else
          {
            int32 worst = 0;
            for (int32 t = 1; t < NumStates; t++) {
              if (LocalCandidates[t].cost > LocalCandidates[worst].cost) worst = t;
            }
            if (c < LocalCandidates[worst].cost) LocalCandidates[worst] = { c, idx, val };
          }
        };

      for (int32 i = LastNonZero; i >= 1; i--)
      {
        const int16 v = currState.coeffs[i];
        if (!m_ProcessZeroCoeffs && v == 0) continue;

        // 0
        if (v != 0)  EvaluateCoeffChange(i, 0);
        // +1
        if (v != -1) EvaluateCoeffChange(i, (int16)(v + 1));
        // -1
        if (v != 1)  EvaluateCoeffChange(i, (int16)(v - 1));
      }

      for (int32 t = 0; t < numLocal; t++)
      {
        xStateRDOQ nxt = currState;
        const int32 idx = LocalCandidates[t].idx;
        nxt.coeffs[idx] = LocalCandidates[t].val;
        nxt.cost = LocalCandidates[t].cost;
        Candidates.push_back(nxt);
      }
    }

    if (Candidates.empty()) break;

    const int32 keep = std::min<int32>(NumStates, (int32)Candidates.size());
    std::nth_element(
      Candidates.begin(),
      Candidates.begin() + (keep - 1),
      Candidates.end(),
      [](const xStateRDOQ& a, const xStateRDOQ& b) { return a.cost < b.cost; });

    Candidates.resize(keep);
    std::sort(Candidates.begin(), Candidates.end(),
      [](const xStateRDOQ& a, const xStateRDOQ& b) { return a.cost < b.cost; });

    if (Candidates[0].cost >= Beam[0].cost) break;

    Beam.swap(Candidates);
  }

  memcpy(OptCoeffScan, Beam[0].coeffs, c_BA * sizeof(int16));
}
void xAdvancedEncoder::xOptQuantBLKdct(int16* OptCoeffScan, const int16* CoeffsScan, const int16* CoeffsTransOrg, eCmp CmpId, int32 LastDC)
{
  const int32 QuantTabId = m_SOF0.getQuantTableId(CmpId);
  const int32 HuffTabIdDC = m_SOS.getHuffTableIdDC(CmpId);
  const int32 HuffTabIdAC = m_SOS.getHuffTableIdAC(CmpId);
  const flt64 Lambda = m_Lambda[(int32)CmpId];

  int32 LastNonZero = xEntropyCommon::findLastNonZero(CoeffsScan);
  if (LastNonZero == 0) { memcpy(OptCoeffScan, CoeffsScan, c_BA * sizeof(int16)); return; } //only DC - nothing to do here

  const int32 OrgBitsDC = m_EntropyEst.EstimateBlockDC(CoeffsScan, LastDC, HuffTabIdDC);
  const int32 OrgBitsAC = m_EntropyEst.EstimateBlockAC(CoeffsScan, HuffTabIdAC);

  int32  BestBits = OrgBitsDC + OrgBitsAC;
  uint64 BestDist = xCalcDistBLKdct(CoeffsScan, CoeffsTransOrg, QuantTabId);
  flt64  BestCost = (flt64)BestDist + Lambda * (flt64)BestBits;

  int16 TmpCoeffsScan[c_BA];
  memcpy(TmpCoeffsScan, CoeffsScan, c_BA * sizeof(int16));

  for (int32 PassIdx = 0; PassIdx < m_NumOptPassesBlock; PassIdx++)
  {
    for (int32 i = LastNonZero; i >= 1; i--)
    {
      const int16 OrgCoeff = TmpCoeffsScan[i];
      if (!m_ProcessZeroCoeffs && OrgCoeff == 0) { continue; }

      int16 BestCoeff = TmpCoeffsScan[i];
      //try zero
      if (OrgCoeff != 0)
      {
        TmpCoeffsScan[i] = 0;
        int32  CurrBits = OrgBitsDC + m_EntropyEst.EstimateBlockAC(TmpCoeffsScan, HuffTabIdAC);
        uint64 CurrDist = xCalcDistBLKdct(TmpCoeffsScan, CoeffsTransOrg, QuantTabId);
        flt64  CurrCost = (flt64)CurrDist + Lambda * (flt64)CurrBits;
        if (CurrCost < BestCost)
        {
          BestBits = CurrBits;
          BestCost = CurrCost;
          BestCoeff = 0;
        }
      }

      //try +1
      if (OrgCoeff != -1)
      {
        TmpCoeffsScan[i] = OrgCoeff + 1;
        int32  CurrBits = OrgBitsDC + m_EntropyEst.EstimateBlockAC(TmpCoeffsScan, HuffTabIdAC);
        uint64 CurrDist = xCalcDistBLKdct(TmpCoeffsScan, CoeffsTransOrg, QuantTabId);
        flt64  CurrCost = (flt64)CurrDist + Lambda * (flt64)CurrBits;
        if (CurrCost < BestCost)
        {
          BestBits = CurrBits;
          BestCost = CurrCost;
          BestCoeff = OrgCoeff + 1;
        }
      }

      //try -1  
      if (OrgCoeff != 1)
      {
        TmpCoeffsScan[i] = OrgCoeff - 1;
        int32  CurrBits = OrgBitsDC + m_EntropyEst.EstimateBlockAC(TmpCoeffsScan, HuffTabIdAC);
        uint64 CurrDist = xCalcDistBLKdct(TmpCoeffsScan, CoeffsTransOrg, QuantTabId);
        flt64  CurrCost = (flt64)CurrDist + Lambda * (flt64)CurrBits;
        if (CurrCost < BestCost)
        {
          BestBits = CurrBits;
          BestCost = CurrCost;
          BestCoeff = OrgCoeff - 1;
        }
      }

      TmpCoeffsScan[i] = BestCoeff;
    }
  }

  //int32 TestBits = m_EntropyEst.EstimateBlock(TmpCoeffsScan, CmpId, HuffTabIdDC, HuffTabIdAC);
  memcpy(OptCoeffScan, TmpCoeffsScan, c_BA * sizeof(int16));
}
uint64 xAdvancedEncoder::xCalcDistBLK(const int16* ScanCoeffs, const uint16* SamplesOrg, int32 QuantTabId)
{
  int16  TmpQuantCoeffs[c_BA];
  int16  TmpTransCoeffs[c_BA];
  uint16 TmpSamples    [c_BA];

  xScan::InvScan(TmpQuantCoeffs, ScanCoeffs);
  m_QuantMain.InvScale(TmpTransCoeffs, TmpQuantCoeffs, QuantTabId);
  TmpTransCoeffs[0] += xTransformConstants::c_InvDcCorr; //DC correction - JPEG requires 128 to be subtracted from every input sample - could be done be DC -= 
  xTransform::InvTransformDCT_8x8(TmpSamples, TmpTransCoeffs);
  uint64 SSD = xDistortion::CalcSSD(SamplesOrg, TmpSamples, c_BS, c_BS, c_BS, c_BS);
  return SSD;
}

uint64 xAdvancedEncoder::xCalcDistBLKdct(const int16* ScanCoeffs, const int16* CoeffsTransOrg, int32 QuantTabId)
{
  int16  TmpQuantCoeffs[c_BA];
  int16  TmpTransCoeffs[c_BA];
  uint16 TmpSamples[c_BA];

  xScan::InvScan(TmpQuantCoeffs, ScanCoeffs);
  m_QuantMain.InvScale(TmpTransCoeffs, TmpQuantCoeffs, QuantTabId);
  uint64 SSD = xDistortion::CalcSSDdct(CoeffsTransOrg, TmpTransCoeffs, c_BS, c_BS, c_BS, c_BS);
  return SSD;
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void xAdvancedEncoder::xOptHuffPic(const int16* CoeffsScanV[])
{
  //calc symbol symbols
  xCntPic(CoeffsScanV);

  //build huffman tables
  std::vector<xJFIF::xHuffTable> NewHT = m_HTs;

  for(xJFIF::xHuffTable& HuffTable : NewHT)
  {
    xJFIF::xHuffTable::eHuffClass HuffTableClass = HuffTable.getClass();
    int32                         HuffTableId    = HuffTable.getIdx  ();
    switch(HuffTableClass)
    {
    case xJFIF::xHuffTable::eHuffClass::DC:
    {
      const uint32* SymbolCount = m_EntropyCnts[0].getSymbolCountDC(HuffTableId);
      uint8 LengthTable[xJPEG_Constants::c_MaxNumCodeSymbolsDC+1];
      xHuffmanTabBuilder::buildLengthTable(LengthTable, SymbolCount, xJPEG_Constants::c_MaxNumCodeSymbolsDC);
      HuffTable.InitCustom((uint8)HuffTableId, HuffTableClass, LengthTable);
      break;
    }
    case xJFIF::xHuffTable::eHuffClass::AC:
    {
      const uint32* SymbolCount = m_EntropyCnts[0].getSymbolCountAC(HuffTableId);
      uint8 LengthTable[xJPEG_Constants::c_MaxNumCodeSymbolsAC+1];
      xHuffmanTabBuilder::buildLengthTable(LengthTable, SymbolCount, xJPEG_Constants::c_MaxNumCodeSymbolsAC);
      HuffTable.InitCustom((uint8)HuffTableId, HuffTableClass, LengthTable);
      break;
    }
    default: break;
    }
  }

  //print tables
  if(m_VerboseLevel >= 8)
  {
    std::string Dump = fmt::format("HuffmanTablesPrev\n");
    for(int32 i = 0; i < (int32)m_HTs.size(); i++) { Dump += fmt::format("  Table_{}\n", i) + m_HTs[i].Format("    "); }
    Dump += fmt::format("HuffmanTablesNext\n");
    for(int32 i = 0; i < (int32)NewHT.size(); i++) { Dump += fmt::format("  Table_{}\n", i) + NewHT[i].Format("    "); }
    fmt::print("{}\n", Dump);
  }

  if(m_VerboseLevel >= 6)
  {
    std::string Dump = fmt::format("HuffmanTablesPrev\n");
    for(xJFIF::xHuffTable& HuffTable : m_HTs)
    {
      const uint32* SymbolCount = m_EntropyCnts[0].getSymbolCount(HuffTable.getClass(), HuffTable.getIdx());
      flt64 AvgCodeLength = xHuffmanTabBuilder::calcAvgCodeLength(HuffTable, SymbolCount);
      Dump += fmt::format("  Table Idx={} Class={} AvgCodeLength={:7.5f}\n", HuffTable.getIdx(), xJFIF::xHuffTable::xHuffClass2Str(HuffTable.getClass()), AvgCodeLength);
    }
    Dump += fmt::format("HuffmanTablesNext\n");
    for(xJFIF::xHuffTable& HuffTable : NewHT)
    {
      const uint32* SymbolCount = m_EntropyCnts[0].getSymbolCount(HuffTable.getClass(), HuffTable.getIdx());
      flt64 AvgCodeLength = xHuffmanTabBuilder::calcAvgCodeLength(HuffTable, SymbolCount);
      Dump += fmt::format("  Table Idx={} Class={} AvgCodeLength={:7.5f}\n", HuffTable.getIdx(), xJFIF::xHuffTable::xHuffClass2Str(HuffTable.getClass()), AvgCodeLength);
    }
    fmt::print("{}\n", Dump);
  }

  m_HTs = NewHT;
  
  if(m_NumOptPassesPic > 1)
  {
    m_EntropyEst.Init(m_HTs);
  }
  for(int32 i = 0; i < (int32)m_EntropyEncs.size(); i++) { m_EntropyEncs[i].Init(m_HTs); }
}
void xAdvancedEncoder::xCntPic(const int16* CoeffsScanV[])
{
  const int32 NumCounters = (int32)m_EntropyCnts.size();
  for(int32 i = 0; i < NumCounters; i++) { m_EntropyCnts[i].Init(m_HTs); }

  //count symbols
  for(int32 CmpIdx = 0; CmpIdx < m_NumOfComponents; CmpIdx++)
  {
    const int32  HuffTabIdDC       = m_SOS.getHuffTableIdDC(eCmp(CmpIdx));
    const int32  HuffTabIdAC       = m_SOS.getHuffTableIdAC(eCmp(CmpIdx));
    const int32  BlocksPerSlice    = m_RestartInterval * m_NumBlocksInMCU[CmpIdx];
    const int32  NumBlocksInWidth  = m_NumBlocksInWidth [CmpIdx];
    const int32  NumBlocksInHeight = m_NumBlocksInHeight[CmpIdx];
    const int16* CoeffsScan         = CoeffsScanV[CmpIdx];

    for(int32 BlockRowIdx = 0; BlockRowIdx < NumBlocksInHeight; BlockRowIdx++) //loop over block rows
    {
      const int32 BlockIdxBeg = BlockRowIdx * NumBlocksInWidth;
      const int32 BlockIdxEnd = BlockIdxBeg + NumBlocksInWidth;
      m_ThPI.storeTask([this, CoeffsScan, HuffTabIdDC, HuffTabIdAC, BlockIdxBeg, BlockIdxEnd, BlocksPerSlice](int32 ThIdx) { xCntRng(CoeffsScan, ThIdx, HuffTabIdDC, HuffTabIdAC, BlockIdxBeg, BlockIdxEnd, BlocksPerSlice); });
    }
  }
  m_ThPI.executeStoredTasks();

  //agregate counters
  for(int32 i = 1; i < NumCounters; i++)
  {
    m_EntropyCnts[0].AddCounters(m_EntropyCnts[i]);
  }
}
void xAdvancedEncoder::xCntRng(const int16* CoeffsScan, int32 CounterIdx, int32 HuffTabIdDC, int32 HuffTabIdAC, int32 BlockIdxBeg, int32 BlockIdxEnd, int32 BlocksPerSlice)
{
  for(int32 BlockIdx = BlockIdxBeg; BlockIdx < BlockIdxEnd; BlockIdx++)
  {
    const int32 CoeffOffset = BlockIdx << c_L2BA;
    const bool  FirstInSlc  = BlockIdx == 0 || (m_RestartInterval > 0 && BlockIdx % BlocksPerSlice == 0);
    const int32 LastDC      = FirstInSlc ? 0 : CoeffsScan[CoeffOffset - c_BA];
    m_EntropyCnts[CounterIdx].CountBlock(CoeffsScan + CoeffOffset, LastDC, HuffTabIdDC, HuffTabIdAC);
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void xAdvancedEncoder::xHuffEncPic(const int16* CoeffsScanV[])
{
  if(m_RestartInterval == 0) //no division - however, try process picture in MCU rows
  {
    if(!m_ThPI.isActive()) //no active thread pool - encode entire picture at once
    {
      xHuffEncSlc(CoeffsScanV, 0, 0, m_NumMCUsInArea - 1);
    }
    else //encode picture in MCU 
    {
      for(int32 MCU_RowIdx = 0; MCU_RowIdx < m_NumMCUsInHeight; MCU_RowIdx++) //loop over MCUs rows
      {
        const int32 MCU_IdxFirst = MCU_RowIdx   * m_NumMCUsInWidth;
        const int32 MCU_IdxLast  = MCU_IdxFirst + m_NumMCUsInWidth - 1;        
        m_ThPI.storeTask([this, &CoeffsScanV, MCU_RowIdx, MCU_IdxFirst, MCU_IdxLast](int32 /*ThIdx*/) { xHuffEncChk(CoeffsScanV, MCU_RowIdx, MCU_IdxFirst, MCU_IdxLast); });
      }
    }
  }
  else //divide picture into independent slices
  {
    for(int32 SliceIdx = 0; SliceIdx < m_NumOfSlices; SliceIdx++) //loop over slices
    {
      const int32 MCU_IdxFirst = SliceIdx * m_RestartInterval;
      const int32 MCU_IdxLast  = xMin(m_NumMCUsInArea, MCU_IdxFirst + m_RestartInterval) - 1;
      m_ThPI.storeTask([this, &CoeffsScanV, SliceIdx, MCU_IdxFirst, MCU_IdxLast](int32 /*ThIdx*/) { xHuffEncSlc(CoeffsScanV, SliceIdx, MCU_IdxFirst, MCU_IdxLast); });
    }
  }
  m_ThPI.executeStoredTasks();
}
void xAdvancedEncoder::xHuffEncSlc(const int16* CoeffsScanV[], int32 SliceIdx, int32 MCU_IdxFirst, int32 MCU_IdxLast)
{
  m_EncBuffers [SliceIdx].reset();
  m_EntropyEncs[SliceIdx].StartSlice(&m_EncBuffers[SliceIdx]);

  //loop over MCUs
  for(int32 MCU_Idx = MCU_IdxFirst; MCU_Idx <= MCU_IdxLast; MCU_Idx++)
  {
    xHuffEncMCU(CoeffsScanV, SliceIdx, MCU_Idx);
  }

  m_EntropyEncs[SliceIdx].FinishSlice();

  m_SliceBuffers[SliceIdx].reset();
  xJFIF::AddStuffing(&m_SliceBuffers[SliceIdx], &m_EncBuffers[SliceIdx]);
}
void xAdvancedEncoder::xHuffEncChk(const int16* CoeffsScanV[], int32 ChunkIdx, int32 MCU_IdxFirst, int32 MCU_IdxLast)
{
  xEntropyCommon::tLDCs LastDCs = { 0,0,0,0 };
  if(MCU_IdxFirst != 0)
  {
    for(int32 CmpIdx = 0; CmpIdx < m_NumOfComponents; CmpIdx++)
    {
      const int32 BlockIdx    = MCU_IdxFirst * m_SampFactorVer[CmpIdx] * m_SampFactorHor[CmpIdx];
      const int32 CoeffOffset = BlockIdx << c_L2BA;
      const int16 LastDC      = CoeffsScanV[CmpIdx][CoeffOffset - c_BA];
      LastDCs[CmpIdx] = LastDC;
    }
  }

  m_EncBuffers [ChunkIdx].reset();
  m_EntropyEncs[ChunkIdx].StartChunk(&m_EncBuffers[ChunkIdx], LastDCs);

  //loop over MCUs
  for(int32 MCU_Idx = MCU_IdxFirst; MCU_Idx <= MCU_IdxLast; MCU_Idx++)
  {
    xHuffEncMCU(CoeffsScanV, ChunkIdx, MCU_Idx);
  }

  m_NumAlignmentBits[ChunkIdx] = m_EntropyEncs[ChunkIdx].FinishChunk();
}
void xAdvancedEncoder::xHuffEncMCU(const int16* CoeffsScanV[], int32 SliceIdx, int32 MCU_Idx)
{
  //estimate blocks
  for(int32 CmpIdx = 0; CmpIdx < m_NumOfComponents; CmpIdx++)
  {
    int32 HuffTabIdDC = m_SOS.getHuffTableIdDC(eCmp(CmpIdx));
    int32 HuffTabIdAC = m_SOS.getHuffTableIdAC(eCmp(CmpIdx));

    int32 BlockIdx = MCU_Idx * m_SampFactorVer[CmpIdx] * m_SampFactorHor[CmpIdx];
    for(int32 V = 0; V < m_SampFactorVer[CmpIdx]; V++)
    {
      for(int32 H = 0; H < m_SampFactorHor[CmpIdx]; H++)
      {
        const int32 CoeffScanOffset = BlockIdx << c_L2BA;        
        m_EntropyEncs[SliceIdx].EncodeBlock(CoeffsScanV[CmpIdx] + CoeffScanOffset, eCmp(CmpIdx), HuffTabIdDC, HuffTabIdAC);
        BlockIdx++;
      }
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void xAdvancedEncoder::xWritePic(xByteBuffer* OutputBuffer)
{
  if(m_RestartInterval == 0)
  {
    if(!m_ThPI.isActive())
    {
      OutputBuffer->transfer(&m_SliceBuffers[0]);
    }
    else
    {
      m_CombineBuffer.reset();
      xBitstreamWriter Bitstream;
      Bitstream.bindByteBuffer(&m_CombineBuffer);
      Bitstream.init();
      for(int32 MCU_RowIdx = 0; MCU_RowIdx < m_NumMCUsInHeight; MCU_RowIdx++)
      {
        Bitstream.writeBuffer(&m_EncBuffers[MCU_RowIdx], m_NumAlignmentBits[MCU_RowIdx]);
      }
      Bitstream.writeAlign(1);
      Bitstream.uninit();
      Bitstream.unbindByteBuffer();
      xJFIF::AddStuffing(OutputBuffer, &m_CombineBuffer);
    }
  }
  else
  {
    for(int32 SliceIdx = 0; SliceIdx < m_NumOfSlices; SliceIdx++)
    {
      const int32 MCU_IdxFirst = SliceIdx * m_RestartInterval;
      const int32 MCU_IdxLast = xMin(m_NumMCUsInArea, MCU_IdxFirst + m_RestartInterval) - 1;
      //copy to output and add stuffing
      OutputBuffer->transfer(&m_SliceBuffers[SliceIdx]);
      if(m_RestartInterval >= 0)
      {
        if(MCU_IdxLast != m_NumMCUsInArea - 1)
        {
          xJFIF::WriteRST(OutputBuffer, (uint8)((uint32)SliceIdx & (uint32)0x07));
        }
      }
    }
  }
}

//=====================================================================================================================================================================================

} //end of namespace PMBB::JPEG