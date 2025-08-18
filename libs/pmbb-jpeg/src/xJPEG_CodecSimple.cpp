/*
    SPDX-FileCopyrightText: 2020-2024 Jakub Stankowski <jakub.stankowski@put.poznan.pl>
    SPDX-License-Identifier: BSD-3-Clause
*/
#include "xJPEG_CodecSimple.h"
#include "xJPEG_Transform.h"
#include "xJPEG_TransformConstants.h"
#include "xJPEG_Scan.h"
#include "xColorSpace.h"
#include "xPixelOps.h"
#include "xString.h"
#include "xMemory.h"

namespace PMBB_NAMESPACE::JPEG {

//=====================================================================================================================================================================================
// xCodecSimple
//=====================================================================================================================================================================================

void xCodecSimple::xCreate(xThreadPool* ThreadPool)
{
  m_EntropyBuffer.create(256);
  if(ThreadPool != nullptr) { m_ThPI.init(ThreadPool, 1024, 1024); }
}
void xCodecSimple::xDestroy()
{
  m_ThPI.uninit();
  m_EntropyBuffer.destroy();
}
std::string xCodecSimple::formatAndResetStats(const std::string Prefix, flt64 TicksPerMicroSec)
{
  if(m_TotalPictureIters == 0) { m_TotalPictureIters = 1; }
  if(m_TotalSliceIters   == 0) { m_TotalSliceIters   = 1; }

  bool Parallel   = m_RestartInterval != 0 && m_ThPI.isActive();

  flt64 InvDenom  = TicksPerMicroSec == 0.0 ? (flt64)1.0 / ((flt64)m_TotalPictureIters) : (flt64)1.0 / ((flt64)m_TotalPictureIters * TicksPerMicroSec);
  flt64 InvDenomN = Parallel ? InvDenom / m_ThPI.getNumThreads() : InvDenom;

  std::string TimeStats; TimeStats.reserve(xMemory::c_MemSizePageBase);
  TimeStats += Prefix + fmt::format("Processing time {}\n", TicksPerMicroSec == 0.0 ? "[ticks]" : "[us]");

  uint64 TotalTicks____Slice = std::accumulate(m_Ticks____Slice.cbegin(), m_Ticks____Slice.cend(), (uint64)0);
  uint64 TotalTicks_____MCUs = std::accumulate(m_Ticks_____MCUs.cbegin(), m_Ticks_____MCUs.cend(), (uint64)0);
  uint64 TotalTicksTransform = std::accumulate(m_TicksTransform.cbegin(), m_TicksTransform.cend(), (uint64)0);
  uint64 TotalTicks____Quant = std::accumulate(m_Ticks____Quant.cbegin(), m_Ticks____Quant.cend(), (uint64)0);
  uint64 TotalTicks_____Scan = std::accumulate(m_Ticks_____Scan.cbegin(), m_Ticks_____Scan.cend(), (uint64)0);
  uint64 TotalTicks__Entropy = std::accumulate(m_Ticks__Entropy.cbegin(), m_Ticks__Entropy.cend(), (uint64)0);
  uint64 TotalTicks_Stuffing = std::accumulate(m_Ticks_Stuffing.cbegin(), m_Ticks_Stuffing.cend(), (uint64)0);

  TimeStats += Prefix + "  " + fmt::format("PicIters   = {}\n"      , m_TotalPictureIters);
  TimeStats += Prefix + "  " + fmt::format("Pic        = {:.2f}\n"  , m_Ticks__Picture    * InvDenom );
  TimeStats += Prefix + "  " + fmt::format("Slice      = {:.2f}{}\n", TotalTicks____Slice * InvDenomN, Parallel ? " P" : "");
  TimeStats += Prefix + "  " + fmt::format("MCU        = {:.2f}{}\n", TotalTicks_____MCUs * InvDenomN, Parallel ? " P" : "");
  TimeStats += Prefix + "  " + fmt::format("Transform  = {:.2f}{}\n", TotalTicksTransform * InvDenomN, Parallel ? " P" : "");
  TimeStats += Prefix + "  " + fmt::format("Quant      = {:.2f}{}\n", TotalTicks____Quant * InvDenomN, Parallel ? " P" : "");
  TimeStats += Prefix + "  " + fmt::format("Scan       = {:.2f}{}\n", TotalTicks_____Scan * InvDenomN, Parallel ? " P" : "");
  TimeStats += Prefix + "  " + fmt::format("Entropy    = {:.2f}{}\n", TotalTicks__Entropy * InvDenomN, Parallel ? " P" : "");
  TimeStats += Prefix + "  " + fmt::format("Stuffing   = {:.2f}\n"  , TotalTicks_Stuffing * InvDenom );

  m_Ticks__Picture = 0;
  std::fill(m_Ticks____Slice.begin(), m_Ticks____Slice.end(), 0);
  std::fill(m_Ticks_____MCUs.begin(), m_Ticks_____MCUs.end(), 0);
  std::fill(m_TicksTransform.begin(), m_TicksTransform.end(), 0);
  std::fill(m_Ticks____Quant.begin(), m_Ticks____Quant.end(), 0);
  std::fill(m_Ticks_____Scan.begin(), m_Ticks_____Scan.end(), 0);
  std::fill(m_Ticks__Entropy.begin(), m_Ticks__Entropy.end(), 0);
  std::fill(m_Ticks_Stuffing.begin(), m_Ticks_Stuffing.end(), 0);

  m_TotalPictureIters = 0;
  m_TotalSliceIters   = 0;

  return TimeStats;
}

//=====================================================================================================================================================================================
// xEncoderSimple
//=====================================================================================================================================================================================
void xEncoderSimple::init(int32V2 PictureSize, eCrF ChromaFormat, int32 Quality, int32 RestartInterval, bool EmitAPP0, bool EmitQuantTabs, bool EmitHuffmanTabs)
{
  initCodecCommon(PictureSize, ChromaFormat);

  m_NumMCUsInSlice  = RestartInterval != 0 ? RestartInterval : m_NumMCUsInArea;

  m_Quality             = Quality;
  m_EmitAPP0            = EmitAPP0;
  m_EmitQuantTabs       = EmitQuantTabs;
  m_EmitHuffTabs        = EmitHuffmanTabs;

  //init markers
  m_APP0.InitDefault();
  m_QTs.resize(2);
  m_QTs[0].Init(0, eCmp::LM, Quality);
  m_QTs[1].Init(1, eCmp::CB, Quality); //any chroma so use CB
  m_RestartInterval = RestartInterval;
  m_SOF0.Init(m_PictureHeight, m_PictureWidth, 8, ChromaFormat, 2);
  m_HTs.resize(4);
  m_HTs[0].InitDefault(0, xJFIF::xHuffTable::eHuffClass::DC, eCmp::LM);
  m_HTs[1].InitDefault(0, xJFIF::xHuffTable::eHuffClass::AC, eCmp::LM);
  m_HTs[2].InitDefault(1, xJFIF::xHuffTable::eHuffClass::DC, eCmp::CB); //any chroma so use CB
  m_HTs[3].InitDefault(1, xJFIF::xHuffTable::eHuffClass::AC, eCmp::CB);
  m_SOS.Init(m_SOF0.getNumComponents(), 0, 1);

  //init toolbox
  m_Quant     .Init(m_QTs);
  m_EntropyEnc.Init(m_HTs);
  int32 NumBlocksInMCU      = m_SampFactorHor[0] * m_SampFactorVer[0] + m_SampFactorHor[1] * m_SampFactorVer[1] * 2;
  int32 MaxEncodedSliceSize = m_NumMCUsInSlice * NumBlocksInMCU * 256;
  m_EntropyBuffer.resize(MaxEncodedSliceSize);

  if(m_Ticks____Slice.empty()) { m_Ticks____Slice.resize(1, 0); }
  if(m_Ticks_____MCUs.empty()) { m_Ticks_____MCUs.resize(1, 0); }
  if(m_TicksTransform.empty()) { m_TicksTransform.resize(1, 0); }
  if(m_Ticks____Quant.empty()) { m_Ticks____Quant.resize(1, 0); }
  if(m_Ticks_____Scan.empty()) { m_Ticks_____Scan.resize(1, 0); }
  if(m_Ticks__Entropy.empty()) { m_Ticks__Entropy.resize(1, 0); }
  if(m_Ticks_Stuffing.empty()) { m_Ticks_Stuffing.resize(1, 0); }
}
void xEncoderSimple::encode(const xPicYUV* InputPicture, xByteBuffer* OutputBuffer)
{
  xJFIF::WriteSOI (OutputBuffer);
  if(m_EmitAPP0       ) { xJFIF::WriteAPP0(OutputBuffer, m_APP0); }
  if(m_EmitQuantTabs  ) { xJFIF::WriteDQT(OutputBuffer, m_QTs); }
  if(m_RestartInterval) { xJFIF::WriteDRI(OutputBuffer, m_RestartInterval); }
  xJFIF::WriteSOF0(OutputBuffer, m_SOF0);
  if(m_EmitHuffTabs   ) { xJFIF::WriteDHT(OutputBuffer, m_HTs); }
  xJFIF::WriteSOS (OutputBuffer, m_SOS);
  xEncodePicture(InputPicture, OutputBuffer);
  xJFIF::WriteEOI(OutputBuffer);
}
void xEncoderSimple::xEncodePicture(const xPicYUV* InputPicture, xByteBuffer* OutputBuffer)
{
  uint64 TP = xTSC();

  if(m_RestartInterval == 0) //no division - encode entire picture at once
  {
    xEncodeSlice(InputPicture, OutputBuffer, 0, m_NumMCUsInArea - 1);
  }
  else //divide picture into independent slices
  {
    for(int32 SliceIdx = 0, MCU_IdxFirst = 0; MCU_IdxFirst < m_NumMCUsInArea; SliceIdx++, MCU_IdxFirst+=m_RestartInterval)
    {
      int32 MCU_IdxLast = xMin(m_NumMCUsInArea, MCU_IdxFirst + m_RestartInterval) - 1;
      xEncodeSlice(InputPicture, OutputBuffer, MCU_IdxFirst, MCU_IdxLast);
      if(MCU_IdxLast != m_NumMCUsInArea - 1) { xJFIF::WriteRST(OutputBuffer, (uint8)((uint32)SliceIdx & (uint32)0x07)); }
    }
  }
  m_TotalPictureIters += 1;
  m_Ticks__Picture    += xTSC() - TP;
}
void xEncoderSimple::xEncodeSlice(const xPicYUV* InputPicture, xByteBuffer* OutputBuffer, int32 MCU_IdxFirst, int32 MCU_IdxLast)
{
  uint64 TP0 = xTSC();

  m_EntropyBuffer.reset();
  //m_EntropyEnc.StartSlice(&m_EntropyBuffer);
  m_EntropyEncDefault.StartSlice(&m_EntropyBuffer);

  const uint16* CmpPtrV   [] = { InputPicture->getAddr  (eCmp::LM), InputPicture->getAddr  (eCmp::CB), InputPicture->getAddr  (eCmp::CR), nullptr };
  const int32   CmpStrideV[] = { InputPicture->getStride(eCmp::LM), InputPicture->getStride(eCmp::CB), InputPicture->getStride(eCmp::CR),       0 };

  //loop over MCUs
  for(int32 MCU_Idx = MCU_IdxFirst; MCU_Idx <= MCU_IdxLast; MCU_Idx++)
  {    
    xEncodeMCU(CmpPtrV, CmpStrideV, MCU_Idx);
  }

  //m_EntropyEnc.FinishSlice();
  m_EntropyEncDefault.FinishSlice();

  uint64 TP1 = xTSC();
  //copy to output and add stuffing
  xJFIF::AddStuffing(OutputBuffer, &m_EntropyBuffer);

  uint64 TP2 = xTSC();

  m_TotalSliceIters += 1;
  m_Ticks____Slice[0] += TP2 - TP0;
  m_Ticks_Stuffing[0] += TP2 - TP1;
}
void xEncoderSimple::xEncodeMCU(const uint16* CmpPtrV[], const int32 CmpStrideV[], int32 MCU_Idx)
{
  uint64 TP = m_GatherTimeStats ? xTSC() : 0;

  //calculate MCU position
  int32 MCU_PosV = MCU_Idx / m_NumMCUsInWidth;
  int32 MCU_PosH = MCU_Idx % m_NumMCUsInWidth;

  //org samples buffer
  uint16 SamplesOrg[c_BA];

  //encode blocks
  for(int32 CmpIdx = 0; CmpIdx < m_NumOfComponents; CmpIdx++)
  {
    const int32 MCU_PelPosV = MCU_PosV << (2 + m_SampFactorVer[CmpIdx]);
    const int32 MCU_PelPosH = MCU_PosH << (2 + m_SampFactorHor[CmpIdx]);

    const uint16* CmpPtr    = CmpPtrV   [CmpIdx];
    const int32   CmpStride = CmpStrideV[CmpIdx];

    for(int32 V = 0; V < m_SampFactorVer[CmpIdx]; V++)
    {
      const int32   BlockPosV = MCU_PelPosV + (V << c_L2BS);
      const int32   BlockResV = m_CmpHeight[CmpIdx] - BlockPosV;
      const uint16* BlockPtrV = CmpPtr + BlockPosV * CmpStride;

      for(int32 H = 0; H < m_SampFactorHor[CmpIdx]; H++)
      {
        const int32   BlockPosH = MCU_PelPosH + (H << c_L2BS);
        const int32   BlockResH = m_CmpWidth[CmpIdx] - BlockPosH;
        const uint16* BlockPtr  = BlockPtrV + BlockPosH;

        if     (BlockResV >= 8 && BlockResH >= 8) { loadEntireBlock(SamplesOrg, BlockPtr, CmpStride); } //C++20 TODO use [[likely]]
        else if(BlockResV >  0 && BlockResH >  0) { loadExtendBlock(SamplesOrg, BlockPtr, CmpStride, xMin(BlockResH, c_BS), xMin(BlockResV, c_BS)); }
        else                                      { zeroEntireBlock(SamplesOrg); }
        xEncodeBlock(SamplesOrg, (eCmp)CmpIdx);
      }
    }
  }

  if(m_GatherTimeStats) { m_Ticks_____MCUs[0] += xTSC() - TP; }
}
void xEncoderSimple::xEncodeBlock(const uint16* SamplesOrg, eCmp CmpId)
{
  const int32 QuantTabId  = m_SOF0.getQuantTableId (CmpId);
  //const int32 HuffTabIdDC = m_SOS .getHuffTableIdDC(CmpId);
  //const int32 HuffTabIdAC = m_SOS .getHuffTableIdAC(CmpId);

  int16 CoeffsTrans[c_BA];
  int16 CoeffsQuant[c_BA];
  int16 CoeffsScan [c_BA];

  uint64 TP0 = m_GatherTimeStats ? xTSC() : 0;
  xTransform::FwdTransformDCT_8x8(CoeffsTrans, SamplesOrg);
  CoeffsTrans[0] -= xTransformConstants::c_FwdDcCorr; //DC correction - JPEG requires 128 to be subtracted from every input sample - could be done be DC -= 
  uint64 TP1 = m_GatherTimeStats ? xTSC() : 0;
  m_Quant.QuantScale(CoeffsQuant, CoeffsTrans, QuantTabId);
  uint64 TP2 = m_GatherTimeStats ? xTSC() : 0;
  xScan::Scan(CoeffsScan, CoeffsQuant);
  uint64 TP3 = m_GatherTimeStats ? xTSC() : 0;
  //m_EntropyEnc.EncodeBlock(CoeffsScan, CmpId, HuffTabIdDC, HuffTabIdAC);
  m_EntropyEncDefault.EncodeBlock(CoeffsScan, CmpId);
  uint64 TP4 = m_GatherTimeStats ? xTSC() : 0;

  if (m_GatherTimeStats)
  {
    m_Ticks__Entropy[0] += TP1 - TP0;
    m_Ticks_____Scan[0] += TP2 - TP1;
    m_Ticks____Quant[0] += TP3 - TP2;
    m_TicksTransform[0] += TP4 - TP3;
  }
}

//=====================================================================================================================================================================================
// xDecoderSimple
//=====================================================================================================================================================================================

void xDecoderSimple::init(int32V2 PictureSize, eCrF ChromaFormat, int32 Quality, int32 RestartInterval)
{
  initCodecCommon(PictureSize, ChromaFormat);

  m_NumMCUsInSlice  = RestartInterval != 0 ? RestartInterval : m_NumMCUsInArea;

  m_Quality         = Quality;

  //init markers
  m_APP0.InitDefault();
  m_QTs.resize(2);
  m_QTs[0].Init(0, eCmp::LM, Quality);
  m_QTs[1].Init(1, eCmp::CB, Quality); //any chroma so use CB
  m_RestartInterval = RestartInterval;
  m_SOF0.Init(m_PictureHeight, m_PictureWidth, 8, ChromaFormat, 2);
  m_HTs.resize(4);
  m_HTs[0].InitDefault(0, xJFIF::xHuffTable::eHuffClass::DC, eCmp::LM);
  m_HTs[1].InitDefault(0, xJFIF::xHuffTable::eHuffClass::AC, eCmp::LM);
  m_HTs[2].InitDefault(1, xJFIF::xHuffTable::eHuffClass::DC, eCmp::CB); //any chroma so use CB
  m_HTs[3].InitDefault(1, xJFIF::xHuffTable::eHuffClass::AC, eCmp::CB);
  m_SOS.Init(m_SOF0.getNumComponents(), 0, 1);

  //init toolbox
  m_Quant     .Init(m_QTs);
  m_EntropyDec.Init(m_HTs);
  int32 NumBlocksInMCU      = m_SampFactorHor[0] * m_SampFactorVer[0] + m_SampFactorHor[1] * m_SampFactorVer[1] * 2;
  int32 MaxEncodedSliceSize = m_NumMCUsInSlice * NumBlocksInMCU * 256;
  m_EntropyBuffer.resize(MaxEncodedSliceSize);

  if(m_Ticks____Slice.empty()) { m_Ticks____Slice.resize(1, 0); }
  if(m_Ticks_____MCUs.empty()) { m_Ticks_____MCUs.resize(1, 0); }
  if(m_TicksTransform.empty()) { m_TicksTransform.resize(1, 0); }
  if(m_Ticks____Quant.empty()) { m_Ticks____Quant.resize(1, 0); }
  if(m_Ticks_____Scan.empty()) { m_Ticks_____Scan.resize(1, 0); }
  if(m_Ticks__Entropy.empty()) { m_Ticks__Entropy.resize(1, 0); }
  if(m_Ticks_Stuffing.empty()) { m_Ticks_Stuffing.resize(1, 0); }
}
bool xDecoderSimple::init(xByteBuffer* InputBuffer)
{
  //save InputBuffer state
  int32 BeforeDataSize = InputBuffer->getDataSize();
    
  bool ReadAPP0 = false;
  bool ReadDQT  = false;
  bool ReadDRI  = false;
  bool ReadSOF0 = false;
  bool ReadDHT  = false;
  bool ReadSOS  = false;

  m_QTs.clear();
  m_HTs.clear();
  
  bool Result = xJFIF::ReadSOI(InputBuffer);
  if(Result == false) { return false; }

  bool ContinueReadingHeaders = true;
  while(ContinueReadingHeaders)
  {
    xJFIF::eMarker Type = xJFIF::IdentifySegment(InputBuffer);
    switch(Type)
    {
      case xJFIF::eMarker::APP0:
        Result = xJFIF::ReadAPP0(InputBuffer, m_APP0);
        if(Result) { ReadAPP0 = true; } else { return false; }
        break;
      case xJFIF::eMarker::DQT:        
        Result = xJFIF::ReadDQT(InputBuffer, m_QTs);
        if(Result) { ReadDQT = true; } else { return false; }
        break;
      case xJFIF::eMarker::DRI:
        Result = xJFIF::ReadDRI(InputBuffer, m_RestartInterval);
        if(Result) { ReadDRI = true; } else { return false; }
        break;
      case xJFIF::eMarker::SOF0:
        Result = xJFIF::ReadSOF0(InputBuffer, m_SOF0);
        if(Result) { ReadSOF0 = true; } else { return false; }
        break;
      case xJFIF::eMarker::DHT:
        Result = xJFIF::ReadDHT(InputBuffer, m_HTs);
        if(Result) { ReadDHT = true; } else { return false; }
        break;
      case xJFIF::eMarker::SOS:
        Result = xJFIF::ReadSOS(InputBuffer, m_SOS);
        if(Result) { ReadSOS = true; } else { return false; }
        ContinueReadingHeaders = false;
        break;
      default:
        return false;
        break;
    }
  }

  //restore buffer state
  int32 AfterDataSize = InputBuffer->getDataSize();
  int32 ReadedDataSize = BeforeDataSize - AfterDataSize;
  InputBuffer->modifyRead(-ReadedDataSize);

  //test if all required
  if(!ReadDQT || !ReadSOF0 || !ReadSOS) { return false; }

  if(!ReadAPP0) { m_APP0.InitDefault();  }
  if(!ReadDRI ) { m_RestartInterval = 0; }
  if(!ReadDHT )
  {
    m_HTs.resize(4);
    m_HTs[0].InitDefault(0, xJFIF::xHuffTable::eHuffClass::DC, eCmp::LM);
    m_HTs[1].InitDefault(0, xJFIF::xHuffTable::eHuffClass::AC, eCmp::LM);
    m_HTs[2].InitDefault(1, xJFIF::xHuffTable::eHuffClass::DC, eCmp::CB); //any chroma so use CB
    m_HTs[3].InitDefault(1, xJFIF::xHuffTable::eHuffClass::AC, eCmp::CB);
  }

  //set fields
  initCodecCommon({ m_SOF0.getWidth(), m_SOF0.getHeight() }, m_SOF0.DetermineChromaFormat());

  m_NumMCUsInSlice  = m_RestartInterval != 0 ? m_RestartInterval : m_NumMCUsInArea;

  m_Quality         = NOT_VALID;
  
  //try estimate Q
  if(m_QTs.size() == 2 && m_SOF0.getNumComponents() == 3 && m_SOF0.getQuantTableId(eCmp::CB) == m_SOF0.getQuantTableId(eCmp::CR))
  {
    int32 QTIL = m_SOF0.getQuantTableId(eCmp::LM);
    int32 QTIC = m_SOF0.getQuantTableId(eCmp::CB);

    if(m_QTs[QTIL].getPrecision() == 0 && m_QTs[QTIC].getPrecision() == 0)
    {
      int32 EstimatedQ = xJPEG_Constants::EstimateQ((uint8*)(m_QTs[QTIL].getTableData().data()), (uint8*)(m_QTs[QTIC].getTableData().data()));
      if(EstimatedQ != NOT_VALID)
      {
        std::vector<byte> TmpQuantTabL(64);
        std::vector<byte> TmpQuantTabC(64);
        xJPEG_Constants::GenerateQuantTableDef(TmpQuantTabL.data(), eCmp::LM, EstimatedQ);
        xJPEG_Constants::GenerateQuantTableDef(TmpQuantTabC.data(), eCmp::CB, EstimatedQ); //any chroma so use CB
        if((m_QTs[0].getTableData() == TmpQuantTabL) && (m_QTs[1].getTableData() == TmpQuantTabC)) { m_Quality = EstimatedQ; }
      }
    }
  }

  //init toolbox
  m_Quant     .Init(m_QTs);
  m_EntropyDec.Init(m_HTs);
  int32 NumBlocksInMCU      = m_SampFactorHor[0] * m_SampFactorVer[0] + m_SampFactorHor[1] * m_SampFactorVer[1] * 2;
  int32 MaxEncodedSliceSize = m_NumMCUsInSlice * NumBlocksInMCU * 256;
  m_EntropyBuffer.resize(MaxEncodedSliceSize);

  if(m_Ticks____Slice.empty()) { m_Ticks____Slice.resize(1, 0); }
  if(m_Ticks_____MCUs.empty()) { m_Ticks_____MCUs.resize(1, 0); }
  if(m_TicksTransform.empty()) { m_TicksTransform.resize(1, 0); }
  if(m_Ticks____Quant.empty()) { m_Ticks____Quant.resize(1, 0); }
  if(m_Ticks_____Scan.empty()) { m_Ticks_____Scan.resize(1, 0); }
  if(m_Ticks__Entropy.empty()) { m_Ticks__Entropy.resize(1, 0); }
  if(m_Ticks_Stuffing.empty()) { m_Ticks_Stuffing.resize(1, 0); }

  return true;
}
void xDecoderSimple::decode(xByteBuffer* InputBuffer, xPicYUV* OutputPicture)
{
  int32 StartOfScanOffset = xJFIF::FindSegment(InputBuffer, xJFIF::eMarker::SOS);
  InputBuffer->modifyRead(StartOfScanOffset);
  xJFIF::SkipSOS(InputBuffer);
  xDecodePicture(InputBuffer, OutputPicture);
  xJFIF::ReadEOI(InputBuffer);
}
void xDecoderSimple::xDecodePicture(xByteBuffer* InputBuffer, xPicYUV* OutputPicture)
{
  uint64 TP = xTSC();

  if(m_RestartInterval == 0) //no division - encode entire picture at once
  {
    xDecodeSlice(InputBuffer, OutputPicture, 0, m_NumMCUsInArea - 1);
  }
  else //divide picture into independent slices
  {
    for(int32 SliceIdx = 0, MCU_IdxFirst = 0; MCU_IdxFirst < m_NumMCUsInArea; SliceIdx++, MCU_IdxFirst+=m_RestartInterval)
    {
      int32 MCU_IdxLast = xMin(m_NumMCUsInArea, MCU_IdxFirst + m_RestartInterval) - 1;
      xDecodeSlice(InputBuffer, OutputPicture, MCU_IdxFirst, MCU_IdxLast);
      if(MCU_IdxLast != m_NumMCUsInArea - 1)
      {
        int32 RST = xJFIF::ReadRST(InputBuffer);
        if(RST != (SliceIdx & 0x7))
        {
          //on error
        }
      }
    }
  }

  m_TotalPictureIters += 1;
  m_Ticks__Picture    += xTSC() - TP;
}
void xDecoderSimple::xDecodeSlice(xByteBuffer* InputBuffer, xPicYUV* OutputPicture, int32 MCU_IdxFirst, int32 MCU_IdxLast)
{
  uint64 TP0 = xTSC();

  m_EntropyBuffer.reset();

  //copy from input and remove stuffing until next marker
  xJFIF::RemoveStuffing(&m_EntropyBuffer, InputBuffer);
  uint64 TP1 = xTSC();

  m_EntropyDec.StartSlice(&m_EntropyBuffer);
  uint16*     CmpPtrV   [] = { OutputPicture->getAddr  (eCmp::LM), OutputPicture->getAddr  (eCmp::CB), OutputPicture->getAddr  (eCmp::CR), nullptr };
  const int32 CmpStrideV[] = { OutputPicture->getStride(eCmp::LM), OutputPicture->getStride(eCmp::CB), OutputPicture->getStride(eCmp::CR),       0 };

  //loop over MCUs
  for(int32 MCU_Idx = MCU_IdxFirst; MCU_Idx <= MCU_IdxLast; MCU_Idx++)
  {
    xDecodeMCU(CmpPtrV, CmpStrideV, MCU_Idx);
  }

  m_EntropyDec.FinishSlice();

  uint64 TP2 = xTSC();

  m_TotalSliceIters   += 1;
  m_Ticks____Slice[0] += TP2 - TP0;
  m_Ticks_Stuffing[0] += TP1 - TP0;
}
void xDecoderSimple::xDecodeMCU(uint16* CmpPtrV[], const int32 CmpStrideV[], int32 MCU_Idx)
{
  uint64 TP = m_GatherTimeStats ? xTSC() : 0;

  //calculate MCU position
  int32 MCU_PosV    = MCU_Idx / m_NumMCUsInWidth;
  int32 MCU_PosH    = MCU_Idx % m_NumMCUsInWidth;

  //dec samples buffer
  uint16 SamplesDec[c_BA];

  //encode blocks
  for(int32 CmpIdx = 0; CmpIdx < m_NumOfComponents; CmpIdx++)
  {
    const int32 MCU_PelPosV = MCU_PosV << (2 + m_SampFactorVer[CmpIdx]);
    const int32 MCU_PelPosH = MCU_PosH << (2 + m_SampFactorHor[CmpIdx]);

    uint16* restrict CmpPtr    = CmpPtrV   [CmpIdx];
    const int32      CmpStride = CmpStrideV[CmpIdx];

    for(int32 V = 0; V < m_SampFactorVer[CmpIdx]; V++)
    {
      const int32 BlockPosV = MCU_PelPosV + (V << c_L2BS);
      const int32 BlockResV = m_CmpHeight[CmpIdx] - BlockPosV;
      uint16* restrict BlockPtrV = CmpPtr + BlockPosV * CmpStride;

      for(int32 H = 0; H < m_SampFactorHor[CmpIdx]; H++)
      {
        const int32 BlockPosH = MCU_PelPosH + (H << c_L2BS);
        const int32 BlockResH = m_CmpWidth[CmpIdx] - BlockPosH;
        uint16* restrict BlockPtr = BlockPtrV + BlockPosH;

        xDecodeBlock(SamplesDec, (eCmp)CmpIdx);

        if     (BlockResV >= 8 && BlockResH >= 8) { storeEntireBlock (BlockPtr, SamplesDec, CmpStride); } //C++20 TODO use [[likely]]
        else if(BlockResV >  0 && BlockResH >  0) { storePartialBlock(BlockPtr, SamplesDec, CmpStride, xMin(BlockResH, c_BS), xMin(BlockResV, c_BS)); }
        else                                      { /* do nothing */ }
      }
    }
  }

  if(m_GatherTimeStats) { m_Ticks_____MCUs[0] += xTSC() - TP; }
}
void xDecoderSimple::xDecodeBlock(uint16* SamplesDec, eCmp CmpId)
{
  int32 QuantTabId  = m_SOF0.getQuantTableId (CmpId);
  int32 HuffTabIdDC = m_SOS .getHuffTableIdDC(CmpId);
  int32 HuffTabIdAC = m_SOS .getHuffTableIdAC(CmpId);
    
  int16 CoeffsScan [c_BA];
  int16 CoeffsQuant[c_BA];
  int16 CoeffsTrans[c_BA];

  uint64 TP0 = m_GatherTimeStats ? xTSC() : 0;
  m_EntropyDec.DecodeBlock(CoeffsScan, CmpId, HuffTabIdDC, HuffTabIdAC);
  uint64 TP1 = m_GatherTimeStats ? xTSC() : 0;
  xScan::InvScan(CoeffsQuant, CoeffsScan);
  uint64 TP2 = m_GatherTimeStats ? xTSC() : 0;
  m_Quant.InvScale(CoeffsTrans, CoeffsQuant, QuantTabId);
  uint64 TP3 = m_GatherTimeStats ? xTSC() : 0;
  CoeffsTrans[0] += xTransformConstants::c_InvDcCorr; //DC correction - JPEG requires 128 to be subtracted from every input sample - could be done be DC -= 
  xTransform::InvTransformDCT_8x8(SamplesDec, CoeffsTrans);
  uint64 TP4 = m_GatherTimeStats ? xTSC() : 0;

  if (m_GatherTimeStats)
  {
    m_Ticks__Entropy[0] += TP1 - TP0;
    m_Ticks_____Scan[0] += TP2 - TP1;
    m_Ticks____Quant[0] += TP3 - TP2;
    m_TicksTransform[0] += TP4 - TP3;
  }
}

//=====================================================================================================================================================================================
// xDecoderParallel
//=====================================================================================================================================================================================
void xDecoderParallel::init(int32V2 PictureSize, eCrF ChromaFormat, int32 Quality, int32 RestartInterval)
{
  initCodecCommon(PictureSize, ChromaFormat);

  m_NumMCUsInSlice  = RestartInterval != 0 ? RestartInterval : m_NumMCUsInArea;

  m_Quality         = Quality;

  //init markers
  m_APP0.InitDefault();
  m_QTs.resize(2);
  m_QTs[0].Init(0, eCmp::LM, Quality);
  m_QTs[1].Init(1, eCmp::CB, Quality); //any chroma so use CB
  m_RestartInterval = RestartInterval;
  m_SOF0.Init(m_PictureHeight, m_PictureWidth, 8, ChromaFormat, 2);
  m_HTs.resize(4);
  m_HTs[0].InitDefault(0, xJFIF::xHuffTable::eHuffClass::DC, eCmp::LM);
  m_HTs[1].InitDefault(0, xJFIF::xHuffTable::eHuffClass::AC, eCmp::LM);
  m_HTs[2].InitDefault(1, xJFIF::xHuffTable::eHuffClass::DC, eCmp::CB); //any chroma so use CB
  m_HTs[3].InitDefault(1, xJFIF::xHuffTable::eHuffClass::AC, eCmp::CB);
  m_SOS.Init(m_SOF0.getNumComponents(), 0, 1);

  //init toolbox
  m_Quant     .Init(m_QTs);

  m_NumOfSlices    = m_RestartInterval != 0 ? (int32)std::ceil((flt64)m_NumMCUsInArea / (flt64)m_RestartInterval) : 1;
  m_NumMCUsInSlice = m_RestartInterval != 0 ? m_RestartInterval : m_NumMCUsInArea;

  m_EntropyDecs = std::vector<xEntropyDecoder>(m_NumOfSlices);
  for(int32 i = 0; i < (int32)m_EntropyDecs.size(); i++) { m_EntropyDecs[i].Init(m_HTs); }

  int32 NumBlocksInMCU      = std::accumulate(m_NumBlocksInMCU.cbegin(), m_NumBlocksInMCU.cend(), 0);
  int32 MaxEncodedSliceSize = m_NumMCUsInSlice * NumBlocksInMCU * c_BA * 2;
  m_EntropyBuffer.resize(MaxEncodedSliceSize);

  m_SliceBuffers.resize(m_NumOfSlices);
  for(int32 i = 0; i < (int32)m_SliceBuffers.size(); i++) { m_SliceBuffers[i].resize(MaxEncodedSliceSize); }

  m_Ticks____Slice.resize(m_NumOfSlices, 0);
  m_Ticks_____MCUs.resize(m_NumOfSlices, 0);
  m_TicksTransform.resize(m_NumOfSlices, 0);
  m_Ticks____Quant.resize(m_NumOfSlices, 0);
  m_Ticks_____Scan.resize(m_NumOfSlices, 0);
  m_Ticks__Entropy.resize(m_NumOfSlices, 0);
  m_Ticks_Stuffing.resize(m_NumOfSlices, 0);
}
bool xDecoderParallel::init(xByteBuffer* InputBuffer)
{
  //save InputBuffer state
  int32 BeforeDataSize = InputBuffer->getDataSize();
    
  bool ReadAPP0 = false;
  bool ReadDQT  = false;
  bool ReadDRI  = false;
  bool ReadSOF0 = false;
  bool ReadDHT  = false;
  bool ReadSOS  = false;

  m_QTs.clear();
  m_HTs.clear();
  
  bool Result = xJFIF::ReadSOI(InputBuffer);
  if(Result == false) { return false; }

  bool ContinueReadingHeaders = true;
  while(ContinueReadingHeaders)
  {
    xJFIF::eMarker Type = xJFIF::IdentifySegment(InputBuffer);
    switch(Type)
    {
      case xJFIF::eMarker::APP0:
        Result = xJFIF::ReadAPP0(InputBuffer, m_APP0);
        if(Result) { ReadAPP0 = true; } else { return false; }
        break;
      case xJFIF::eMarker::DQT:        
        Result = xJFIF::ReadDQT(InputBuffer, m_QTs);
        if(Result) { ReadDQT = true; } else { return false; }
        break;
      case xJFIF::eMarker::DRI:
        Result = xJFIF::ReadDRI(InputBuffer, m_RestartInterval);
        if(Result) { ReadDRI = true; } else { return false; }
        break;
      case xJFIF::eMarker::SOF0:
        Result = xJFIF::ReadSOF0(InputBuffer, m_SOF0);
        if(Result) { ReadSOF0 = true; } else { return false; }
        break;
      case xJFIF::eMarker::DHT:
        Result = xJFIF::ReadDHT(InputBuffer, m_HTs);
        if(Result) { ReadDHT = true; } else { return false; }
        break;
      case xJFIF::eMarker::SOS:
        Result = xJFIF::ReadSOS(InputBuffer, m_SOS);
        if(Result) { ReadSOS = true; } else { return false; }
        ContinueReadingHeaders = false;
        break;
      default:
        return false;
        break;
    }
  }

  //restore buffer state
  int32 AfterDataSize = InputBuffer->getDataSize();
  int32 ReadedDataSize = BeforeDataSize - AfterDataSize;
  InputBuffer->modifyRead(-ReadedDataSize);

  //test if all required
  if(!ReadDQT || !ReadSOF0 || !ReadSOS) { return false; }

  if(!ReadAPP0) { m_APP0.InitDefault();  }
  if(!ReadDRI ) { m_RestartInterval = 0; }
  if(!ReadDHT )
  {
    m_HTs.resize(4);
    m_HTs[0].InitDefault(0, xJFIF::xHuffTable::eHuffClass::DC, eCmp::LM);
    m_HTs[1].InitDefault(0, xJFIF::xHuffTable::eHuffClass::AC, eCmp::LM);
    m_HTs[2].InitDefault(1, xJFIF::xHuffTable::eHuffClass::DC, eCmp::CB); //any chroma so use CB
    m_HTs[3].InitDefault(1, xJFIF::xHuffTable::eHuffClass::AC, eCmp::CB);
  }

  //set fields
  initCodecCommon({ m_SOF0.getWidth(), m_SOF0.getHeight() }, m_SOF0.DetermineChromaFormat());

  m_NumMCUsInSlice  = m_RestartInterval != 0 ? m_RestartInterval : m_NumMCUsInArea;

  m_Quality         = NOT_VALID;
  
  //try estimate Q
  if(m_QTs.size() == 2 && m_SOF0.getNumComponents() == 3 && m_SOF0.getQuantTableId(eCmp::CB) == m_SOF0.getQuantTableId(eCmp::CR))
  {
    int32 QTIL = m_SOF0.getQuantTableId(eCmp::LM);
    int32 QTIC = m_SOF0.getQuantTableId(eCmp::CB);

    if(m_QTs[QTIL].getPrecision() == 0 && m_QTs[QTIC].getPrecision() == 0)
    {
      int32 EstimatedQ = xJPEG_Constants::EstimateQ((uint8*)(m_QTs[QTIL].getTableData().data()), (uint8*)(m_QTs[QTIC].getTableData().data()));
      if(EstimatedQ != NOT_VALID)
      {
        std::vector<byte> TmpQuantTabL(64);
        std::vector<byte> TmpQuantTabC(64);
        xJPEG_Constants::GenerateQuantTableDef(TmpQuantTabL.data(), eCmp::LM, EstimatedQ);
        xJPEG_Constants::GenerateQuantTableDef(TmpQuantTabC.data(), eCmp::CB, EstimatedQ); //any chroma so use CB
        if((m_QTs[0].getTableData() == TmpQuantTabL) && (m_QTs[1].getTableData() == TmpQuantTabC)) { m_Quality = EstimatedQ; }
      }
    }
  }

  //init toolbox
  m_Quant     .Init(m_QTs);

  m_NumOfSlices    = m_RestartInterval != 0 ? (int32)std::ceil((flt64)m_NumMCUsInArea / (flt64)m_RestartInterval) : 1;
  m_NumMCUsInSlice = m_RestartInterval != 0 ? m_RestartInterval : m_NumMCUsInArea;

  m_EntropyDecs = std::vector<xEntropyDecoder>(m_NumOfSlices);
  for(int32 i = 0; i < (int32)m_EntropyDecs.size(); i++) { m_EntropyDecs[i].Init(m_HTs); }

  int32 NumBlocksInMCU      = std::accumulate(m_NumBlocksInMCU.cbegin(), m_NumBlocksInMCU.cend(), 0);
  int32 MaxEncodedSliceSize = m_NumMCUsInSlice * NumBlocksInMCU * c_BA * 2;
  m_EntropyBuffer.resize(MaxEncodedSliceSize);

  m_SliceBuffers.resize(m_NumOfSlices);
  for(int32 i = 0; i < (int32)m_SliceBuffers.size(); i++) { m_SliceBuffers[i].resize(MaxEncodedSliceSize); }

  m_Ticks____Slice.resize(m_NumOfSlices, 0);
  m_Ticks_____MCUs.resize(m_NumOfSlices, 0);
  m_TicksTransform.resize(m_NumOfSlices, 0);
  m_Ticks____Quant.resize(m_NumOfSlices, 0);
  m_Ticks_____Scan.resize(m_NumOfSlices, 0);
  m_Ticks__Entropy.resize(m_NumOfSlices, 0);
  m_Ticks_Stuffing.resize(m_NumOfSlices, 0);

  return true;
}
void xDecoderParallel::decode(xByteBuffer* InputBuffer, xPicYUV* OutputPicture)
{
  int32 StartOfScanOffset = xJFIF::FindSegment(InputBuffer, xJFIF::eMarker::SOS);
  InputBuffer->modifyRead(StartOfScanOffset);
  xJFIF::SkipSOS(InputBuffer);
  xDecodePicture(InputBuffer, OutputPicture);
  xJFIF::ReadEOI(InputBuffer);
}
bool xDecoderParallel::xDecodePicture(xByteBuffer* InputBuffer, xPicYUV* OutputPicture)
{
  uint64 BegTick = xTSC();

  if(m_RestartInterval == 0) //no division - encode entire picture at once
  {
    m_SliceBuffers[0].reset();
    uint64 TP = m_GatherTimeStats ? xTSC() : 0;
    xJFIF::RemoveStuffing(&m_SliceBuffers[0], InputBuffer); //copy from input and remove stuffing until next marker
    if(m_GatherTimeStats) { m_Ticks_Stuffing[0] += xTSC() - TP; }
    xDecodeSlice(OutputPicture, 0);
  }
  else //divide picture into independent slices
  {
    if(!m_ThPI.isActive())
    {
      for(int32 SliceIdx = 0; SliceIdx < m_NumOfSlices; SliceIdx++) //loop over slices
      {
        uint64 TP = m_GatherTimeStats ? xTSC() : 0;
        xJFIF::RemoveStuffing(&m_SliceBuffers[SliceIdx], InputBuffer); //copy from input and remove stuffing until next marker
        if(m_GatherTimeStats) { m_Ticks_Stuffing[SliceIdx] += xTSC() - TP; }
        xDecodeSlice(OutputPicture, SliceIdx);
        if(SliceIdx != m_NumOfSlices - 1)
        {
          int32 RST = xJFIF::ReadRST(InputBuffer); if(RST != (SliceIdx & 0x7)) { return false; }
        }
      }
    }
    else
    {
      //parsing & splitting
      for(int32 SliceIdx = 0; SliceIdx < m_NumOfSlices; SliceIdx++) //loop over slices
      {
        m_SliceBuffers[SliceIdx].reset(); 
        uint64 TP = m_GatherTimeStats ? xTSC() : 0;
        xJFIF::RemoveStuffing(&m_SliceBuffers[SliceIdx], InputBuffer); //copy from input and remove stuffing until next marker
        if(m_GatherTimeStats) { m_Ticks_Stuffing[SliceIdx] += xTSC() - TP; }
        if(SliceIdx != m_NumOfSlices - 1)
        {
          int32 RST = xJFIF::ReadRST(InputBuffer); if(RST != (SliceIdx & 0x7)) { return false; }
        }
      }
      //decoding
      for(int32 SliceIdx = 0; SliceIdx < m_NumOfSlices; SliceIdx++) //loop over slices
      {
        m_ThPI.storeTask([this, &OutputPicture, SliceIdx](int32 /*ThIdx*/) { xDecodeSlice(OutputPicture, SliceIdx); });
      }
      m_ThPI.executeStoredTasks();
    }
  }

  m_TotalPictureIters += 1;
  m_TotalSliceIters   += m_NumOfSlices;
  m_Ticks__Picture    += xTSC() - BegTick;
  return true;
}
void xDecoderParallel::xDecodeSlice(xPicYUV* OutputPicture, int32 SliceIdx)
{
  uint64 TP0 = xTSC();

  m_EntropyDecs[SliceIdx].StartSlice(&m_SliceBuffers[SliceIdx]);
  uint16*     CmpPtrV   [] = { OutputPicture->getAddr  (eCmp::LM), OutputPicture->getAddr  (eCmp::CB), OutputPicture->getAddr  (eCmp::CR), nullptr };
  const int32 CmpStrideV[] = { OutputPicture->getStride(eCmp::LM), OutputPicture->getStride(eCmp::CB), OutputPicture->getStride(eCmp::CR),       0 };

  const int32 MCU_IdxFirst = SliceIdx * m_RestartInterval;
  const int32 MCU_IdxLast  = xMin(m_NumMCUsInArea, MCU_IdxFirst + m_NumMCUsInSlice) - 1;

  //loop over MCUs
  for(int32 MCU_Idx = MCU_IdxFirst; MCU_Idx <= MCU_IdxLast; MCU_Idx++)
  {
    xDecodeMCU(CmpPtrV, CmpStrideV, SliceIdx, MCU_Idx);
  }

  m_EntropyDecs[SliceIdx].FinishSlice();

  m_Ticks____Slice[SliceIdx] += xTSC() - TP0;
}
void xDecoderParallel::xDecodeMCU(uint16* CmpPtrV[], const int32 CmpStrideV[], int32 SliceIdx, int32 MCU_Idx)
{
  uint64 TP = m_GatherTimeStats ? xTSC() : 0;

  //calculate MCU position
  int32 MCU_PosV    = MCU_Idx / m_NumMCUsInWidth;
  int32 MCU_PosH    = MCU_Idx % m_NumMCUsInWidth;

  //dec samples buffer
  uint16 SamplesDec[c_BA];

  //encode blocks
  for(int32 CmpIdx = 0; CmpIdx < m_NumOfComponents; CmpIdx++)
  {
    const int32 MCU_PelPosV = MCU_PosV << (2 + m_SampFactorVer[CmpIdx]);
    const int32 MCU_PelPosH = MCU_PosH << (2 + m_SampFactorHor[CmpIdx]);

    uint16* restrict CmpPtr    = CmpPtrV   [CmpIdx];
    const int32      CmpStride = CmpStrideV[CmpIdx];

    for(int32 V = 0; V < m_SampFactorVer[CmpIdx]; V++)
    {
      const int32 BlockPosV = MCU_PelPosV + (V << c_L2BS);
      const int32 BlockResV = m_CmpHeight[CmpIdx] - BlockPosV;
      uint16* restrict BlockPtrV = CmpPtr + BlockPosV * CmpStride;

      for(int32 H = 0; H < m_SampFactorHor[CmpIdx]; H++)
      {
        const int32 BlockPosH = MCU_PelPosH + (H << c_L2BS);
        const int32 BlockResH = m_CmpWidth[CmpIdx] - BlockPosH;
        uint16* restrict BlockPtr = BlockPtrV + BlockPosH;

        xDecodeBlock(SamplesDec, SliceIdx, (eCmp)CmpIdx);

        if     (BlockResV >= 8 && BlockResH >= 8) { storeEntireBlock (BlockPtr, SamplesDec, CmpStride); } //C++20 TODO use [[likely]]
        else if(BlockResV >  0 && BlockResH >  0) { storePartialBlock(BlockPtr, SamplesDec, CmpStride, xMin(BlockResH, c_BS), xMin(BlockResV, c_BS)); }
        else                                      { /* do nothing */ }
      }
    }
  }

  if(m_GatherTimeStats) { m_Ticks_____MCUs[SliceIdx] += xTSC() - TP; }
}
void xDecoderParallel::xDecodeBlock(uint16* SamplesDec, int32 SliceIdx, eCmp CmpId)
{
  int32 QuantTabId  = m_SOF0.getQuantTableId (CmpId);
  int32 HuffTabIdDC = m_SOS .getHuffTableIdDC(CmpId);
  int32 HuffTabIdAC = m_SOS .getHuffTableIdAC(CmpId);
    
  int16 CoeffsScan [c_BA];
  int16 CoeffsQuant[c_BA];
  int16 CoeffsTrans[c_BA];

  uint64 TP0 = m_GatherTimeStats ? xTSC() : 0;
  m_EntropyDecs[SliceIdx].DecodeBlock(CoeffsScan, CmpId, HuffTabIdDC, HuffTabIdAC);
  uint64 TP1 = m_GatherTimeStats ? xTSC() : 0;
  xScan::InvScan(CoeffsQuant, CoeffsScan);
  uint64 TP2 = m_GatherTimeStats ? xTSC() : 0;
  m_Quant.InvScale(CoeffsTrans, CoeffsQuant, QuantTabId);
  uint64 TP3 = m_GatherTimeStats ? xTSC() : 0;
  CoeffsTrans[0] += xTransformConstants::c_InvDcCorr; //DC correction - JPEG requires 128 to be subtracted from every input sample - could be done be DC -= 
  xTransform::InvTransformDCT_8x8(SamplesDec, CoeffsTrans);
  uint64 TP4 = m_GatherTimeStats ? xTSC() : 0;

  if(m_GatherTimeStats)
  {
    m_Ticks__Entropy[SliceIdx] += TP1 - TP0;
    m_Ticks_____Scan[SliceIdx] += TP2 - TP1;
    m_Ticks____Quant[SliceIdx] += TP3 - TP2;
    m_TicksTransform[SliceIdx] += TP4 - TP3;
  }
}

//=====================================================================================================================================================================================

} //end of namespace PMBB::JPEG