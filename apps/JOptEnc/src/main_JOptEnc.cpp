/*
    SPDX-FileCopyrightText: 2019-2024 Jakub Stankowski <jakub.stankowski@put.poznan.pl>
    SPDX-License-Identifier: BSD-3-Clause
*/

//===============================================================================================================================================================================================================

#include "xAppJOptEnc.h"
#include "xFmtScn.h"
#include <filesystem>

using namespace PMBB_NAMESPACE;
using namespace PMBB_NAMESPACE::JPEG;

//===============================================================================================================================================================================================================
// Main
//===============================================================================================================================================================================================================
#ifndef APP_MAIN
#define APP_MAIN main
#endif

int32 APP_MAIN(int argc, char* argv[], char* /*envp*/[])
{
  fmt::printf("%s\n", xAppJPEG::c_BannerString);
  tTimePoint AppBeg = tClock::now();
  xAppJPEG AppJPEG;

  //===================================================================================================================
  //configuring
  //===================================================================================================================
  
  //parsing configuration
  AppJPEG.registerCmdParams();
  bool CfgLoadResult = AppJPEG.loadConfiguration(argc, const_cast<const char**>(argv));
  if(!CfgLoadResult) { xErrMsg::printError(AppJPEG.getErrorLog() + "\n\n", xAppJPEG::c_HelpString); return EXIT_FAILURE; }
  bool CfgReadResult = AppJPEG.readConfiguration();
  if(!CfgReadResult) { xErrMsg::printError(AppJPEG.getErrorLog() + "\n\n", xAppJPEG::c_HelpString); return EXIT_FAILURE; }
  const int32 VerboseLevel = AppJPEG.getVerboseLevel();

  if(VerboseLevel >= 2)
  { 
    fmt::print("WorkingDir = {}\n\n", std::filesystem::current_path().string());
    fmt::print("Commandline args:\n"); xCfgINI::printCommandlineArgs(argc, const_cast<const char**>(argv));
  }

  //print compile time setup
  if (VerboseLevel >= 5) { fmt::print("{}\n", xMiscUtilsCORE::formatBuildInfo       ()); }
  if (VerboseLevel >= 1) { fmt::print("{}\n", xMiscUtilsCORE::formatCompileTimeSetup()); }  

  //print config
  if(VerboseLevel >= 1) { fmt::print("{}\n", AppJPEG.formatConfiguration()); }

  //validate file names against input parameters
  eAppRes ValidFilesRes = AppJPEG.validateInputFiles();
  if(ValidFilesRes == eAppRes::Warning) { xErrMsg::printError(std::string("PARAMETERS WARNING: Invalid parameters\n") + AppJPEG.getErrorLog()); }
  if(ValidFilesRes == eAppRes::Error  ) { xErrMsg::printError(std::string("PARAMETERS WARNING: Invalid parameters\n") + AppJPEG.getErrorLog()); return EXIT_FAILURE; }

  //print configuration warnings
  std::string ConfigWarnings = AppJPEG.formatWarnings();
  if(!ConfigWarnings.empty()) { fmt::print("{}", ConfigWarnings); }

  //hardware concurency
  AppJPEG.setupMultithreading();
  if(VerboseLevel >= 1) { fmt::print("{}\n", AppJPEG.formatMultithreading()); }

  //spacer
  fmt::print("\n\n\n");

  //===================================================================================================================
  // preparation
  //===================================================================================================================
  if(VerboseLevel >= 2) { fmt::print("Initializing:\n"); }

  eAppRes SeqRes = AppJPEG.setupSeqAndBuffs();
  if(SeqRes == eAppRes::Error) { return EXIT_FAILURE; }

  AppJPEG.createProcessors();

  //===================================================================================================================
  //running
  //===================================================================================================================
  tTimePoint PrcBeg = tClock::now();
  eAppRes ClcRes = AppJPEG.processAllFrames();
  if(ClcRes == eAppRes::Error) { return EXIT_FAILURE; }
  tTimePoint PrcEnd = tClock::now();

  //===================================================================================================================
  //finalizing
  //===================================================================================================================
  if(VerboseLevel >= 1) { fmt::print("{}\n", AppJPEG.calibrateTimeStamp()); }
  fmt::print("\n\n");
  AppJPEG.combineFrameStats  ();
  AppJPEG.ceaseSeqAndBuffs   ();
  AppJPEG.ceaseMultithreading();

  //save results to file
  //std::ofstream file("output.txt", std::ios::app);

  //if (file.is_open()) {
  //  std::string results = AppJPEG.formatResultsFile();

  //  file << results;
  //  file.close();
  //}


  //printout results
  fmt::print("{}\n", AppJPEG.formatResultsStdOut());
  tTimePoint AppEnd = tClock::now();
  fmt::print("TotalProcessingTime  = {:.3f} s\n", std::chrono::duration_cast<tDurationS>(PrcEnd - PrcBeg).count());
  fmt::print("TotalApplicationTime = {:.3f} s\n", std::chrono::duration_cast<tDurationS>(AppEnd - AppBeg).count());
  fmt::print("END-OF-LOG\n");
  fflush(stdout);

  return EXIT_SUCCESS;
}
