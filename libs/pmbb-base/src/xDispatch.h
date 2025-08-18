/*
    SPDX-FileCopyrightText: 2019-2023 Jakub Stankowski <jakub.stankowski@put.poznan.pl>
    SPDX-License-Identifier: BSD-3-Clause
*/

#pragma once

#include "xCfgINI.h"
#include "xErrMsg.h"
#include "xProcInfo.h"
#include "xString.h"
#include "fmt/printf.h"
#include <vector>
#include <algorithm>
#include <iostream>

//===============================================================================================================================================================================================================

#if defined(BUILD_WITH_AMD64v1)
int main_AMD64v1(int argc, char* argv[], char* envp[]);
static constexpr bool c_BUILD_WITH_AMD64v1 = true;
#else
int main_AMD64v1(int /*argc*/, char* /*argv*/[], char* /*envp*/[]) { return EXIT_FAILURE; }
static constexpr bool c_BUILD_WITH_AMD64v1 = false;
#endif

#if defined(BUILD_WITH_AMD64v2)
int main_AMD64v2(int argc, char* argv[], char* envp[]);
static constexpr bool c_BUILD_WITH_AMD64v2 = true;
#else
int main_AMD64v2(int /*argc*/, char* /*argv*/[], char* /*envp*/[]) { return EXIT_FAILURE; }
static constexpr bool c_BUILD_WITH_AMD64v2 = false;
#endif

#if defined(BUILD_WITH_AMD64v3)
int main_AMD64v3(int argc, char* argv[], char* envp[]);
static constexpr bool c_BUILD_WITH_AMD64v3 = true;
#else
int main_AMD64v3(int /*argc*/, char* /*argv*/[], char* /*envp*/[]) { return EXIT_FAILURE; }
static constexpr bool c_BUILD_WITH_AMD64v3 = false;
#endif

#if defined(BUILD_WITH_AMD64v4)
int main_AMD64v4(int argc, char* argv[], char* envp[]);
static constexpr bool c_BUILD_WITH_AMD64v4 = true;
#else
int main_AMD64v4(int /*argc*/, char* /*argv*/[], char* /*envp*/[]) { return EXIT_FAILURE; }
static constexpr bool c_BUILD_WITH_AMD64v4 = false;
#endif

//===============================================================================================================================================================================================================

static std::vector<PMBB_BASE::xProcInfo::eMFL> determineSoftwareAvailable()
{
  using eMFL = PMBB_BASE::xProcInfo::eMFL;
  std::vector<eMFL> LevelAvailable; LevelAvailable.reserve(4);
  if constexpr(c_BUILD_WITH_AMD64v4) { LevelAvailable.push_back(eMFL::AMD64v4); }
  if constexpr(c_BUILD_WITH_AMD64v3) { LevelAvailable.push_back(eMFL::AMD64v3); }
  if constexpr(c_BUILD_WITH_AMD64v2) { LevelAvailable.push_back(eMFL::AMD64v2); }
  if constexpr(c_BUILD_WITH_AMD64v1) { LevelAvailable.push_back(eMFL::AMD64v1); }
  return LevelAvailable;
}

static void printSoftwareAvailable()
{
  fmt::print("SoftwareAvailable:\n");
  if constexpr(c_BUILD_WITH_AMD64v4) { fmt::print("  main_AMD64v4 @ {:p}\n", (void*)main_AMD64v4); }
  if constexpr(c_BUILD_WITH_AMD64v3) { fmt::print("  main_AMD64v3 @ {:p}\n", (void*)main_AMD64v3); }
  if constexpr(c_BUILD_WITH_AMD64v2) { fmt::print("  main_AMD64v2 @ {:p}\n", (void*)main_AMD64v2); }
  if constexpr(c_BUILD_WITH_AMD64v1) { fmt::print("  main_AMD64v1 @ {:p}\n", (void*)main_AMD64v1); }
}

static std::string MflToMainName(PMBB_BASE::xProcInfo::eMFL Mfl)
{
  using eMFL = PMBB_BASE::xProcInfo::eMFL;
  return Mfl == eMFL::UNDEFINED ? "MotImplemented" :
         Mfl == eMFL::AMD64v1   ? "main_AMD64v1" :
         Mfl == eMFL::AMD64v2   ? "main_AMD64v2" :
         Mfl == eMFL::AMD64v3   ? "main_AMD64v3" :
         Mfl == eMFL::AMD64v4   ? "main_AMD64v4" :
                                  "INVALID";
}

using tMainPtr = int(*)(int, char*[], char*[]);

static tMainPtr selectMain(PMBB_BASE::xProcInfo::eMFL Mfl)
{
  switch(Mfl)
  {
  case PMBB_BASE::xProcInfo::eMFL::AMD64v1: return main_AMD64v1; break;
  case PMBB_BASE::xProcInfo::eMFL::AMD64v2: return main_AMD64v2; break;
  case PMBB_BASE::xProcInfo::eMFL::AMD64v3: return main_AMD64v3; break;
  case PMBB_BASE::xProcInfo::eMFL::AMD64v4: return main_AMD64v4; break;
  default: return nullptr; break;
  }
}

//static int dispatchMain(PMBB_BASE::xProcInfo::eMFL Mfl, int argc, char* argv[], char* envp[])
//{
//  switch(Mfl)
//  {
//  case PMBB_BASE::xProcInfo::eMFL::AMD64v1: return main_AMD64v1(argc, argv, envp); break;
//  case PMBB_BASE::xProcInfo::eMFL::AMD64v2: return main_AMD64v2(argc, argv, envp); break;
//  case PMBB_BASE::xProcInfo::eMFL::AMD64v3: return main_AMD64v3(argc, argv, envp); break;
//  case PMBB_BASE::xProcInfo::eMFL::AMD64v4: return main_AMD64v4(argc, argv, envp); break;
//  default: return EXIT_FAILURE; break;
//  }
//}

static const std::string_view HelpString =
R"PMBBRAWSTRING(
=============================================================================
PMBB runtime dispatch module for x86-64 Microarchitecture Feature Levels

Usage:

 Cmd                | Description
 --DispatchForceMFL   Force dispatcher to selected microarchitecture (optional, default=UNDEFINED)
 --DispatchVerbose    Verbose level for runtime dispatch module      (optional, default=0)
=============================================================================
)PMBBRAWSTRING";


//===============================================================================================================================================================================================================

int main(int argc, char* argv[], char* envp[])
{
  using eMFL = PMBB_BASE::xProcInfo::eMFL;

  // header
  fmt::print("PMBB runtime dispatch module for x86-64 Microarchitecture Feature Levels \n");
  fmt::print("-----------------------------------------------------------------------------\n");
  std::cout.flush();

  // parsing configuration
  PMBB_BASE::xCfgINI::xParser* CfgParser = new PMBB_BASE::xCfgINI::xParser;
  CfgParser->addCmdParm("", "DispatchForceMFL", "", "DispatchForceMFL");
  CfgParser->addCmdParm("", "DispatchVerbose" , "", "DispatchVerbose" );
  CfgParser->setUnknownCmdParams(true);
  CfgParser->setEmptyCmdParams  (true);
  bool CommandlineResult = CfgParser->loadFromCmdln(argc, const_cast<const char**>(argv));
  if(!CommandlineResult) { PMBB_BASE::xErrMsg::printError(std::string("! invalid commandline\n") + CfgParser->getParsingLog() + "\n\n", HelpString); return EXIT_FAILURE; }

  std::string LevelOverrideS = CfgParser->getParam1stArg("DispatchForceMFL", PMBB_BASE::xProcInfo::xMflToStr(eMFL::UNDEFINED));
  eMFL        LevelOverride  = PMBB_BASE::xProcInfo::xStrToMfl(LevelOverrideS);
  if(LevelOverride != eMFL::UNDEFINED) { fmt::print("UserOverridedMFL = {}\n", PMBB_BASE::xProcInfo::xMflToStr(LevelOverride)); }
  bool        Verbose        = CfgParser->getParam1stArg("DispatchVerbose", false);

  // examining system
  PMBB_BASE::xProcInfo ProcInfo;
  ProcInfo.detectSysInfo();
  if(Verbose) { fmt::print("{}\n", ProcInfo.formatSysInfo()); }
  eMFL LevelDetected = ProcInfo.determineMicroArchFeatureLevel();
  fmt::print("DetectedHardwareMFL = {}\n", PMBB_BASE::xProcInfo::xMflToStr(LevelDetected));
  if(LevelDetected == eMFL::UNDEFINED) { fmt::print("Errrrrrrrror!!!\n"); return EXIT_FAILURE; }
   
  // examine software
  std::vector<eMFL> LevelSoftware = determineSoftwareAvailable();
  fmt::print("DetectedSoftwareMFL = ");
  for(eMFL l : LevelSoftware) { fmt::print("{} ", PMBB_BASE::xProcInfo::xMflToStr(l)); }
  fmt::print("\n"); 
  if(Verbose) { printSoftwareAvailable(); }
  std::cout.flush();

  // select highest available and compatible level
  eMFL LevelSelected = eMFL::UNDEFINED;
  for(eMFL l : LevelSoftware) { if((int32_t)LevelDetected <= (int32_t)l) { LevelSelected = LevelDetected; break; } }
  if(LevelSelected == eMFL::UNDEFINED) { fmt::print("Errrrrrrrror!!!\n"); return EXIT_FAILURE; }

  // optional override
  if(LevelOverride != eMFL::UNDEFINED && std::find(LevelSoftware.begin(), LevelSoftware.end(), LevelOverride) != std::end(LevelSoftware)) { LevelSelected = LevelOverride; }

  // dispatch
  tMainPtr MainPtr = selectMain(LevelSelected);
  fmt::print("Dispatching {} ----> {}", PMBB_BASE::xProcInfo::xMflToStr(LevelSelected), MflToMainName(LevelSelected));
  if(Verbose) { fmt::print(" @ {:p}", (void*)MainPtr); }
  fmt::print("\n-----------------------------------------------------------------------------\n\n"); std::cout.flush();

  int RetVal = MainPtr(argc, argv, envp);
  //int RetVal = dispatchMain(LevelSelected, argc, argv, envp);

  return RetVal;
}

//===============================================================================================================================================================================================================
