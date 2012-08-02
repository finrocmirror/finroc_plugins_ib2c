//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    pTestBehaviours.cpp
 *
 * \author  Bernd-Helge Schäfer
 * \author  Tobias Föhst
 *
 * \date    2011-01-09
 *
 */
//----------------------------------------------------------------------
#include "core/default_main_wrapper.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/ib2c/test/mbbTestModule.h"
#include "plugins/ib2c/mbbFusion.h"
//#include "plugins/ib2c/mbbConditionalBehaviorStimulator.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
const char * const cPROGRAM_VERSION = "ver 1.0";
const char * const cPROGRAM_DESCRIPTION = "This program executes the TestBehaviors module/group.";

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// StartUp
//----------------------------------------------------------------------
void StartUp()
{}

//----------------------------------------------------------------------
// InitMainGroup
//----------------------------------------------------------------------
void InitMainGroup(finroc::core::tThreadContainer *main_thread, std::vector<char *> remaining_args)
{
  finroc::ib2c::mbbTestModule *module_1 = new finroc::ib2c::mbbTestModule(main_thread, "Module 1");
  finroc::ib2c::mbbTestModule *module_2 = new finroc::ib2c::mbbTestModule(main_thread, "Module 2");
  finroc::ib2c::mbbTestModule *module_3 = new finroc::ib2c::mbbTestModule(main_thread, "Module 3");

  finroc::ib2c::mbbFusion<int, double, rrlib::math::tAngleRad> *fusion = new finroc::ib2c::mbbFusion<int, double, rrlib::math::tAngleRad>(main_thread, "Fusion");

  module_1->activity.ConnectTo(fusion->InputActivity(0));
  module_1->target_rating.ConnectTo(fusion->InputTargetRating(0));
  module_1->output1.ConnectTo(fusion->InputPort(0, 0));
  fusion->OutputPort(1).ConnectTo(module_2->output2);

//  new finroc::ib2c::mbbConditionalBehaviorStimulator(main_thread, "CBS");

  main_thread->SetCycleTime(500);
}
