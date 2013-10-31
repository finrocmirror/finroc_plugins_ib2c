//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    plugins/ib2c/test/gbbTestGroup.cpp
 *
 * \author  Tobias Föhst
 *
 * \date    2013-07-04
 *
 */
//----------------------------------------------------------------------
#include "plugins/ib2c/test/gbbTestGroup.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/ib2c/test/mbbTestModule.h"
#include "plugins/ib2c/mbbFusion.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace ib2c
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<gbbTestGroup> cCREATE_ACTION_FOR_GBB_TESTGROUP("TestGroup");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gTestGroup constructor
//----------------------------------------------------------------------
gbbTestGroup::gbbTestGroup(core::tFrameworkElement *parent, const std::string &name,
                           tStimulationMode stimulation_mode, unsigned int number_of_inhibition_ports,
                           const std::string &structure_config_file) :
  ib2c::tGroup(parent, name, stimulation_mode, number_of_inhibition_ports, structure_config_file)
{
  this->RegisterCharacteristicModule(new mbbFusion<int, double, rrlib::math::tAngleRad>(this, "Fusion", 2));

  mbbTestModule *module_1 = new finroc::ib2c::mbbTestModule(this, "Module 1", tStimulationMode::ENABLED);
  mbbTestModule *module_2 = new finroc::ib2c::mbbTestModule(this, "Module 2", tStimulationMode::ENABLED);

  this->input1_1.ConnectTo(module_1->input1);
  this->input1_2.ConnectTo(module_1->input2);
  this->input1_3.ConnectTo(module_1->input3);

  this->input2_1.ConnectTo(module_2->input1);
  this->input2_2.ConnectTo(module_2->input2);
  this->input2_3.ConnectTo(module_2->input3);

  mbbFusionBase &fusion = static_cast<mbbFusionBase &>(this->CharacteristicModule());

  module_1->activity.ConnectTo(fusion.InputActivity(0));
  module_1->target_rating.ConnectTo(fusion.InputTargetRating(0));
  module_1->output1.ConnectTo(fusion.InputPort(0, 0));
  module_1->output2.ConnectTo(fusion.InputPort(0, 1));
  module_1->output3.ConnectTo(fusion.InputPort(0, 2));

  module_2->activity.ConnectTo(fusion.InputActivity(1));
  module_2->target_rating.ConnectTo(fusion.InputTargetRating(1));
  module_2->output1.ConnectTo(fusion.InputPort(1, 0));
  module_2->output2.ConnectTo(fusion.InputPort(1, 1));
  module_2->output3.ConnectTo(fusion.InputPort(1, 2));

  fusion.OutputPort(0).ConnectTo(this->output1);
  fusion.OutputPort(1).ConnectTo(this->output2);
  fusion.OutputPort(2).ConnectTo(this->output3);
}

//----------------------------------------------------------------------
// gbbTestGroup destructor
//----------------------------------------------------------------------
gbbTestGroup::~gbbTestGroup()
{}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
