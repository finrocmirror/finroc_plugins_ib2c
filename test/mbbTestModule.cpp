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
/*!\file    plugins/ib2c/test/mbbTestModule.cpp
 *
 * \author  Tobias Föhst
 *
 * \date    2012-07-01
 *
 */
//----------------------------------------------------------------------
#include "plugins/ib2c/test/mbbTestModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/logging/messages.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

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
runtime_construction::tStandardCreateModuleAction<mbbTestModule> cCREATE_ACTION_FOR_MBB_TEST_MODULE("TestModule");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mbbTestModule constructor
//----------------------------------------------------------------------
mbbTestModule::mbbTestModule(core::tFrameworkElement *parent, const std::string &name,
                             tStimulationMode stimulation_mode, unsigned int number_of_inhibition_ports) :
  tModule(parent, name, stimulation_mode, number_of_inhibition_ports)
{}

//----------------------------------------------------------------------
// mbbTestModule ProcessTransferFunction
//----------------------------------------------------------------------
bool mbbTestModule::ProcessTransferFunction(double activation)
{
  this->output1.Publish(this->input1.Get());
  this->output2.Publish(this->input2.Get());
  this->output3.Publish(this->input3.Get());

  return true;
}

//----------------------------------------------------------------------
// mbbTestModule CalculateActivity
//----------------------------------------------------------------------
tActivity mbbTestModule::CalculateActivity(std::vector<tActivity> &derived_activity, double activation) const
{
  return activation;
}

//----------------------------------------------------------------------
// mbbTestModule CalculateTargetRating
//----------------------------------------------------------------------
tTargetRating mbbTestModule::CalculateTargetRating(double activation) const
{
  return 0.5;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
