//
// You received this file as part of Finroc
// A Framework for intelligent robot control
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
core::tStandardCreateModuleAction<mbbTestModule> mbbTestModule::cCREATE_ACTION("TestModule");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mbbTestModule constructor
//----------------------------------------------------------------------
mbbTestModule::mbbTestModule(core::tFrameworkElement *parent, const util::tString &name)
  : ib2c::tModule(parent, name)
{}

//----------------------------------------------------------------------
// mbbTestModule destructor
//----------------------------------------------------------------------
mbbTestModule::~mbbTestModule()
{}

//----------------------------------------------------------------------
// mbbTestModule ProcessTransferFunction
//----------------------------------------------------------------------
void mbbTestModule::ProcessTransferFunction(double activation)
{}

//----------------------------------------------------------------------
// mbbTestModule CalculateActivity
//----------------------------------------------------------------------
double mbbTestModule::CalculateActivity(std::vector<double> &derived_activity, double activation) // const FIXME
{
  RRLIB_LOG_PRINTF(rrlib::logging::eLL_USER, "Activity = ", activation);
  return activation;
}

//----------------------------------------------------------------------
// mbbTestModule CalculateTargetRating
//----------------------------------------------------------------------
double mbbTestModule::CalculateTargetRating() // const FIXME
{
  return 0.5;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
