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
/*!\file    plugins/ib2c/test/mbbTestModule.h
 *
 * \author  Tobias Föhst
 *
 * \date    2012-07-01
 *
 * \brief Contains mbbTestModule
 *
 * \b mbbTestModule
 *
 * This is a test for iB2C modules.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__ib2c__test__mbbTestModule_h__
#define __plugins__ib2c__test__mbbTestModule_h__

#include "plugins/ib2c/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/math/tAngle.h"

//----------------------------------------------------------------------
// Internal includes with ""
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
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This is a test for iB2C modules.
 */
class mbbTestModule : public ib2c::tModule
{
  static runtime_construction::tStandardCreateModuleAction<mbbTestModule> cCREATE_ACTION;

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tInput<int> input1;
  tInput<double> input2;
  tInput<rrlib::math::tAngleRad> input3;

  tOutput<int> output1;
  tOutput<double> output2;
  tOutput<rrlib::math::tAngleRad> output3;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mbbTestModule(core::tFrameworkElement *parent, const std::string &name = "TestModule",
                tStimulationMode stimulation_mode = tStimulationMode::AUTO, unsigned int number_of_inhibition_ports = 0);

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  ~mbbTestModule()
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual bool ProcessTransferFunction(double activation) override;

  virtual tActivity CalculateActivity(std::vector<tActivity> &derived_activities, double activation) const override;

  virtual tTargetRating CalculateTargetRating(double activation) const override;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
