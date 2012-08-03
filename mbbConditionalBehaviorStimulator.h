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
/*!\file    plugins/ib2c/mbbConditionalBehaviorStimulator.h
 *
 * \author  Tobias Föhst
 *
 * \date    2012-06-20
 *
 * \brief Contains mbbConditionalBehaviorStimulator
 *
 * \b mbbConditionalBehaviorStimulator
 *
 * This class implements the iB2C Conditional Behavior Stimulator (CBS)
 * that allows to easily create behavior activity sequences.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__ib2c__mbbConditionalBehaviorStimulator_h__
#define __plugins__ib2c__mbbConditionalBehaviorStimulator_h__

#include "plugins/ib2c/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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
 * This class implements the iB2C Conditional Behavior Stimulator (CBS)
 * that allows to easily create behavior activity sequences.
 */
class mbbConditionalBehaviorStimulator : public ib2c::tModule
{
  static core::tStandardCreateModuleAction<mbbConditionalBehaviorStimulator> cCREATE_ACTION;

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  std::vector<tInput<>> enabling_input;
  std::vector<tInput<>> ordering_input;
  std::vector<tInput<>> permanent_input;

  std::vector<tInput<>> enabling_output;
  std::vector<tInput<>> ordering_output;
  std::vector<tInput<>> permanent_output;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mbbConditionalBehaviorStimulator(core::tFrameworkElement *parent, const util::tString &name = "ConditionalBehaviorStimulator");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual bool ProcessTransferFunction(double activation);

  virtual double CalculateActivity(std::vector<double> &derived_activities, double activity) const;

  virtual double CalculateTargetRating(double activation) const;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
