//
// You received this file as part of Finroc
// A Framework for intelligent robot control
//
// Copyright (C) Robot Makers GmbH (www.robotmakers.de)
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
/*!\file    plugins/ib2c/mbbLimitOutput.h
 *
 * \author  Jochen Hirth
 *
 * \date    2012-10-30
 *
 * \brief Contains mbbLimitOutput
 *
 * \b mbbLimitOutput
 *
 * This module draws the output signals to 0 in case the activation is 0,
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__ib2c__mbbLimitOutput_h__
#define __plugins__ib2c__mbbLimitOutput_h__

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
 * This module draws the output signals to 0 in case the activation is 0,
 */
class mbbLimitOutput : public ib2c::tModule
{
  static runtime_construction::tStandardCreateModuleAction<mbbLimitOutput> cCREATE_ACTION;

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tStaticParameter<unsigned int> number_of_signals;

  std::vector<tInput<double>> input_signals;
  std::vector<tOutput<double>> output_signals;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mbbLimitOutput(core::tFrameworkElement *parent, const std::string &name = "LimitOutput",
                 tStimulationMode stimulation_mode = tStimulationMode::AUTO, unsigned int number_of_inhibition_ports = 0);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:
  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mbbLimitOutput();

  virtual void OnStaticParameterChange();

  virtual bool ProcessTransferFunction(double activation);

  virtual ib2c::tActivity CalculateActivity(std::vector<ib2c::tActivity> &derived_activities, double activation) const;

  virtual ib2c::tTargetRating CalculateTargetRating(double activation) const;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
