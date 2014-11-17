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
/*!\file    plugins/ib2c/test/gbbTestGroup.h
 *
 * \author  Tobias Föhst
 *
 * \date    2013-07-04
 *
 * \brief Contains gbbTestGroup
 *
 * \b gbbTestGroup
 *
 * This is a test for iB2C groups.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__ib2c__test__gbbTestGroup_h__
#define __plugins__ib2c__test__gbbTestGroup_h__

#include "plugins/ib2c/tGroup.h"

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
 * This is a test for iB2C groups.
 */
class gbbTestGroup : public ib2c::tGroup
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tInput<int> input1_1;
  tInput<double> input1_2;
  tInput<rrlib::math::tAngleRad> input1_3;

  tInput<int> input2_1;
  tInput<double> input2_2;
  tInput<rrlib::math::tAngleRad> input2_3;

  tOutput<int> output1;
  tOutput<double> output2;
  tOutput<rrlib::math::tAngleRad> output3;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  gbbTestGroup(core::tFrameworkElement *parent, const std::string &name = "TestGroup",
               tStimulationMode stimulation_mode = tStimulationMode::AUTO, unsigned int number_of_inhibition_ports = 0,
               const std::string &structure_config_file = __FILE__".xml");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of groups is declared private to avoid accidental deletion. Deleting
   * groups is already handled by the framework.
   */
  ~gbbTestGroup();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
