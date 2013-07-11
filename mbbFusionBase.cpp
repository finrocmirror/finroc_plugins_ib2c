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
/*!\file    plugins/ib2c/mbbFusionBase.cpp
 *
 * \author  Tobias Föhst
 *
 * \date    2013-06-04
 *
 */
//----------------------------------------------------------------------
#include "plugins/ib2c/mbbFusionBase.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/tFrameworkElementTags.h"

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
const unsigned int cMAX_NUMBER_OF_INPUT_MODULES = 1000;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mbbFusionBase constructor
//----------------------------------------------------------------------
mbbFusionBase::mbbFusionBase(core::tFrameworkElement *parent, const std::string &name,
                             unsigned int number_of_input_modules,
                             tStimulationMode stimulation_mode, unsigned int number_of_inhibition_ports) :
  tModule(parent, name, stimulation_mode, number_of_inhibition_ports, "(F) "),

  number_of_input_modules(number_of_input_modules, data_ports::tBounds<unsigned int>(1, cMAX_NUMBER_OF_INPUT_MODULES, data_ports::tOutOfBoundsAction::ADJUST_TO_RANGE))
{
  core::tFrameworkElementTags::AddTag(*this, "ib2c_fusion");
}

//----------------------------------------------------------------------
// mbbFusionBase destructor
//----------------------------------------------------------------------
mbbFusionBase::~mbbFusionBase()
{}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
