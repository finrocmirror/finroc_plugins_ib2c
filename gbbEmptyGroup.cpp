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
/*!\file    plugins/ib2c/gbbEmptyGroup.cpp
 *
 * \author  Tobias Föhst
 *
 * \date    2013-07-11
 *
 */
//----------------------------------------------------------------------
#include "plugins/ib2c/gbbEmptyGroup.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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
runtime_construction::tStandardCreateModuleAction<gbbEmptyGroup> cCREATE_ACTION_FOR_GBB_EMPTYGROUP("EmptyGroup");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gTestGroup constructor
//----------------------------------------------------------------------
gbbEmptyGroup::gbbEmptyGroup(core::tFrameworkElement *parent, const std::string &name,
                             tStimulationMode stimulation_mode, unsigned int number_of_inhibition_ports,
                             const std::string &structure_config_file) :
  ib2c::tGroup(parent, name, stimulation_mode, number_of_inhibition_ports, structure_config_file)
{}

//----------------------------------------------------------------------
// gbbEmptyGroup destructor
//----------------------------------------------------------------------
gbbEmptyGroup::~gbbEmptyGroup()
{}

//----------------------------------------------------------------------
// gbbEmptyGroup OnStaticParameterChange
//----------------------------------------------------------------------
void gbbEmptyGroup::OnStaticParameterChange()
{
  if (this->characteristic_module.HasChanged() && !this->characteristic_module.Get().empty())
  {
    tModule *module = dynamic_cast<tModule *>(this->GetChild(this->characteristic_module.Get()));
    if (module)
    {
      this->RegisterCharacteristicModule(module);
    }
    else
    {
      FINROC_LOG_PRINT(ERROR, "Could not find module '", this->characteristic_module.Get(), "'. This group still has no characteristic module!");
      this->characteristic_module.Set("");
    }
  }

  tGroup::OnStaticParameterChange();
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
