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
/*!\file    plugins/ib2c/mNumberToActivityConverter.cpp
 *
 * \author  Tobias Föhst
 *
 * \date    2012-09-19
 *
 */
//----------------------------------------------------------------------
#include "plugins/ib2c/mNumberToActivityConverter.h"

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
runtime_construction::tStandardCreateModuleAction<mNumberToActivityConverter> mNumberToActivityConverter::cCREATE_ACTION("NumberToActivityConverter");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mNumberToActivityConverter constructor
//----------------------------------------------------------------------
mNumberToActivityConverter::mNumberToActivityConverter(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name),
  number_of_ports(1)
{}

//----------------------------------------------------------------------
// mNumberToActivityConverter destructor
//----------------------------------------------------------------------
mNumberToActivityConverter::~mNumberToActivityConverter()
{}

//----------------------------------------------------------------------
// mNumberToActivityConverter EvaluateParameters
//----------------------------------------------------------------------
void mNumberToActivityConverter::OnParameterChange()
{
  if (this->number_of_ports.HasChanged())
  {
    while (this->input.size() > this->number_of_ports.Get())
    {
      this->input.back().GetWrapped()->ManagedDelete();
      this->input.pop_back();
      this->output.back().GetWrapped()->ManagedDelete();
      this->output.pop_back();
    }
    for (size_t i = this->input.size(); i < this->number_of_ports.Get(); ++i)
    {
      this->input.push_back(tInput<double>("Input Signal " + std::to_string(i + 1), this));
      this->output.push_back(tOutput<tActivity>("Output Signal " + std::to_string(i + 1), this));
    }
  }
}

//----------------------------------------------------------------------
// mNumberToActivityConverter Update
//----------------------------------------------------------------------
void mNumberToActivityConverter::Update()
{
  if (this->InputChanged())
  {
    for (size_t i = 0; i < this->input.size(); ++i)
    {
      if (this->input[i].HasChanged())
      {
        this->output[i].Publish(this->input[i].Get());
      }
    }
  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
