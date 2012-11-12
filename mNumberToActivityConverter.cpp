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
#include "rrlib/util/sStringUtils.h"
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
core::tStandardCreateModuleAction<mNumberToActivityConverter> mNumberToActivityConverter::cCREATE_ACTION("NumberToActivityConverter");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mNumberToActivityConverter constructor
//----------------------------------------------------------------------
mNumberToActivityConverter::mNumberToActivityConverter(core::tFrameworkElement *parent, const util::tString &name) :
  tModule(parent, name),
  number_of_values(1)
{}

//----------------------------------------------------------------------
// mNumberToActivityConverter destructor
//----------------------------------------------------------------------
mNumberToActivityConverter::~mNumberToActivityConverter()
{}

void mNumberToActivityConverter::EvaluateStaticParameters()
{
  if (this->number_of_values.HasChanged())
  {
    while (this->input_signals.size() > this->number_of_values.Get())
    {
      this->input_signals.rbegin()->GetWrapped()->ManagedDelete();
      this->input_signals.pop_back();
      FINROC_LOG_PRINTF(DEBUG, "deleted input port\n");
    }
    while (this->output_signals.size() > this->number_of_values.Get())
    {
      this->output_signals.rbegin()->GetWrapped()->ManagedDelete();
      this->output_signals.pop_back();
      FINROC_LOG_PRINTF(DEBUG, "deleted output port\n");
    }

    int count = this->input_signals.size();

    FINROC_LOG_PRINTF(DEBUG, "creating %i input port(s)\n", this->number_of_values.Get() - count);

    while (this->input_signals.size() < this->number_of_values.Get())
    {
      this->input_signals.push_back(tInput<double>("Input signal " + rrlib::util::sStringUtils::StringOf(count), this));
      FINROC_LOG_PRINTF(DEBUG, "created input port %i\n", count);
      ++count;
    }

    count = this->output_signals.size();

    FINROC_LOG_PRINTF(DEBUG, "creating %i output port(s)", this->number_of_values.Get() - count);

    while (this->output_signals.size() < this->number_of_values.Get())
    {
      this->output_signals.push_back(tOutput<tActivity>("Output signal " + rrlib::util::sStringUtils::StringOf(count), this));
      FINROC_LOG_PRINTF(DEBUG, "created output port %i", count);
      ++count;
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
    for (unsigned int i = 0; i < this->number_of_values.Get(); ++i)
    {
      this->output_signals.at(i).Publish(this->input_signals.at(i).Get());
    }
  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
