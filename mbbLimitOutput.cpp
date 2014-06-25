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
/*!\file    plugins/ib2c/mbbLimitOutput.cpp
 *
 * \author  Jochen Hirth
 *
 * \date    2012-10-30
 *
 */
//----------------------------------------------------------------------
#include "plugins/ib2c/mbbLimitOutput.h"

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
runtime_construction::tStandardCreateModuleAction<mbbLimitOutput> cCREATE_ACTION_FOR_MBB_LIMIT_OUTPUT("LimitOutput");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mbbLimitOutput constructor
//----------------------------------------------------------------------
mbbLimitOutput::mbbLimitOutput(core::tFrameworkElement *parent, const std::string &name,
                               tStimulationMode stimulation_mode, unsigned int number_of_inhibition_ports) :
  tModule(parent, name, stimulation_mode, number_of_inhibition_ports)
{}

//----------------------------------------------------------------------
// mbbLimitOutput destructor
//----------------------------------------------------------------------
mbbLimitOutput::~mbbLimitOutput()
{}

//----------------------------------------------------------------------
// mbbLimitOutput EvaluateStaticParameters
//----------------------------------------------------------------------
void mbbLimitOutput::OnStaticParameterChange()
{
  if (this->number_of_signals.HasChanged())
  {
    FINROC_LOG_PRINTF(DEBUG, "number of signals has changed\n");

    while (this->input_signals.size() > this->number_of_signals.Get())
    {
      this->input_signals.rbegin()->GetWrapped()->ManagedDelete();
      this->input_signals.pop_back();
      FINROC_LOG_PRINTF(DEBUG, "deleted input port");
    }
    while (this->output_signals.size() > this->number_of_signals.Get())
    {
      this->output_signals.rbegin()->GetWrapped()->ManagedDelete();
      this->output_signals.pop_back();
      FINROC_LOG_PRINTF(DEBUG, "deleted output port");
    }

    int count = this->input_signals.size();

    FINROC_LOG_PRINTF(DEBUG, "creating %i input port(s)", this->number_of_signals.Get() - count);

    while (this->input_signals.size() < this->number_of_signals.Get())
    {
      this->input_signals.push_back(tInput<double>("Input signal" + std::to_string(count), this));
      FINROC_LOG_PRINTF(DEBUG, "created input port %i", count);
      ++count;
    }

    count = this->output_signals.size();

    FINROC_LOG_PRINTF(DEBUG, "creating %i output port(s)", this->number_of_signals.Get() - count);

    while (this->output_signals.size() < this->number_of_signals.Get())
    {
      this->output_signals.push_back(tOutput<double>("Output signal" + std::to_string(count), this));
      FINROC_LOG_PRINTF(DEBUG, "created output port %i", count);
      ++count;
    }

    FINROC_LOG_PRINTF(DEBUG, "size of input signals: %zu, size of output signals: %zu", this->input_signals.size(), this->output_signals.size());
  }
}


//----------------------------------------------------------------------
// mbbLimitOutput ProcessTransferFunction
//----------------------------------------------------------------------
bool mbbLimitOutput::ProcessTransferFunction(double activation)
{
  if (activation > 0)
  {
    for (unsigned int i = 0; i < this->number_of_signals.Get(); ++i)
    {
      this->output_signals.at(i).Publish(this->input_signals.at(i).Get());
    }
  }
  else
  {
    for (unsigned int i = 0; i < this->number_of_signals.Get(); ++i)
    {
      this->output_signals.at(i).Publish(0);
    }
  }

  return true;
}

//----------------------------------------------------------------------
// mbbLimitOutput CalculateActivity
//----------------------------------------------------------------------
ib2c::tActivity mbbLimitOutput::CalculateActivity(std::vector<ib2c::tActivity> &derived_activity, double activation) const
{
  return tActivity(activation);
}

//----------------------------------------------------------------------
// mbbLimitOutput CalculateTargetRating
//----------------------------------------------------------------------
ib2c::tTargetRating mbbLimitOutput::CalculateTargetRating(double activation) const
{
  return tTargetRating(activation);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
