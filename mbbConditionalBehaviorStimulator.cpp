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
/*!\file    plugins/ib2c/mbbConditionalBehaviorStimulator.cpp
 *
 * \author  Tobias Föhst
 *
 * \date    2012-06-20
 *
 */
//----------------------------------------------------------------------
#include "plugins/ib2c/mbbConditionalBehaviorStimulator.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <boost/lexical_cast.hpp>

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
core::tStandardCreateModuleAction<mbbConditionalBehaviorStimulator> mbbConditionalBehaviorStimulator::cCREATE_ACTION("ConditionalBehaviorStimulator");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mbbConditionalBehaviorStimulator constructor
//----------------------------------------------------------------------
mbbConditionalBehaviorStimulator::mbbConditionalBehaviorStimulator(core::tFrameworkElement *parent, const util::tString &name) :
  ib2c::tModule(parent, name, "(CBS) "),

  reset_requests(0),
  all_input_conditions_fulfilled(false)
{
  core::tFrameworkElementTags::AddTag(*this, "ib2c_cbs");
}

//----------------------------------------------------------------------
// mbbConditionalBehaviorStimulator EvaluateConditions
//----------------------------------------------------------------------
bool mbbConditionalBehaviorStimulator::EvaluateConditions(std::vector<tCondition> &conditions)
{
  bool all_conditions_fulfilled = true;

  // Process the ordering and permanent conditions:
  for (auto it = conditions.begin(); it != conditions.end(); ++it)
  {
    switch (it->Type())
    {
    case tConditionType::PERMANENT:
    case tConditionType::ORDERING:
      if (!it->Evaluate())
      {
        all_conditions_fulfilled = false;
      }
      break;
    default:
      ; // Enabling conditions are not evaluated in this step
    }
  }

  // Process the enabling conditions:
  if (all_conditions_fulfilled)
  {
    for (auto it = conditions.begin(); it != conditions.end(); ++it)
    {
      if (it->Type() == tConditionType::ENABLING)
      {
        if (!it->Evaluate())
        {
          all_conditions_fulfilled = false;
          break;
        }
      }
    }
  }

  if (!all_conditions_fulfilled)
  {
    for (auto it = conditions.begin(); it != conditions.end(); ++it)
    {
      if (it->Type() == tConditionType::ENABLING)
      {
        it->Reset();
      }
    }
  }

  return all_conditions_fulfilled;
}

//----------------------------------------------------------------------
// mbbConditionalBehaviorStimulator Reset
//----------------------------------------------------------------------
void mbbConditionalBehaviorStimulator::Reset()
{
  for (auto it = this->input_conditions.begin(); it != this->input_conditions.end(); ++it)
  {
    it->Reset();
  }

  for (auto it = this->feedback_conditions.begin(); it != this->feedback_conditions.end(); ++it)
  {
    it->Reset();
  }

  this->all_input_conditions_fulfilled = false;
}

//----------------------------------------------------------------------
// mbbConditionalBehaviorStimulator AdjustConditionList
//----------------------------------------------------------------------
void mbbConditionalBehaviorStimulator::AdjustConditionList(std::vector<tCondition> &condition_list, size_t size, const std::string &name_prefix)
{
  while (condition_list.size() > size)
  {
    condition_list.back().ManagedDelete();
    condition_list.pop_back();
  }
  for (size_t i = condition_list.size(); i < size; ++i)
  {
    condition_list.push_back(tCondition(this, name_prefix + boost::lexical_cast<std::string>(i + 1)));
  }
}

//----------------------------------------------------------------------
// mbbConditionalBehaviorStimulator EvaluateStaticParameters
//----------------------------------------------------------------------
void mbbConditionalBehaviorStimulator::EvaluateStaticParameters()
{
  tModule::EvaluateStaticParameters();

  if (this->number_of_input_conditions.HasChanged())
  {
    this->AdjustConditionList(this->input_conditions, this->number_of_input_conditions.Get(), "Input Condition ");
  }
  if (this->number_of_feedback_conditions.HasChanged())
  {
    this->AdjustConditionList(this->feedback_conditions, this->number_of_feedback_conditions.Get(), "Feedback Condition ");
  }
}

//----------------------------------------------------------------------
// mbbConditionalBehaviorStimulator ProcessTransferFunction
//----------------------------------------------------------------------
bool mbbConditionalBehaviorStimulator::ProcessTransferFunction(double activation)
{
  if (this->reset_requests != this->reset.Get())
  {
    this->Reset();
    this->reset_requests = this->reset.Get();
  }

  this->all_input_conditions_fulfilled = this->EvaluateConditions(this->input_conditions);

  if (this->all_input_conditions_fulfilled)
  {
    FINROC_LOG_PRINT(DEBUG_VERBOSE_2, "All input conditions have been fulfilled.");
    FINROC_LOG_PRINT(DEBUG_VERBOSE_2, "Registered feedback conditions: ", this->feedback_conditions.size());
    if (!this->feedback_conditions.empty() && this->EvaluateConditions(this->feedback_conditions))
    {
      FINROC_LOG_PRINT(DEBUG_VERBOSE_2, "All feedback conditions have been fulfilled. Resetting ...");
      this->Reset();
    }
  }

  return true;
}

//----------------------------------------------------------------------
// mbbConditionalBehaviorStimulator CalculateActivity
//----------------------------------------------------------------------
tActivity mbbConditionalBehaviorStimulator::CalculateActivity(std::vector<tActivity> &derived_activity, double activation) const
{
  return this->all_input_conditions_fulfilled ? activation : 0.0;
}

//----------------------------------------------------------------------
// mbbConditionalBehaviorStimulator CalculateTargetRating
//----------------------------------------------------------------------
tTargetRating mbbConditionalBehaviorStimulator::CalculateTargetRating(double activation) const
{
  return this->all_input_conditions_fulfilled;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
