//
// You received this file as part of Finroc
// A framework for intelligent robot control
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
/*!\file    plugins/ib2c/tModule.cpp
 *
 * \author  Bernd-Helge Schäfer
 * \author  Tobias Föhst
 *
 * \date    2010-12-31
 *
 */
//----------------------------------------------------------------------
#include "plugins/ib2c/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <boost/lexical_cast.hpp>

#include "core/thread/tPeriodicFrameworkElementTask.h"
#include "core/tFrameworkElementTags.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/ib2c/tViolation.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------

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

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tModule constructors
//----------------------------------------------------------------------
tModule::tModule(core::tFrameworkElement *parent, const util::tString &name) :
  tModuleBase(parent, name),

  meta_input(new core::tPortGroup(this, "iB2C Input", core::tEdgeAggregator::cIS_INTERFACE, core::tPortFlags::cINPUT_PORT)),
  input(new core::tPortGroup(this, "Input", core::tEdgeAggregator::cIS_INTERFACE, core::tPortFlags::cINPUT_PORT)),
  meta_output(new core::tPortGroup(this, "iB2C Output", core::tEdgeAggregator::cIS_INTERFACE, core::tPortFlags::cOUTPUT_PORT)),
  output(new core::tPortGroup(this, "Output", core::tEdgeAggregator::cIS_INTERFACE, core::tPortFlags::cOUTPUT_PORT)),

  stimulation_mode("Stimulation Mode", this),     // TODO: use port_name_generator for this block
  number_of_inhibition_ports("Number Of Inhibition Ports", this),
  warn_any_n_cycles("Warn Any N Cycles", this, 250),

  stimulation("Stimulation", this),               // TODO: use port_name_generator for this block
  activity("Activity", this),
  target_rating("Target Rating", this),
  activation("Activation", this),

  already_warned_this_cycle(false),
  warn_cycles(0),
  update_task(this),
  input_changed(true),
  last_activation(0),
  last_activity(0),
  last_target_rating(0)
{
  std::vector<core::tEdgeAggregator *> input_ports = { this->meta_input, this->input };
  std::vector<core::tEdgeAggregator *> output_ports = { this->meta_output, this->output };
  this->AddAnnotation(new core::tPeriodicFrameworkElementTask(input_ports, output_ports, this->update_task));
  core::tFrameworkElementTags::AddTag(*this, "ib2c_module");
}

//----------------------------------------------------------------------
// tModule EvaluateParameters
//----------------------------------------------------------------------
void tModule::EvaluateParameters()
{
  if (this->stimulation_mode.HasChanged())
  {
    if (this->stimulation_mode.Get() == tStimulationMode::ENABLED)
    {
      this->stimulation.Publish(1);
    }
    if (this->stimulation_mode.Get() == tStimulationMode::DISABLED)
    {
      this->stimulation.Publish(0);
    }
  }

  if (this->number_of_inhibition_ports.HasChanged())
  {
    while (this->inhibition.size() > this->number_of_inhibition_ports.Get())
    {
      this->inhibition.back().GetWrapped()->ManagedDelete();
      this->inhibition.pop_back();
    }
    for (size_t i = this->inhibition.size(); i < this->number_of_inhibition_ports.Get(); ++i)
    {
      this->RegisterInhibition("Inhibition " + boost::lexical_cast<std::string>(i + 1));
    }
  }
}

//----------------------------------------------------------------------
// tModule CalculateActivation
//----------------------------------------------------------------------
double tModule::CalculateActivation()
{
  tStimulation stimulation = 0;
  tInhibition inhibition = this->CalculateInhibition();

  switch (this->stimulation_mode.Get())
  {
  case tStimulationMode::AUTO:
    if (!this->stimulation.IsConnected())
    {
      if (this->WarnNow())
      {
        FINROC_LOG_PRINT(WARNING, "Stimulation mode is AUTO but stimulation port not connected. Activation is zero.");
      }
      return 0;
    }
    stimulation = this->stimulation.Get();
    break;

  case tStimulationMode::ENABLED:
    if (this->stimulation.IsConnected())
    {
      if (this->WarnNow())
      {
        FINROC_LOG_PRINT(WARNING, "Stimulation mode is ENABLED but stimulation port also connected. Ignoring incoming value.");
      }
    }
    stimulation = 1;
    break;

  case tStimulationMode::DISABLED:
    if (this->stimulation.IsConnected())
    {
      if (this->WarnNow())
      {
        FINROC_LOG_PRINT(WARNING, "Stimulation mode is DISABLED but stimulation port also connected. Ignoring incoming value.");
      }
    }
    stimulation = 0;
  }

  assert(0 <= this->stimulation.Get() && this->stimulation.Get() <= 1 && "Stimulation out of bounds!");
  return stimulation * (1 - inhibition);
}

//----------------------------------------------------------------------
// tModule CalculateInhibition
//----------------------------------------------------------------------
tInhibition tModule::CalculateInhibition() const
{
  tInhibition inhibition = 0;

  for (auto it = this->inhibition.begin(); it != this->inhibition.end(); ++it)
  {
    if (it->IsConnected())
    {
      inhibition = std::max(inhibition, it->Get());
    }
  }

  return inhibition;
}

//----------------------------------------------------------------------
// tModule CheckActivityLimitation
//----------------------------------------------------------------------
void tModule::CheckActivityLimitation(tActivity activity, double activation)
{
  if (activity > activation)
  {
    std::stringstream message;
    message << "Activity limitation: Activity = " << activity << " exceeds Activation = " << activation << "!";
    FINROC_LOG_PRINT(DEBUG_WARNING, tViolation(message.str()));
    throw tViolation(message.str());
  }
}

//----------------------------------------------------------------------
// tModule CheckGoalStateActivity
//----------------------------------------------------------------------
void tModule::CheckGoalStateActivity(tActivity activity, tTargetRating target_rating, double activation)
{
  if (target_rating == 0 && this->last_target_rating == 0 && activation != this->last_activation)
  {
    bool activity_transfer_inputs_changed = false;


    for (auto it = this->activity_transfer_inputs.begin(); it != this->activity_transfer_inputs.end(); ++it)
    {
      if (it->HasChanged())
      {
        activity_transfer_inputs_changed = true;
        break;
      }
    }

    if (activity != this->last_activity && !activity_transfer_inputs_changed)
    {
      std::stringstream message;
      message << "Goal state activity: Target rating = 0 but Activity not constant!";
      FINROC_LOG_PRINT(DEBUG_WARNING, tViolation(message.str()));
      throw tViolation(message.str());
    }
  }

  this->last_activation = activation;
  this->last_activity = activity;
  this->last_target_rating = target_rating;
}

//----------------------------------------------------------------------
// tModule CheckDerivedActivities
//----------------------------------------------------------------------
void tModule::CheckDerivedActivities(std::vector<tActivity> &derived_activities, tActivity activity)
{
  for (size_t i = 0; i < derived_activities.size(); ++i)
  {
    if (!(0 <= derived_activities[i] && derived_activities[i] <= 1))
    {
      std::stringstream message;
      message << "Derived activity \"" << this->derived_activity[i].GetName() << "\" out of bounds: " << derived_activities[i];
      FINROC_LOG_PRINT(DEBUG_WARNING, tViolation(message.str()));
      throw tViolation(message.str());
    }

    if (derived_activities[i] > activity)
    {
      std::stringstream message;
      message << "Derived activity \"" << this->derived_activity[i].GetName() << "\" = " << derived_activities[i] << " exceeds Activity = " << activity << "!";
      FINROC_LOG_PRINT(DEBUG_WARNING, tViolation(message.str()));
      throw tViolation(message.str());
    }
  }
}

//----------------------------------------------------------------------
// tModule::UpdateTask constructors
//----------------------------------------------------------------------
tModule::UpdateTask::UpdateTask(tModule *module)
  : module(module)
{}

//----------------------------------------------------------------------
// tModule::UpdateTask ExecuteTask
//----------------------------------------------------------------------
void tModule::UpdateTask::ExecuteTask()
{
  // reset warn flag; is set on call to WarnNow()
  this->module->SetWarningFlag(false);
  this->module->CheckParameters();
  this->module->input_changed = this->module->ProcessChangedFlags(*this->module->input);

  double activation = this->module->CalculateActivation();
  assert(0 <= activation && activation <= 1);

  if (!this->module->ProcessTransferFunction(activation))
  {
    if (this->module->WarnNow())
    {
      FINROC_LOG_PRINT(WARNING, "Could not process transfer function. Not updating meta signals.");
    }
    return;
  }

  std::vector<tActivity> derived_activities;
  tActivity activity = this->module->CalculateActivity(derived_activities, activation);
  assert(derived_activities.size() == this->module->derived_activity.size() && "You are not allowed to change the number of derived activities during calculation!");

  tTargetRating target_rating = this->module->CalculateTargetRating(activation);

  this->module->CheckActivityLimitation(activity, activation);
  this->module->CheckGoalStateActivity(activity, target_rating, activation);
  this->module->CheckDerivedActivities(derived_activities, activity);

  this->module->activity.Publish(activity);
  this->module->target_rating.Publish(target_rating);

  for (size_t i = 0; i < derived_activities.size(); ++i)
  {
    this->module->derived_activity[i].Publish(derived_activities[i]);
  }

  this->module->activation.Publish(activation);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
