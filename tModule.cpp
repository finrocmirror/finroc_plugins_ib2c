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
#include "plugins/scheduling/tPeriodicFrameworkElementTask.h"
#include "core/tFrameworkElementTags.h"

#include "rrlib/math/utilities.h"

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
const size_t cNUMBER_OF_CYCLES_WITH_SUPPRESSED_WARNINGS = 250;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tModule constructors
//----------------------------------------------------------------------
tModule::tModule(core::tFrameworkElement *parent, const std::string &name,
                 tStimulationMode stimulation_mode, unsigned int number_of_inhibition_ports,
                 bool share_ports) :
  tModuleBase(parent, name),

  meta_input(new core::tPortGroup(this, "iB2C Input", tFlag::INTERFACE, share_ports ? tFlags(tFlag::SHARED) : tFlags())),
  input(new core::tPortGroup(this, "Input", tFlag::INTERFACE, share_ports ? tFlags(tFlag::SHARED) : tFlags())),
  meta_output(new core::tPortGroup(this, "iB2C Output", tFlag::INTERFACE, share_ports ? tFlags(tFlag::SHARED) : tFlags())),
  output(new core::tPortGroup(this, "Output", tFlag::INTERFACE, share_ports ? tFlags(tFlag::SHARED) : tFlags())),

  number_of_inhibition_ports("Number Of Inhibition Ports", this),

  number_of_cycles_with_suppressed_warnings("Number Of Cycles With Suppressed Warnings", this, cNUMBER_OF_CYCLES_WITH_SUPPRESSED_WARNINGS),
  stimulation_mode("Stimulation Mode", this, stimulation_mode),

  stimulation("Stimulation", this),
  activity("Activity", this),
  target_rating("Target Rating", this),

  status("Status", this),

  update_task(this),
  input_changed(true),
  cycles_since_last_warning(-1),
  last_activation(0),
  last_activity(0),
  last_target_rating(0)
{
  std::vector<core::tEdgeAggregator *> input_ports = { this->meta_input, this->input };
  std::vector<core::tEdgeAggregator *> output_ports = { this->meta_output, this->output };
  this->AddAnnotation(*new scheduling::tPeriodicFrameworkElementTask(input_ports, output_ports, this->update_task));
  core::tFrameworkElementTags::AddTag(*this, "ib2c_module");

  this->number_of_inhibition_ports.Set(number_of_inhibition_ports);
}

//----------------------------------------------------------------------
// tModule destructor
//----------------------------------------------------------------------
tModule::~tModule()
{}

//----------------------------------------------------------------------
// tModule EvaluateStaticParameters
//----------------------------------------------------------------------
void tModule::OnStaticParameterChange()
{
  if (this->number_of_inhibition_ports.HasChanged())
  {
    while (this->inhibition.size() > this->number_of_inhibition_ports.Get())
    {
      this->inhibition.back().GetWrapped()->ManagedDelete();
      this->inhibition.pop_back();
    }
    unsigned int number_of_inhibition_ports = this->number_of_inhibition_ports.Get();
    for (size_t i = this->inhibition.size(); i < number_of_inhibition_ports; ++i)
    {
      this->AddInhibition("Inhibition " + std::to_string(i + 1));
    }
  }
}

//----------------------------------------------------------------------
// tModule EvaluateParameters
//----------------------------------------------------------------------
void tModule::OnParameterChange()
{
  if (this->stimulation_mode.HasChanged())
  {
    if (this->stimulation_mode.Get() == tStimulationMode::ENABLED)
    {
      this->stimulation.Set(1);
    }
    if (this->stimulation_mode.Get() == tStimulationMode::DISABLED)
    {
      this->stimulation.Set(0);
    }
  }
}

//----------------------------------------------------------------------
// tModule IsConnectedToOutputPort
//----------------------------------------------------------------------
bool tModule::IsConnectedToOutputPort(const core::tAbstractPort &port)
{
  for (auto it = port.IncomingConnectionsBegin(); it != port.IncomingConnectionsEnd(); ++it)
  {
    if (it->IsOutputPort() || it->GetAnnotation<parameters::internal::tParameterInfo>() || IsConnectedToOutputPort(*it))
    {
      return true;
    }
  }
  return false;
}

//----------------------------------------------------------------------
// tModule CalculateActivation
//----------------------------------------------------------------------
double tModule::CalculateActivation() const
{
  tStimulation stimulation = 0;
  tInhibition inhibition = this->CalculateInhibition();

  switch (this->stimulation_mode.Get())
  {
  case tStimulationMode::AUTO:
    if (!IsConnectedToOutputPort(this->stimulation))
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
    if (IsConnectedToOutputPort(this->stimulation))
    {
      if (this->WarnNow())
      {
        FINROC_LOG_PRINT(WARNING, "Stimulation mode is ENABLED but stimulation port also connected. Ignoring incoming value.");
      }
    }
    stimulation = 1;
    break;

  case tStimulationMode::DISABLED:
    if (IsConnectedToOutputPort(this->stimulation))
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
    if (IsConnectedToOutputPort(*it))
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
    FINROC_LOG_THROW(tViolation("Activity limitation: Activity = "  + std::to_string(activity) + " exceeds Activation = " + std::to_string(activation) + "!"));
  }
}

//----------------------------------------------------------------------
// tModule CheckGoalStateActivity
//----------------------------------------------------------------------
void tModule::CheckGoalStateActivity(tActivity activity, tTargetRating target_rating, double activation)
{
  if (rrlib::math::IsEqual(target_rating, 0) && rrlib::math::IsEqual(this->last_target_rating, 0) && rrlib::math::IsEqual(activation, this->last_activation))
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
    if (!rrlib::math::IsEqual(activity, this->last_activity) && !activity_transfer_inputs_changed)
    {
      FINROC_LOG_THROW(tViolation("Goal state activity: Target rating = 0 but Activity not constant!"));
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
      FINROC_LOG_THROW(tViolation("Derived activity \"" + this->derived_activity[i].GetName() + "\" out of bounds: " + std::to_string(derived_activities[i])));
    }

    if (derived_activities[i] > activity)
    {
      FINROC_LOG_THROW(tViolation("Derived activity \"" + this->derived_activity[i].GetName() + "\" = " + std::to_string(derived_activities[i]) + " exceeds Activity = " + std::to_string(activity)));
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
  this->module->cycles_since_last_warning++;

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

  // Publish status of behavior module
  data_ports::tPortDataPointer<tStatus> status_buffer = this->module->status.GetUnusedBuffer();
  status_buffer->name = this->module->GetName(); // since the name should stay the same, this should not allocate memory
  status_buffer->module_handle = this->module->GetHandle();
  status_buffer->stimulation_mode = this->module->stimulation_mode.Get();
  status_buffer->activity = activity;
  status_buffer->target_rating = target_rating;
  status_buffer->activation = activation;
  this->module->status.Publish(status_buffer);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
