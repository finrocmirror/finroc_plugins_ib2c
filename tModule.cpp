//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
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
#include "core/thread/tPeriodicFrameworkElementTask.h"

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
tModule::tModule(core::tFrameworkElement *parent, const util::tString &name)
  : tModuleBase(parent, name),

    input(new core::tPortGroup(this, "Input", core::tEdgeAggregator::cIS_INTERFACE, core::tPortFlags::cINPUT_PORT)),
    output(new core::tPortGroup(this, "Output", core::tEdgeAggregator::cIS_INTERFACE, core::tPortFlags::cOUTPUT_PORT)),
    update_task(this),
    input_changed(true),
    last_activation(0)
{
  this->AddAnnotation(new core::tPeriodicFrameworkElementTask(this->input, this->output, &this->update_task));
}

//----------------------------------------------------------------------
// tModule CalculateActivation
//----------------------------------------------------------------------
double tModule::CalculateActivation() //const FIXME
{
  // FIXME: assertion not needed when using bounded ports
  assert(0 <= this->stimulation.Get() && this->stimulation.Get() <= 1 && "Stimulation out of bounds!");
  return this->stimulation.Get() * (1 - this->CalculateInhibition());
}

//----------------------------------------------------------------------
// tModule CalculateInhibition
//----------------------------------------------------------------------
double tModule::CalculateInhibition() //const FIXME
{
  double inhibition = 0;

  for (auto it = this->inhibition.begin(); it != this->inhibition.end(); ++it)
  {
    // FIXME: assertion not needed when using bounded ports
    assert(0 <= it->Get() && it->Get() <= 1 && "Inhibition out of bounds!");
    inhibition = std::max(inhibition, it->Get());
  }

  return inhibition;
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
  this->module->CheckParameters();
  this->module->input_changed = this->module->ProcessChangedFlags(*this->module->input);

  double activation = this->module->CalculateActivation();

  if (!(0 <= activation && activation <= 1))
  {
    std::stringstream message;
    message << "Activation out of bounds: " << activation;
    throw tViolation(message.str());
  }

  this->module->ProcessTransferFunction(activation);

  std::vector<double> derived_activities;
  double activity = this->module->CalculateActivity(derived_activities, activation);
  assert(derived_activities.size() == this->module->activity.size() && "You are not allowed to change the number of derived activities during calculation!");

  if (activity > activation)
  {
    std::stringstream message;
    message << "Activity limitation: Activity = " << activity << " exceeds Activation = " << activation << "!";
    throw tViolation(message.str());
  }

  for (size_t i = 0; i < derived_activities.size(); ++i)
  {
    if (!(0 <= derived_activities[i] && derived_activities[i] <= 1))
    {
      std::stringstream message;
      message << "Derived activity \"" << this->module->activity[i].GetName() << "\" out of bounds: " << derived_activities[i];
      throw tViolation(message.str());
    }

    if (derived_activities[i] > activity)
    {
      std::stringstream message;
      message << "Derived activity \"" << this->module->activity[i].GetName() << "\" = " << derived_activities[i] << " exceeds Activity = " << activity << "!";
      throw tViolation(message.str());
    }

    this->module->activity[i].Publish(derived_activities[i]);
  }

  this->module->target_rating.Publish(this->module->CalculateTargetRating());

  if (this->module->target_rating.Get() == 0)
  {
    if (activity != this->module->last_activation) // FIXME activity transfer inputs???
    {
      std::stringstream message;
      message << "Goal state activity: Target rating = 0 but Activity not constant!";
      throw tViolation(message.str());
    }
  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
