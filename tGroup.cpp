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
/*!\file    plugins/ib2c/tGroup.cpp
 *
 * \author  Tobias Föhst
 *
 * \date    2012-12-07
 *
 */
//----------------------------------------------------------------------
#include "plugins/ib2c/tGroup.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/tFrameworkElementTags.h"
#include "plugins/runtime_construction/tEditableInterfaces.h"

#include "rrlib/util/tTraceableException.h"

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
typedef runtime_construction::tEditableInterfaces::tStaticInterfaceInfo tStaticInterfaceInfo;
typedef core::tFrameworkElement::tFlag tFlag;

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
const size_t cNUMBER_OF_CYCLES_WITH_SUPPRESSED_WARNINGS = 250;
const std::vector<tStaticInterfaceInfo>& cSTATIC_INTERFACE_INFO_GROUP =
{
  tStaticInterfaceInfo { "Input", tFlag::INTERFACE, tFlag::EMITS_DATA | tFlag::ACCEPTS_DATA | tFlag::PUSH_STRATEGY, runtime_construction::tPortCreateOption::SHARED },
  tStaticInterfaceInfo { "Output", tFlag::INTERFACE, tFlag::EMITS_DATA | tFlag::ACCEPTS_DATA | tFlag::OUTPUT_PORT | tFlag::PUSH_STRATEGY, runtime_construction::tPortCreateOption::SHARED }
};

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tGroup constructors
//----------------------------------------------------------------------
tGroup::tGroup(core::tFrameworkElement *parent, const std::string &name,
               tStimulationMode stimulation_mode, unsigned int number_of_inhibition_ports,
               const std::string &structure_config_file,
               bool share_ports, tFlags extra_flags) :
  tCompositeComponent(parent, name, structure_config_file, extra_flags),

  meta_input(new core::tPortGroup(this, "iB2C Input", tFlag::INTERFACE, tFlag::PUSH_STRATEGY | (share_ports ? tFlags(tFlag::SHARED) : tFlags()))),
  meta_output(new core::tPortGroup(this, "iB2C Output", tFlag::INTERFACE, tFlag::PUSH_STRATEGY | (share_ports ? tFlags(tFlag::SHARED) : tFlags()))),

  number_of_inhibition_ports("Number Of Inhibition Ports", this),

  number_of_cycles_with_suppressed_warnings("Number Of Cycles With Suppressed Warnings", this, cNUMBER_OF_CYCLES_WITH_SUPPRESSED_WARNINGS),
  stimulation_mode("Stimulation Mode", this, stimulation_mode),

  stimulation("Stimulation", this),
  activity("Activity", this),
  target_rating("Target Rating", this),

  status("Status", this),

  characteristic_module(NULL)
{
  core::tFrameworkElementTags::AddTag(*this, "ib2c_group");

  this->interfaces.fill(NULL);
  this->EmplaceAnnotation<runtime_construction::tEditableInterfaces>(cSTATIC_INTERFACE_INFO_GROUP, this->interfaces.begin(), share_ports | (share_ports << 1));

  this->number_of_inhibition_ports.Set(number_of_inhibition_ports);
}

//----------------------------------------------------------------------
// tGroup GetInterface
//----------------------------------------------------------------------
core::tPortGroup& tGroup::GetInterface(tInterfaceEnumeration interface)
{
  if (!this->interfaces[interface])
  {
    runtime_construction::tEditableInterfaces *editable_interfaces = this->GetAnnotation<runtime_construction::tEditableInterfaces>();
    editable_interfaces->CreateInterface(this, interface, IsReady());
  }
  return *this->interfaces[interface];
}

core::tPortGroup& tGroup::GetInterface(const std::string &interface_name)
{
  for (size_t i = 0; i < cSTATIC_INTERFACE_INFO_GROUP.size(); ++i)
  {
    if (interface_name.compare(cSTATIC_INTERFACE_INFO_GROUP[i].name) == 0)
    {
      return GetInterface(static_cast<tInterfaceEnumeration>(i));
    }
  }
  throw rrlib::util::tTraceableException<std::runtime_error>("No interface with name '" + interface_name + "' is meant to be added to a group.");
}

//----------------------------------------------------------------------
// tGroup EvaluateStaticParameters
//----------------------------------------------------------------------
void tGroup::OnStaticParameterChange()
{
  tCompositeComponent::OnStaticParameterChange();

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

    if (this->characteristic_module)
    {
      this->characteristic_module->OnStaticParameterChange();
      this->ConnectCharacteristicModule();
    }
  }
}

//----------------------------------------------------------------------
// tGroup RegisterCharacteristicModule
//----------------------------------------------------------------------
void tGroup::RegisterCharacteristicModule(tModule *module)
{
  if (this->characteristic_module)
  {
    std::runtime_error("This group already has a characteristic module");
  }

  this->characteristic_module = module;
  this->characteristic_module->number_of_inhibition_ports.AttachTo(this->number_of_inhibition_ports);
  this->OnStaticParameterChange();
  this->ConnectCharacteristicModule();
}

//----------------------------------------------------------------------
// tGroup UpdateConnectionsToCharacteristicModule
//----------------------------------------------------------------------
void tGroup::ConnectCharacteristicModule()
{
  assert(this->characteristic_module);

  this->number_of_cycles_with_suppressed_warnings.AttachTo(this->characteristic_module->number_of_cycles_with_suppressed_warnings);
  this->stimulation_mode.AttachTo(this->characteristic_module->stimulation_mode);
  this->characteristic_module->OnParameterChange();

  this->stimulation.ConnectTo(this->characteristic_module->stimulation);

  assert(this->inhibition.size() == this->characteristic_module->inhibition.size());
  assert(this->inhibition.size() == this->number_of_inhibition_ports.Get());
  for (size_t i = 0; i < this->characteristic_module->inhibition.size(); ++i)
  {
    this->inhibition[i].ConnectTo(this->characteristic_module->inhibition[i]);
  }

  this->characteristic_module->activity.ConnectTo(this->activity);
  this->characteristic_module->target_rating.ConnectTo(this->target_rating);

  this->characteristic_module->status.ConnectTo(this->status);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
