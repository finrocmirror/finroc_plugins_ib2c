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
/*!\file    plugins/ib2c/tGlobalService.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2013-10-13
 *
 */
//----------------------------------------------------------------------
#include "plugins/ib2c/tGlobalService.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/tPlugin.h"
#include "core/tRuntimeEnvironment.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/ib2c/tModule.h"

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
namespace internal
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
static const char* cPORT_NAME = "ib2c";

static rpc_ports::tRPCInterfaceType<tGlobalService> cTYPE("ib2c Interface", &tGlobalService::SetStimulationMode);

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------
static tGlobalService global_service;

/*!
 * Plugin class for ib2c
 * (can also be moved to some other .cpp file - further initializations may be added)
 */
class tPlugin : public core::tPlugin
{
public:
  tPlugin() {}

  virtual void Init() // TODO mark override with gcc 4.7
  {
    /*! Port that receives ib2c requests */
    tGlobalService::CreateGlobalServicePort();
  }
};

static tPlugin plugin;

} // namespace internal

//----------------------------------------------------------------------
// tGlobalService constructors
//----------------------------------------------------------------------
tGlobalService::tGlobalService()
{}

void tGlobalService::CreateGlobalServicePort()
{
  rpc_ports::tServerPort<tGlobalService>(internal::global_service, internal::cPORT_NAME, internal::cTYPE, core::tFrameworkElement::tFlag::SHARED,
                                         &core::tRuntimeEnvironment::GetInstance().GetElement(core::tSpecialRuntimeElement::SERVICES));
}

void tGlobalService::SetStimulationMode(core::tFrameworkElement::tHandle module_handle, tStimulationMode mode)
{
  core::tFrameworkElement* element = core::tRuntimeEnvironment::GetInstance().GetElement(module_handle);
  tModule* module = dynamic_cast<tModule*>(element);
  if (!module)
  {
    FINROC_LOG_PRINT(WARNING, "No behavior module with handle ", module_handle, " found. Stimulation mode not changed.");
    return;
  }
  module->stimulation_mode.Set(mode);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
