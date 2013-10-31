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
/*!\file    plugins/ib2c/tGlobalService.h
 *
 * \author  Max Reichardt
 *
 * \date    2013-10-13
 *
 * \brief   Contains tGlobalService
 *
 * \b tGlobalService
 *
 * Global service for ib2c behaviors
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__ib2c__tGlobalService_h__
#define __plugins__ib2c__tGlobalService_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/rpc_ports/tServerPort.h"
#include "plugins/ib2c/tStatus.h"

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
// Class declaration
//----------------------------------------------------------------------
//! Global service for ib2c behaviors
/*!
 * Global service for ib2c behaviors
 */
class tGlobalService : public rpc_ports::tRPCInterface
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tGlobalService();

  /*!
   * Instantiates port for ib2c services
   */
  static void CreateGlobalServicePort();

  /**
   * Sets stimulation of behavior module with specified handle
   *
   * \param module_handle Runtime Handle of behavior module to modify
   * \param mode New stimulation mode
   */
  void SetStimulationMode(core::tFrameworkElement::tHandle module_handle, tStimulationMode mode);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
