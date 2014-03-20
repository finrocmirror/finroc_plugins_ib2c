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
/*!\file    plugins/ib2c/gbbEmptyGroup.h
 *
 * \author  Tobias Föhst
 *
 * \date    2013-07-11
 *
 * \brief Contains gbbEmptyGroup
 *
 * \b gbbEmptyGroup
 *
 * This is an iB2C group that can be fully configured from within finstruct
 * by creating its characteristic module online and registering it by name
 * in this group.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__ib2c__test__gbbEmptyGroup_h__
#define __plugins__ib2c__test__gbbEmptyGroup_h__

#include "plugins/ib2c/tGroup.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
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
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This is an iB2C group that can be fully configured from within finstruct
 * by creating its characteristic module online and registering it by name
 * in this group.
 */
class gbbEmptyGroup : public ib2c::tGroup
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tStaticParameter<std::string> characteristic_module;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  gbbEmptyGroup(core::tFrameworkElement *parent, const std::string &name = "EmptyGroup",
                tStimulationMode stimulation_mode = tStimulationMode::AUTO, unsigned int number_of_inhibition_ports = 0);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Destructor
   *
   * The destructor of groups is declared private to avoid accidental deletion. Deleting
   * groups is already handled by the framework.
   */
  ~gbbEmptyGroup();

  virtual void OnStaticParameterChange() override;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
