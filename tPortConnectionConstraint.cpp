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
/*!\file    plugins/ib2c/tPortConnectionConstraint.cpp
 *
 * \author  Tobias Föhst
 *
 * \date    2012-09-06
 *
 */
//----------------------------------------------------------------------
#include "plugins/ib2c/tPortConnectionConstraint.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/ib2c/tMetaSignal.h"

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
const tPortConnectionConstraint cPORT_CONNECTION_CONSTRAINT;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tPortConnectionConstraint AllowPortConnection
//----------------------------------------------------------------------
bool tPortConnectionConstraint::AllowPortConnection(const core::tAbstractPort &source_port, const core::tAbstractPort &destination_port) const
{
  auto source_type =  source_port.GetWrapperDataType();
  auto destination_type = destination_port.GetWrapperDataType();

  if (destination_tye == source_type)
  {
    return true;
  }

  if (destination_type == rrlib::rtti::tDataType<tStimulation>() || destination_type == rrlib::rtti::tDataType<tInhibition>())
  {
    return source_type == rrlib::rtti::tDataType<tActivity>();
  }
  return true;
}

//----------------------------------------------------------------------
// tPortConnectionConstraint Description
//----------------------------------------------------------------------
const char *tPortConnectionConstraint::Description() const
{
  return "";
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
