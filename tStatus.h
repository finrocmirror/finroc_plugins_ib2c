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
/*!\file    plugins/ib2c/tMetaSignal.h
 *
 * \author  Tobias Föhst
 *
 * \date    2013-10-10
 *
 * \brief Contains tStatus
 *
 * \b tStatus
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__ib2c__tStatus_h__
#define __plugins__ib2c__tStatus_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <iostream>

#include "rrlib/serialization/serialization.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/ib2c/tMetaSignal.h"

//----------------------------------------------------------------------
// Debugging
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
enum class tStimulationMode
{
  AUTO,
  ENABLED,
  DISABLED
};

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//!
/*!
 *
 */
struct tStatus
{
  std::string name;
  core::tFrameworkElement::tHandle module_handle;
  tStimulationMode stimulation_mode;
  tActivity activity;
  tTargetRating target_rating;
  double activation;

  tStatus() :
    name(),
    module_handle(0),
    stimulation_mode(tStimulationMode::AUTO),
    activity(0),
    target_rating(0),
    activation(0)
  {}
};

inline rrlib::serialization::tOutputStream &operator << (rrlib::serialization::tOutputStream &stream, const tStatus &status)
{
  stream << status.name << status.module_handle << status.stimulation_mode << status.activity << status.target_rating << status.activation;
  return stream;
}

inline rrlib::serialization::tInputStream &operator >> (rrlib::serialization::tInputStream &stream, tStatus &status)
{
  stream >> status.name >> status.module_handle >> status.stimulation_mode >> status.activity >> status.target_rating >> status.activation;
  return stream;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
