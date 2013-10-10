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
  tStimulationMode stimulation_mode;
  tActivity activity;
  tTargetRating target_rating;
  double activation;
};

inline std::ostream &operator << (std::ostream &stream, const tStatus &status)
{
  stream << static_cast<uint8_t>(status.stimulation_mode) << status.activity << status.target_rating << status.activation;
  return stream;
}

inline std::istream &operator >> (std::istream &stream, tStatus &status)
{
  uint8_t stimulation_mode;
  stream >> stimulation_mode >> status.activity >> status.target_rating >> status.activation;
  status.stimulation_mode = static_cast<tStimulationMode>(stimulation_mode);
  return stream;
}

inline rrlib::serialization::tOutputStream &operator << (rrlib::serialization::tOutputStream &stream, const tStatus &status)
{
  stream << static_cast<uint8_t>(status.stimulation_mode) << status.activity << status.target_rating << status.activation;
  return stream;
}

inline rrlib::serialization::tInputStream &operator >> (rrlib::serialization::tInputStream &stream, tStatus &status)
{
  uint8_t stimulation_mode;
  stream >> stimulation_mode >> status.activity >> status.target_rating >> status.activation;
  status.stimulation_mode = static_cast<tStimulationMode>(stimulation_mode);
  return stream;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
