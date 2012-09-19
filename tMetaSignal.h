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
 * \date    2012-08-30
 *
 * \brief Contains tMetaSignal
 *
 * \b tMetaSignal
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__ib2c__tMetaSignal_h__
#define __plugins__ib2c__tMetaSignal_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <iostream>
#include <sstream>

#include "rrlib/serialization/serialization.h"

#include "core/portdatabase/typeutil.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/ib2c/tViolation.h"

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
enum tMetaSignalType
{
  eMETA_SIGNAL_TYPE_STIMULATION,
  eMETA_SIGNAL_TYPE_INHIBITION,
  eMETA_SIGNAL_TYPE_ACTIVITY,
  eMETA_SIGNAL_TYPE_TARGET_RATING
};

template <int Tmeta_signal_type>
class tMetaSignal;

typedef tMetaSignal<eMETA_SIGNAL_TYPE_STIMULATION> tStimulation;
typedef tMetaSignal<eMETA_SIGNAL_TYPE_INHIBITION> tInhibition;
typedef tMetaSignal<eMETA_SIGNAL_TYPE_ACTIVITY> tActivity;
typedef tMetaSignal<eMETA_SIGNAL_TYPE_TARGET_RATING> tTargetRating;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//!
/*!
 *
 */
template <int Tmeta_signal_type>
class tMetaSignal
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline tMetaSignal() :
    value(0)
  {}

  inline tMetaSignal(const tMetaSignal &other) :
    value(other.value)
  {}

  inline tMetaSignal(double value) :
    value(value)
  {
    if (0 > value || value > 1)
    {
      throw tViolation("Meta signal out of bounds!");
    }
  }

  inline operator double() const
  {
    return this->value;
  }

  inline tMetaSignal &operator += (const tMetaSignal &other)
  {
    this->value += other.value;
    return *this;
  }

  inline tMetaSignal &operator *= (double factor)
  {
    this->value *= factor;
    return *this;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  double value;

};

template <int Tmeta_signal_type>
inline bool operator < (const tMetaSignal<Tmeta_signal_type> &left, const tMetaSignal<Tmeta_signal_type> &right)
{
  return static_cast<double>(left) < static_cast<double>(right);
}

template <int Tmeta_signal_type>
inline std::ostream &operator << (std::ostream &stream, const tMetaSignal<Tmeta_signal_type> &meta_signal)
{
  stream << static_cast<double>(meta_signal);
  return stream;
}

template <int Tmeta_signal_type>
inline std::istream &operator >> (std::istream &stream, tMetaSignal<Tmeta_signal_type> &meta_signal)
{
  double meta_signal_value;
  stream >> meta_signal_value;
  meta_signal = meta_signal_value;
  return stream;
}

template <int Tmeta_signal_type>
inline rrlib::serialization::tOutputStream &operator << (rrlib::serialization::tOutputStream &stream, const tMetaSignal<Tmeta_signal_type> &meta_signal)
{
  stream << static_cast<double>(meta_signal);
  return stream;
}

template <int Tmeta_signal_type>
inline rrlib::serialization::tInputStream &operator >> (rrlib::serialization::tInputStream &stream, tMetaSignal<Tmeta_signal_type> &meta_signal)
{
  double meta_signal_value;
  stream >> meta_signal_value;
  meta_signal = meta_signal_value;
  return stream;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "plugins/ib2c/typeutil.h"

#endif
