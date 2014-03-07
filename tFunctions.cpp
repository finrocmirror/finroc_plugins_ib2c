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
/*!\file    plugins/ib2c/tFunctions.cpp
 *
 * \author  Jochen Hirth
 *
 * \date    2014-03-07
 *
 */
//----------------------------------------------------------------------
#include "plugins/ib2c/tFunctions.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
#include "rrlib/logging/messages.h"
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
// tFunctions constructors
//----------------------------------------------------------------------
tFunctions::tFunctions(tFunctionMode mode, tFunctionShape shape, double start1, double end1, double start2, double end2) :
  mode(mode),
  shape(shape),
  start_rise(0.),
  end_rise(0.),
  start_fall(0.),
  end_fall(0.)
{
  this->SetParameters(start1, end1, start2, end2);
}

//----------------------------------------------------------------------
// tFunctions destructor
//----------------------------------------------------------------------
tFunctions::~tFunctions()
{}

//----------------------------------------------------------------------
// tFunctions SomeExampleMethod
//----------------------------------------------------------------------
double tFunctions::Value(double input)
{
  switch (this->mode)
  {
  case eMODE_RISE:
    if (input <= this->start_rise)
      return 0.;
    else if (input >= this->end_rise)
      return 1.;
    else
      switch (this->shape)
      {
      case eSHP_SIGMOID:
        return (0.5 + 0.5 * sin(M_PI * ((input - this->start_rise) / (this->end_rise - this->start_rise) - 0.5)));
      case eSHP_LINEAR:
        return ((input - this->start_rise) / (this->end_rise - this->start_rise)) ;
      default:
        RRLIB_LOG_PRINTF(ERROR, "no shape set");
        return 0.;
      }
    break;
  case eMODE_FALL:
    if (input <= this->start_fall)
      return 1.;
    else if (input >= this->end_fall)
      return 0.;
    else
      switch (this->shape)
      {
      case eSHP_SIGMOID:
        return (0.5 - 0.5 * sin(M_PI * ((input - this->start_fall) / (this->end_fall - this->start_fall) - 0.5)));
      case eSHP_LINEAR:
        return (1. - (input - this->start_fall) / (this->end_fall - this->start_fall)) ;
      default:
        RRLIB_LOG_PRINTF(ERROR, "no shape set");
        return 0.;
      }

    break;
  case eMODE_RISE_FALL:
    if ((input >= this->start_rise) && (input < this->end_rise))
      switch (this->shape)
      {
      case eSHP_SIGMOID:
        return (0.5 + 0.5 * sin(M_PI * ((input - this->start_rise) / (this->end_rise - this->start_rise) - 0.5)));
      case eSHP_LINEAR:
        return ((input - this->start_rise) / (this->end_rise - this->start_rise)) ;
      default:
        RRLIB_LOG_PRINTF(ERROR, "no shape set");
        return 0.;
      }
    else if ((input >= this->end_rise) && (input < this->start_fall))
      return 1.;
    else if ((input >= this->start_fall) && (input < this->end_fall))
      switch (this->shape)
      {
      case eSHP_SIGMOID:
        return (0.5 - 0.5 * sin(M_PI * ((input - this->start_fall) / (this->end_fall - this->start_fall) - 0.5)));
      case eSHP_LINEAR:
        return (1. - (input - this->start_fall) / (this->end_fall - this->start_fall)) ;
      default:
        RRLIB_LOG_PRINTF(ERROR, "no shape set");
        return 0.;
      }
    else
      return 0.;
    break;
  case eMODE_FALL_RISE:
    if ((input >= this->start_fall) && (input < this->end_fall))
      switch (this->shape)
      {
      case eSHP_SIGMOID:
        return (0.5 - 0.5 * sin(M_PI * ((input - this->start_fall) / (this->end_fall - this->start_fall) - 0.5)));
      case eSHP_LINEAR:
        return (1. - (input - this->start_fall) / (this->end_fall - this->start_fall)) ;
      default:
        RRLIB_LOG_PRINTF(ERROR, "no shape set");
        return 0.;
      }

    else if ((input >= this->end_fall) && (input < this->start_rise))
      return 0.;
    else if ((input >= this->start_rise) && (input < this->end_rise))
      switch (this->shape)
      {
      case eSHP_SIGMOID:
        return (0.5 + 0.5 * sin(M_PI * ((input - this->start_rise) / (this->end_rise - this->start_rise) - 0.5)));
      case eSHP_LINEAR:
        return ((input - this->start_rise) / (this->end_rise - this->start_rise)) ;
      default:
        RRLIB_LOG_PRINTF(ERROR, "no shape set");
        return 0.;
      }
    else
      return 1.;
    break;
  default:
    RRLIB_LOG_PRINTF(ERROR, "no mode set");
    return 0.;
  }
}

void tFunctions::SetFunctionMode(tFunctionMode mode)
{
  this->mode = mode;
}

void tFunctions::SetFunctionShape(tFunctionShape shape)
{
  this->shape = shape;
}

void tFunctions::SetParameters(double start1, double end1, double start2, double end2)
{
  if (start1 > end1)
  {
    RRLIB_LOG_PRINTF(ERROR, "1st start point (%f) larger than 1st end point (%f)!", start1, end1);
    end1 = start1;
  }
  else if (start2 > end2)
  {
    RRLIB_LOG_PRINTF(ERROR, "2nd start point (%f) larger than 2nd end point (%f)!", start2, end2);
    end2 = start2;
  }
  else if ((end1 > start2) && (this->mode == eMODE_RISE_FALL || this->mode == eMODE_FALL_RISE))
  {
    RRLIB_LOG_PRINTF(ERROR, "1st end point (%f) larger than 2nd start point (%f)!", end1, start2);
    start2 = end1;
  }

  switch (this->mode)
  {
  case eMODE_RISE:
    this->start_rise = start1;
    this->end_rise = end1;
    break;
  case eMODE_FALL:
    this->start_fall = start1;
    this->end_fall = end1;
    break;
  case eMODE_RISE_FALL:
    this->start_rise = start1;
    this->end_rise = end1;
    this->start_fall = start2;
    this->end_fall = end2;
    break;
  case eMODE_FALL_RISE:
    this->start_fall = start1;
    this->end_fall = end1;
    this->start_rise = start2;
    this->end_rise = end2;
    break;
  default:
    RRLIB_LOG_PRINTF(ERROR, "unknown mode");
    break;
  }
}
//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
