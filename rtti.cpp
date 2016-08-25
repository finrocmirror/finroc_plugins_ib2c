//
// You received this file as part of RRLib
// Robotics Research Library
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
/*!\file    rrlib/localization/rtti.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2016-08-25
 *
 * \brief   Contains Runtime Type Information initializations
 *
 */
//----------------------------------------------------------------------

#ifdef _LIB_RRLIB_RTTI_PRESENT_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/rtti/rtti.h"

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
using namespace rrlib::rtti;
using namespace finroc::ib2c;

//----------------------------------------------------------------------
// Type initializers
//----------------------------------------------------------------------

static tDataType<tStimulation> cINIT_TYPE_STIMULATION("finroc.ib2c.Stimulation");
static tDataType<tInhibition> cINIT_TYPE_INHIBITION("finroc.ib2c.Inhibition");
static tDataType<tActivity> cINIT_TYPE_ACTIVITY("finroc.ib2c.Activity");
static tDataType<tTargetRating>cINIT_TYPE_TARGETRATING("finroc.ib2c.TargetRating");

#endif

