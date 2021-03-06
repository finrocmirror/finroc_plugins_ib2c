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
/*!\file    plugins/ib2c/tCondition.cpp
 *
 * \author  Tobias Föhst
 * \author  Max Reichardt
 *
 * \date    2014-01-14
 *
 */
//----------------------------------------------------------------------
#include "plugins/ib2c/tCondition.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/ib2c/mbbConditionalBehaviorStimulator.h"

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
// tCondition constructors
//----------------------------------------------------------------------
tCondition::tCondition(mbbConditionalBehaviorStimulator *parent, const std::string &name,
                       tConditionType type, tConditionRelation relation, double threshold) :
  type(name + " Type", parent, type),
  relation(name + " Relation", parent, relation),
  threshold(name + " Threshold", parent, threshold),
  input(name + " Value", parent),

  fulfilled(false)
{}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
