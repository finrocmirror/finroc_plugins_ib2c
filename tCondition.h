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
/*!\file    plugins/ib2c/tCondition.h
 *
 * \author  Tobias Föhst
 *
 * \date    2012-06-20
 *
 * \brief Contains tCondition
 *
 * \b tCondition
 *
 * This class implements the conditions for the iB2C Conditional Behavior
 * Stimulator (CBS) that allows to easily create behavior activity sequences.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__ib2c__tCondition_h__
#define __plugins__ib2c__tCondition_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/math/utilities.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/ib2c/tModule.h"

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
enum class tConditionType
{
  PERMANENT,
  ORDERING,
  ENABLING
};

enum class tConditionRelation
{
  GREATER_THAN_THRESHOLD,
  GREATER_OR_EQUAL_TO_THRESHOLD,
  EQUAL_TO_THRESHOLD,
  SMALLER_OR_EQUAL_TO_THRESHOLD,
  SMALLER_THAN_THRESHOLD,
  UNEQUAL_TO_THRESHOLD
};

class mbbConditionalBehaviorStimulator;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This class implements the conditions for the iB2C Conditional Behavior
 * Stimulator (CBS) that allows to easily create behavior activity sequences.
 */
class tCondition
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tCondition(mbbConditionalBehaviorStimulator *parent, const std::string &name,
             tConditionType type = tConditionType::PERMANENT, tConditionRelation relation = tConditionRelation::GREATER_THAN_THRESHOLD, double threshold = 0) :
    type(name + " Type", parent, type),
    relation(name + " Relation", parent, relation),
    threshold(name + " Threshold", parent, threshold),
    input(name + " Value", parent),

    fulfilled(false)
  {}

  inline const char *GetLogDescription()
  {
    return "tCondition";
  }

  inline tConditionType Type() const
  {
    return this->type.Get();
  }

  inline tModule::tInput<double> &Input()
  {
    return this->input;
  }

  inline bool Evaluate()
  {
    if (this->fulfilled && this->type.Get() != tConditionType::PERMANENT)
    {
      return true;
    }

    this->fulfilled = this->CheckThreshold();

    return this->fulfilled;
  }

  inline void Reset()
  {
    this->fulfilled = false;
  }

  inline void ManagedDelete()
  {
    this->input.GetWrapped()->ManagedDelete();
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  tModule::tParameter<tConditionType> type;
  tModule::tParameter<tConditionRelation> relation;
  tModule::tParameter<double> threshold;
  tModule::tInput<double> input;

  bool fulfilled;

  bool CheckThreshold()
  {
    switch (this->relation.Get())
    {
    case tConditionRelation::GREATER_THAN_THRESHOLD:
      return this->input.Get() > this->threshold.Get();
    case tConditionRelation::GREATER_OR_EQUAL_TO_THRESHOLD:
      return this->input.Get() >= this->threshold.Get();
    case tConditionRelation::EQUAL_TO_THRESHOLD:
      return rrlib::math::IsEqual(this->input.Get(), this->threshold.Get());
    case tConditionRelation::SMALLER_OR_EQUAL_TO_THRESHOLD:
      return this->input.Get() <= this->threshold.Get();
    case tConditionRelation::SMALLER_THAN_THRESHOLD:
      return this->input.Get() < this->threshold.Get();
    case tConditionRelation::UNEQUAL_TO_THRESHOLD:
      return this->input.Get() != this->threshold.Get();
    }

    FINROC_LOG_PRINT(ERROR, "Unhandled relation: ", make_builder::GetEnumString(this->relation.Get()));
    return false;
  }

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
