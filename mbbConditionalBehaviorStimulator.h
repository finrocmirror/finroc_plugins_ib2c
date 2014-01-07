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
/*!\file    plugins/ib2c/mbbConditionalBehaviorStimulator.h
 *
 * \author  Tobias Föhst
 *
 * \date    2012-06-20
 *
 * \brief Contains mbbConditionalBehaviorStimulator
 *
 * \b mbbConditionalBehaviorStimulator
 *
 * This class implements the iB2C Conditional Behavior Stimulator (CBS)
 * that allows to easily create behavior activity sequences.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__ib2c__mbbConditionalBehaviorStimulator_h__
#define __plugins__ib2c__mbbConditionalBehaviorStimulator_h__

#include "plugins/ib2c/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/math/utilities.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/ib2c/tCondition.h"

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
 * This class implements the iB2C Conditional Behavior Stimulator (CBS)
 * that allows to easily create behavior activity sequences.
 */
class mbbConditionalBehaviorStimulator : public ib2c::tModule
{
  static runtime_construction::tStandardCreateModuleAction<mbbConditionalBehaviorStimulator> cCREATE_ACTION;

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tStaticParameter<unsigned int> number_of_input_conditions;
  tStaticParameter<unsigned int> number_of_feedback_conditions;

  tInput<unsigned int> reset;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mbbConditionalBehaviorStimulator(core::tFrameworkElement *parent, const std::string &name = "ConditionalBehaviorStimulator",
                                   tStimulationMode stimulation_mode = tStimulationMode::AUTO, unsigned int number_of_inhibition_ports = 0);

  inline const tInput<double> &AddInputCondition(tConditionType type, tConditionRelation relation, double threshold, const std::string &name = "")
  {
    this->input_conditions.push_back(tCondition(this, name != "" ? name : ("Input Condition " + std::to_string(this->input_conditions.size() + 1)), type, relation, threshold));
    return this->input_conditions.back().Input();
  }

  inline const tInput<double> &AddFeedbackCondition(tConditionType type, tConditionRelation relation, double threshold, const std::string &name = "")
  {
    this->feedback_conditions.push_back(tCondition(this, name != "" ? name : ("Feedback Condition " + std::to_string(this->feedback_conditions.size() + 1)), type, relation, threshold));
    return this->feedback_conditions.back().Input();
  }

  inline void RegisterInputBehavior(tModule &input_behavior, tConditionType type, tConditionRelation relation, double threshold, const std::string &name = "")
  {
    input_behavior.activity.ConnectTo(this->AddFeedbackCondition(type, relation, threshold, name != "" ? name : input_behavior.GetName()));
  }

  inline void RegisterFeedbackBehavior(tModule &feedback_behavior, tConditionType type, tConditionRelation relation, double threshold, const std::string &name = "")
  {
    feedback_behavior.target_rating.ConnectTo(this->AddFeedbackCondition(type, relation, threshold, name != "" ? name : feedback_behavior.GetName()));
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  unsigned int reset_requests;

  std::vector<tCondition> input_conditions;
  std::vector<tCondition> feedback_conditions;

  bool all_input_conditions_fulfilled;
  bool all_feedback_conditions_fulfilled;

  bool EvaluateConditions(std::vector<tCondition> &conditions);

  void Reset();

  void AdjustConditionList(std::vector<tCondition> &condition_list, size_t size, const std::string &name_prefix);

  virtual void OnStaticParameterChange();

  virtual bool ProcessTransferFunction(double activation);

  virtual tActivity CalculateActivity(std::vector<tActivity> &derived_activities, double activity) const;

  virtual tTargetRating CalculateTargetRating(double activation) const;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
