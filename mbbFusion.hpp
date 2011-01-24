//
// You received this file as part of Finroc
// A framework for integrated robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
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
/*!\file    mbbFusion.cpp
 *
 * \author  Bernd-Helge Schaefer
 *
 * \date    2011-01-07
 *
 */
//----------------------------------------------------------------------

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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

namespace finroc
{
namespace ibbc
{

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
template <typename THead, typename ... TRest>
finroc::core::tStandardCreateModuleAction< mbbFusion <THead, TRest... > > mbbFusion <THead, TRest... >::cCREATE_ACTION("Fusion");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mbbFusion constructors
//----------------------------------------------------------------------
template <typename THead, typename ... TRest>
mbbFusion <THead, TRest... >::mbbFusion(finroc::core::tFrameworkElement *parent, const finroc::util::tString &name)
    : tBehaviourBasedModule(parent, name)
{

  // this->CreateOutputs (&value_2);
}

template <typename THead, typename ... TRest>
void mbbFusion <THead, TRest... >::Update()
{
  assert(this->input_activities.size() == this->input_target_ratings.size());

  this->CalculateBehaviourSignalInfo(this->behaviour_signal_info);

  tBehaviourBasedModule::Update();
}

//----------------------------------------------------------------------
// mbbFusion CalculateActivity
//----------------------------------------------------------------------
template <typename THead, typename ... TRest>
double mbbFusion <THead, TRest... >::CalculateActivity(std::vector <double>& derived_activities,
    double activation)
{
  double activity = 0.;
  switch (this->control_fusion_method)
  {
    // weighted sums:
  case tBehaviourDefinitions::eFUS_WEIGHT:
    if (this->behaviour_signal_info.sum_of_activity > 0.0)
    {
      activity = this->behaviour_signal_info.square_sum_of_activity / this->behaviour_signal_info.sum_of_activity * activation;
    }
    else
    {
      activity = 0.;
    }
    break;
    // weighted sum
  case tBehaviourDefinitions::eFUS_WEIGHTED_SUM:
    activity = 0.;
    if (this->behaviour_signal_info.max_a > 0.)
    {
      //     std::vector<int>::iterator it;
      //     for (it = this->start_indices_for_control_edges.begin(); it != start_indices_for_control_edges.end(); it++)
      for (int i = 0; i < this->behaviour_signal_info.number_of_values; ++i)
      {
        // similar to weighted sum of controller outputs: activity * (activity / max_activity)
        double input_activity = input_activities [i].GetDoubleRaw();
        assert(input_activity <= this->behaviour_signal_info.max_a);
        activity += input_activity * input_activity / this->behaviour_signal_info.max_a;
      }
      assert(activity >= 0.);
      assert(activity <= 1.);
      activity = activity * activation;
      //  activity = Limit(this->activity, 0.0, 1.0) * activation;
    }
    break;
    // maximum activity wins:
  case tBehaviourDefinitions::eFUS_MAX:
  default:
    //write out maximum value of activity:
    activity = 0.;
    if (this->behaviour_signal_info.max_a > 0.0)
    {
      activity = this->behaviour_signal_info.max_a * activation;
    }
  }

  //@todo: put into AssertIbbcPrinciples(bool strict = false)?
  // if (this->transfer_function_called && ((this->activity < this->min_activity_limit - DBL_EPSILON) || (this->activity > this->max_activity_limit + DBL_EPSILON)))
  // {
  //   char buffer[sStringUtils::max_chars+1];
  //   snprintf(buffer, sStringUtils::max_chars, "activity=%f, min activity limit=%f, max activity limit=%f", this->activity, this->min_activity_limit, this->max_activity_limit);
  //   tBehaviourBasis::ReportViolation("principle", "fusion behavior neutrality (activity)", buffer);
  // }

  return activity;
}

//----------------------------------------------------------------------
// mbbFusion CalculateTargetRating
//----------------------------------------------------------------------
template <typename THead, typename ... TRest>
double mbbFusion <THead, TRest... >::CalculateTargetRating()
{
  double target_rating = 0.5;
  switch (this->control_fusion_method)
  {
  case tBehaviourDefinitions::eFUS_WEIGHTED_SUM:
  case tBehaviourDefinitions::eFUS_WEIGHT:
    if (this->behaviour_signal_info.sum_of_activity > 0.0)
    {
      target_rating = this->behaviour_signal_info.sum_of_activity_times_target_rating / this->behaviour_signal_info.sum_of_activity;
    }
    else
    {
      if (this->behaviour_signal_info.number_of_values > 0.)
      {
        target_rating = this->behaviour_signal_info.sum_of_target_rating / this->behaviour_signal_info.number_of_values;
      }
      else
      {
        target_rating = 0.;
      }
    }
    break;
  case tBehaviourDefinitions::eFUS_MAX:
  default:
    target_rating = this->behaviour_signal_info.max_a_target_rating;
  }

  //@todo: put into AssertIbbcPrinciples(bool strict = false)?
  // if (this->transfer_function_called && ((this->target_rating < this->min_target_rating_limit - DBL_EPSILON) || (this->target_rating > this->max_target_rating_limit + DBL_EPSILON)))
  // {
  //   char buffer[sStringUtils::max_chars+1];
  //   snprintf(buffer, sStringUtils::max_chars, "target rating=%f, min target rating limit=%f, max target rating limit=%f", this->target_rating, this->min_target_rating_limit, this->max_target_rating_limit);
  //   tBehaviourBasis::ReportViolation("principle", "fusion behavior neutrality (target rating)", buffer);
  // }
  return target_rating;
}

//----------------------------------------------------------------------
// mbbFusion CalculateTransferFunction
//----------------------------------------------------------------------
template <typename THead, typename ... TRest>
void mbbFusion <THead, TRest... >::CalculateTransferFunction(double activation)
{
  switch (this->control_fusion_method)
  {
    //------------------------------------------------------------------------------------------------
    // A) Weighted (eFUS_WEIGHT):
    //------------------------------------------------------------------------------------------------
  case tBehaviourDefinitions::eFUS_WEIGHT:
    this->CalculateTransferFunctionWeight(activation, this->behaviour_signal_info);
    break;

    //------------------------------------------------------------------------------------------------
    // B) Weighted Sum (eFUS_WEIGHTED_SUM):
    //------------------------------------------------------------------------------------------------
  case tBehaviourDefinitions::eFUS_WEIGHTED_SUM:
    this->CalculateTransferFunctionWeightedSum(activation, this->behaviour_signal_info);
    break;

    //------------------------------------------------------------------------------------------------
    // C) Maximum activity wins (eFUS_MAX):
    //------------------------------------------------------------------------------------------------
  case tBehaviourDefinitions::eFUS_MAX:
  default:
    // find behaviour with highest activity:
    this->CalculateTransferFunctionMax(activation, this->behaviour_signal_info);
    break;
  };
}

} // end of namespace ibbc
} // end of namespace finroc
