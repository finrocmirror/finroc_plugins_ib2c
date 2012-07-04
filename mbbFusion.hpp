//
// You received this file as part of Finroc
// A framework for intelligent robot control
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
 * \author  Tobias Föhst
 *
 * \date    2011-01-07
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/data_fusion/functions.h"

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
namespace ib2c
{

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
template <typename ... TSignalTypes>
finroc::core::tStandardCreateModuleAction<mbbFusion<TSignalTypes...>> mbbFusion<TSignalTypes...>::cCREATE_ACTION("Fusion");

const unsigned int cMAX_NUMBER_OF_INPUT_MODULES = 1000;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mbbFusion constructors
//----------------------------------------------------------------------
template <typename ... TSignalTypes>
mbbFusion<TSignalTypes...>::mbbFusion(finroc::core::tFrameworkElement *parent, const finroc::util::tString &name)
  : tModule(parent, name),

    number_of_input_modules(2, core::tBounds<unsigned int>(2, cMAX_NUMBER_OF_INPUT_MODULES, false)),

    output(this, "Output "),

    max_input_activity_index(0),
    max_input_activity(0),
    sum_of_input_activities(0),
    min_input_target_rating(1),
    max_input_target_rating(0)
{}

//----------------------------------------------------------------------
// mbbFusion ProcessTransferFunction
//----------------------------------------------------------------------
template <typename ... TSignalTypes>
void mbbFusion<TSignalTypes...>::ParametersChanged()
{
  if (this->number_of_input_modules.HasChanged())
  {
    while (this->input.size() > this->number_of_input_modules.Get())
    {
      this->input.back().ManagedDelete();
      this->input.pop_back();
    }
    for (size_t i = this->input.size(); i < this->number_of_input_modules.Get(); ++i)
    {
      this->input.push_back(tChannel(this, i));
    }
  }
}

//----------------------------------------------------------------------
// mbbFusion ProcessTransferFunction
//----------------------------------------------------------------------
template <typename ... TSignalTypes>
bool mbbFusion<TSignalTypes...>::ProcessTransferFunction(double activation)
{
  this->max_input_activity_index = 0;
  this->max_input_activity = 0;
  this->sum_of_input_activities = 0;
  this->min_input_target_rating = 1;
  this->max_input_target_rating = 0;

  for (size_t i = 0; i < this->input.size(); ++i)
  {
    if (!this->input[i].activity.IsConnected())
    {
      FINROC_LOG_PRINT(rrlib::logging::eLL_ERROR, this->input[i].activity.GetName(), " is not connected.");
      return false;
    }
    if (!this->input[i].target_rating.IsConnected())
    {
      FINROC_LOG_PRINT(rrlib::logging::eLL_ERROR, this->input[i].target_rating.GetName(), " is not connected.");
      return false;
    }

    double input_activity = this->input[i].activity.Get();
    double input_target_rating = this->input[i].target_rating.Get();

    if (input_activity > this->max_input_activity)
    {
      this->max_input_activity_index = i;
      this->max_input_activity = input_activity;
    }

    this->sum_of_input_activities += input_activity;

    this->min_input_target_rating = std::min(this->min_input_target_rating, input_target_rating);
    this->max_input_target_rating = std::max(this->max_input_target_rating, input_target_rating);
  }

  return tDataPortFuser::ForwardFusedValues(this);
}

//----------------------------------------------------------------------
// mbbFusion CalculateActivity
//----------------------------------------------------------------------
template <typename ... TSignalTypes>
double mbbFusion<TSignalTypes...>::CalculateActivity(std::vector<double> &derived_activities, double activation) // const FIXME
{
  double fused_activity = 0;

  switch (this->fusion_method.Get())
  {

  case tFusionMethod::WINNER_TAKES_ALL:
    fused_activity = this->max_input_activity;
    break;

  case tFusionMethod::WEIGHTED_AVERAGE:
    for (auto it = this->input.begin(); it != this->input.end(); ++it)
    {
      fused_activity += it->activity.Get() * it->activity.Get();
    }
    fused_activity /= this->sum_of_input_activities;
    break;

  case tFusionMethod::WEIGHTED_SUM:
    for (auto it = this->input.begin(); it != this->input.end(); ++it)
    {
      fused_activity += it->activity.Get() * it->activity.Get() / this->max_input_activity;
    }
    fused_activity = std::min(1.0, fused_activity);

  }

  return fused_activity * activation;
}

//----------------------------------------------------------------------
// mbbFusion CalculateTargetRating
//----------------------------------------------------------------------
template <typename ... TSignalTypes>
double mbbFusion<TSignalTypes...>::CalculateTargetRating() // const FIXME
{
  double fused_target_rating = 0;
  switch (this->fusion_method.Get())
  {

  case tFusionMethod::WINNER_TAKES_ALL:
    fused_target_rating = this->input[this->max_input_activity_index].target_rating.Get();
    break;

  case tFusionMethod::WEIGHTED_AVERAGE:
  case tFusionMethod::WEIGHTED_SUM:
    for (auto it = this->input.begin(); it != this->input.end(); ++it)
    {
      fused_target_rating += it->activity.Get() * it->target_rating.Get();
    }
    fused_target_rating /= this->sum_of_input_activities;
    break;

  }

  return fused_target_rating;
}

//----------------------------------------------------------------------
// mbbFusion::tDataPortFuser PerformFusion
//----------------------------------------------------------------------
template <typename ... TSignalTypes>
template <size_t Tindex>
bool mbbFusion<TSignalTypes...>::tDataPortFuser::PerformFusion(mbbFusion *parent, tPortPackAccessor<tInput, Tindex> input_accessor, tPortPackAccessor<tOutput, Tindex> output_accessor)
{
  const size_t n = parent->input.size();

  typedef typename tSignalTypes::template tAt<Tindex>::tResult tPortData;

  double input_activities[n];
  tPortData values[n];
  for (size_t i = 0; i < n; ++i)
  {
    input_activities[i] = parent->input[i].activity.Get();
    tInput<tPortData> &input_port = input_accessor.GetPort(parent->input[i].data);
    if (!input_port.IsConnected())
    {
      FINROC_LOG_PRINT_STATIC(rrlib::logging::eLL_ERROR, input_port.GetName(), " is not connected.");
      return false;
    }
    values[i] = input_port.Get();
  }

  tPortData fused_value;

  switch (parent->fusion_method.Get())
  {

  case tFusionMethod::WINNER_TAKES_ALL:
    fused_value = values[parent->max_input_activity_index];
    break;

  case tFusionMethod::WEIGHTED_AVERAGE:
    fused_value = rrlib::data_fusion::FuseValuesUsingWeightedAverage<tPortData>(values, values + n, input_activities, input_activities + n);
    break;

  case tFusionMethod::WEIGHTED_SUM:
    fused_value = rrlib::data_fusion::FuseValuesUsingWeightedSum<tPortData>(values, values + n, input_activities, input_activities + n);
    break;

  }

  output_accessor.GetPort(parent->output).Publish(fused_value);

  return true;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
