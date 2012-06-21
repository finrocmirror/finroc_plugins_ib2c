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
 * \author  Bernd-Helge Schaefer
 * \author  Tobias Föhst
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
namespace ib2c
{

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
template <typename ... TSignalTypes>
finroc::core::tStandardCreateModuleAction<mbbFusion<TSignalTypes...>> mbbFusion<TSignalTypes...>::cCREATE_ACTION("Fusion");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mbbFusion constructors
//----------------------------------------------------------------------
template <typename ... TSignalTypes>
mbbFusion<TSignalTypes...>::mbbFusion(finroc::core::tFrameworkElement *parent, const finroc::util::tString &name)
  : tModule(parent, name),

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
void mbbFusion<TSignalTypes...>::ProcessTransferFunction(double activation)
{
  this->max_input_activity_index = 0;
  this->max_input_activity = 0;
  this->sum_of_input_activities = 0;
  this->min_input_target_rating = 1;
  this->max_input_target_rating = 0;

  for (size_t i = 0; 0 < this->input.size(); ++i)
  {
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

  switch (this->fusion_method.Get())
  {

  case tFusionMethod::WINNER_TAKES_ALL:
    break;

  case tFusionMethod::WEIGHTED_AVERAGE:
    break;

  case tFusionMethod::WEIGHTED_SUM:
    break;

  }
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
// End of namespace declaration
//----------------------------------------------------------------------
}
}
