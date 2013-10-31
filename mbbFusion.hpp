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
/*!\file    mbbFusion.hpp
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

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mbbFusion constructors
//----------------------------------------------------------------------
template <typename ... TSignalTypes>
mbbFusion<TSignalTypes...>::mbbFusion(core::tFrameworkElement *parent, const std::string &name,
                                      unsigned int number_of_input_modules,
                                      tStimulationMode stimulation_mode, unsigned int number_of_inhibition_ports) :
  mbbFusionBase(parent, name, number_of_input_modules, stimulation_mode, number_of_inhibition_ports),

  output(this, "Output "),
  max_input_activity_index(0)
{
  this->AdjustInputChannels();
}

//----------------------------------------------------------------------
// mbbFusion InputActivity
//----------------------------------------------------------------------
template <typename ... TSignalTypes>
auto mbbFusion<TSignalTypes...>::InputActivity(size_t channel_index) -> tInputActivityPort&
{
  assert(channel_index < this->input.size());
  return this->input[channel_index].activity;
}

//----------------------------------------------------------------------
// mbbFusion InputTargetRating
//----------------------------------------------------------------------
template <typename ... TSignalTypes>
auto mbbFusion<TSignalTypes...>::InputTargetRating(size_t channel_index) -> tInputTargetRatingPort&
{
  assert(channel_index < this->input.size());
  return this->input[channel_index].target_rating;
}

//----------------------------------------------------------------------
// mbbFusion InputPort
//----------------------------------------------------------------------
template <typename ... TSignalTypes>
core::tPortWrapperBase &mbbFusion<TSignalTypes...>::InputPort(size_t channel_index, size_t port_index)
{
  assert(channel_index < this->input.size());
  return this->input[channel_index].data.GetPort(port_index);
}

//----------------------------------------------------------------------
// mbbFusion OutputPort
//----------------------------------------------------------------------
template <typename ... TSignalTypes>
core::tPortWrapperBase &mbbFusion<TSignalTypes...>::OutputPort(size_t port_index)
{
  return this->output.GetPort(port_index);
}

//----------------------------------------------------------------------
// mbbFusion AdjustInputChannels
//----------------------------------------------------------------------
template <typename ... TSignalTypes>
void mbbFusion<TSignalTypes...>::AdjustInputChannels()
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

//----------------------------------------------------------------------
// mbbFusion EvaluateStaticParameters
//----------------------------------------------------------------------
template <typename ... TSignalTypes>
void mbbFusion<TSignalTypes...>::OnStaticParameterChange()
{
  mbbFusionBase::OnStaticParameterChange();

  if (this->number_of_input_modules.HasChanged())
  {
    this->AdjustInputChannels();
  }
}

//----------------------------------------------------------------------
// mbbFusion ProcessTransferFunction
//----------------------------------------------------------------------
template <typename ... TSignalTypes>
bool mbbFusion<TSignalTypes...>::ProcessTransferFunction(double activation)
{
  this->input_activities.resize(this->input.size());
  this->input_target_ratings.resize(this->input.size());
  this->max_input_activity_index = 0;

  for (size_t i = 0; i < this->input.size(); ++i)
  {
    if (!this->input[i].activity.IsConnected())
    {
      if (this->WarnNow())
      {
        FINROC_LOG_PRINT(ERROR, this->input[i].activity.GetName(), " is not connected.");
      }
      return false;
    }
    if (!this->input[i].target_rating.IsConnected())
    {
      if (this->WarnNow())
      {
        FINROC_LOG_PRINT(ERROR, this->input[i].target_rating.GetName(), " is not connected.");
      }
      return false;
    }

    this->input_activities[i] = this->input[i].activity.Get();
    this->input_target_ratings[i] = this->input[i].target_rating.Get();

    if (this->input_activities[i] > this->input_activities[this->max_input_activity_index])
    {
      this->max_input_activity_index = i;
    }
  }

  return tDataPortFuser<0>::PerformFusion(this);
}

//----------------------------------------------------------------------
// mbbFusion CalculateActivity
//----------------------------------------------------------------------
template <typename ... TSignalTypes>
tActivity mbbFusion<TSignalTypes...>::CalculateActivity(std::vector<tActivity> &derived_activities, double activation) const
{
  double fused_activity = 0;

  switch (this->fusion_method.Get())
  {

  case tFusionMethod::WINNER_TAKES_ALL:
    fused_activity = this->input_activities[this->max_input_activity_index];
    break;

  case tFusionMethod::WEIGHTED_AVERAGE:
    fused_activity = rrlib::data_fusion::FuseValuesUsingWeightedAverage<tActivity>(this->input_activities.begin(), this->input_activities.end(), this->input_activities.begin(), this->input_activities.end());
    break;

  case tFusionMethod::WEIGHTED_SUM:
    fused_activity = rrlib::data_fusion::FuseValuesUsingWeightedSum<tActivity>(this->input_activities.begin(), this->input_activities.end(), this->input_activities.begin(), this->input_activities.end());
    fused_activity = std::min(1.0, fused_activity);
    break;

  }

  return fused_activity * activation;
}

//----------------------------------------------------------------------
// mbbFusion CalculateTargetRating
//----------------------------------------------------------------------
template <typename ... TSignalTypes>
tTargetRating mbbFusion<TSignalTypes...>::CalculateTargetRating(double) const
{
  double fused_target_rating = 0;
  switch (this->fusion_method.Get())
  {

  case tFusionMethod::WINNER_TAKES_ALL:
    fused_target_rating = this->input_target_ratings[this->max_input_activity_index];
    break;

  case tFusionMethod::WEIGHTED_AVERAGE:
  case tFusionMethod::WEIGHTED_SUM:
    fused_target_rating = rrlib::data_fusion::FuseValuesUsingWeightedAverage<tTargetRating>(this->input_target_ratings.begin(), this->input_target_ratings.end(), this->input_activities.begin(), this->input_activities.end());
    break;

  }

  return fused_target_rating;
}

//----------------------------------------------------------------------
// mbbFusion::tDataPortFuser PerformFusion
//----------------------------------------------------------------------
template <typename ... TSignalTypes>
template <size_t Tindex, typename dummy>
bool mbbFusion<TSignalTypes...>::tDataPortFuser<Tindex, dummy>::PerformFusion(mbbFusion *parent)
{
  const size_t n = parent->input.size();

  typedef typename tSignalTypes::template tAt<Tindex>::tResult tPortData;

  tPortData values[n];
  for (size_t i = 0; i < n; ++i)
  {
    data_ports::tInputPort<tPortData> input_port = data_ports::tInputPort<tPortData>::Wrap(*parent->input[i].data.GetPort(Tindex).GetWrapped());
    if (!input_port.IsConnected())
    {
      FINROC_LOG_PRINT_STATIC(ERROR, input_port.GetName(), " is not connected.");
      return false;
    }
    values[i] = input_port.Get();
  }

  data_ports::tOutputPort<tPortData> output_port = data_ports::tOutputPort<tPortData>::Wrap(*parent->output.GetPort(Tindex).GetWrapped());

  switch (parent->fusion_method.Get())
  {

  case tFusionMethod::WINNER_TAKES_ALL:
    output_port.Publish(values[parent->max_input_activity_index]);
    break;

  case tFusionMethod::WEIGHTED_AVERAGE:
    output_port.Publish(rrlib::data_fusion::FuseValuesUsingWeightedAverage<tPortData>(values, values + n, parent->input_activities.begin(), parent->input_activities.end()));
    break;

  case tFusionMethod::WEIGHTED_SUM:
    output_port.Publish(rrlib::data_fusion::FuseValuesUsingWeightedSum<tPortData>(values, values + n, parent->input_activities.begin(), parent->input_activities.end()));
    break;

  }

  return tDataPortFuser < Tindex + 1, dummy >::PerformFusion(parent);
}

template <typename ... TSignalTypes>
template <typename dummy>
bool mbbFusion<TSignalTypes...>::tDataPortFuser<sizeof...(TSignalTypes), dummy>::PerformFusion(mbbFusion *parent)
{
  return true;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
