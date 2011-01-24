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
/*!\file    mbbFusion.h
 *
 * \author  Bernd-Helge Schaefer
 *
 * \date    2011-01-07
 *
 * \brief Contains mbbFusion
 *
 * \b mbbFusion
 *
 */
//----------------------------------------------------------------------
#ifndef _ibbc__mbbFusion_h_
#define _ibbc__mbbFusion_h_

#include "plugins/ibbc/tBehaviourBasedModule.h"
#include "plugins/ibbc/tBehaviourDefinitions.h"
#include "core/port/tPortWrapperBase.h"
#include "core/port/tAbstractPort.h"
#include "core/port/std/tPort.h"

#include <vector>
#include <sstream>
#include <boost/utility/enable_if.hpp>

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace ibbc
{
//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Short description of mbbFusion
/*! A more detailed description of mbbFusion which
 *   has not done yet!
 *
 */
template <typename THead, typename ... TRest>
class mbbFusion : public tBehaviourBasedModule
{
  static finroc::core::tStandardCreateModuleAction<mbbFusion> cCREATE_ACTION;
//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------

  tBehaviourDefinitions::tFUSION_METHOD control_fusion_method;

  typedef tInputBehaviourSignal tInputActivity;

  typedef tInputBehaviourSignal tInputTargetRating;

  virtual double CalculateActivity(std::vector <double>& derived_activities,
                                   double activation);

  virtual double CalculateTargetRating();

  virtual void CalculateTransferFunction(double activation);

  std::vector < std::vector < finroc::core::tAbstractPort* > > input_vectors;

  std::vector <tInputActivity> input_activities;
  std::vector <tInputTargetRating> input_target_ratings;

  std::vector < finroc::core::tAbstractPort* > output_vector;

  struct tBehaviourSignalInfo
  {
    tBehaviourSignalInfo()
    {
      this->Reset();
    }

    void Reset()
    {
      this->max_activity_index = -1;
      this->max_a = -1;
      this->max_a_target_rating = 0.;
      this->sum_of_activity = 0.;
      this->sum_of_target_rating = 0.;
      this->square_sum_of_activity = 0.;
      this->sum_of_activity_times_target_rating = 0.;

      this->max_target_rating_limit = -1.;
      this->min_target_rating_limit = 1.;
      this->max_activity_limit = -1.;
      this->min_activity_limit = 1.;
      this->number_of_values = 0;
    }

    int max_activity_index;
    double max_a;
    double max_a_target_rating;
    double sum_of_activity;
    double sum_of_target_rating;
    double square_sum_of_activity;
    double sum_of_activity_times_target_rating;

    double max_target_rating_limit;
    double min_target_rating_limit;
    double max_activity_limit;
    double min_activity_limit;
    int number_of_values;
  };

  void CalculateBehaviourSignalInfo(tBehaviourSignalInfo& behaviour_signal_info)
  {
    // Reset the behaviour signal info struct to default values
    behaviour_signal_info.Reset();

    assert(input_activities.size() == input_target_ratings.size());
    double activation = this->CalculateActivation();

    behaviour_signal_info.number_of_values = input_activities.size();
    double max_activity = 0.;
    for (int i = 0; i < behaviour_signal_info.number_of_values; ++i)
    {
      double input_activity = input_activities [i].GetDoubleRaw();
      double input_target_rating = input_target_ratings [i].GetDoubleRaw();
      behaviour_signal_info.sum_of_activity += input_activity;
      behaviour_signal_info.sum_of_target_rating += input_target_rating;
      behaviour_signal_info.square_sum_of_activity += input_activity * input_activity;
      behaviour_signal_info.sum_of_activity_times_target_rating = input_activity * input_target_rating;

      behaviour_signal_info.min_activity_limit = std::min(input_activity, behaviour_signal_info.min_activity_limit);
      behaviour_signal_info.min_target_rating_limit = std::min(input_target_rating, behaviour_signal_info.min_target_rating_limit);
      behaviour_signal_info.max_target_rating_limit = std::max(input_target_rating, behaviour_signal_info.max_target_rating_limit);

      if (input_activity > max_activity)
      {
        max_activity = input_activity;
        behaviour_signal_info.max_activity_index = i;
      }
    }

    if (behaviour_signal_info.number_of_values > 0 &&
        behaviour_signal_info.max_activity_index >= 0)
    {
      assert(behaviour_signal_info.max_activity_index >= 0);
      assert(behaviour_signal_info.max_activity_index < (int) input_target_ratings.size());

      behaviour_signal_info.max_a = max_activity;
      behaviour_signal_info.max_a_target_rating = input_target_ratings [behaviour_signal_info.max_activity_index].GetDoubleRaw();
      behaviour_signal_info.min_activity_limit *= activation;
      behaviour_signal_info.max_activity_limit = std::min(1., behaviour_signal_info.sum_of_activity) * activation;
    }
    else
    {
      behaviour_signal_info.min_activity_limit = 0;
      behaviour_signal_info.max_activity_limit = activation;
      behaviour_signal_info.min_target_rating_limit = 0.;
      behaviour_signal_info.max_target_rating_limit = 1.;
    }
  }

  tBehaviourSignalInfo behaviour_signal_info;

//----------------------------------------------------------------------
// Public methods
//----------------------------------------------------------------------
public:

  mbbFusion(finroc::core::tFrameworkElement *parent, const finroc::util::tString &name = "Fusion");

  virtual void Update();

  void CreateInputs(const std::vector <std::string>& names)
  {
    //assert that we have a name for each signal
    assert((sizeof...(TRest) + 1) == names.size());

    //create outputs if applicable
    if (this->input_vectors.size() == 0)
    {
      this->input_vectors.resize(sizeof...(TRest) + 1);
      this->InnerCreateOutputs <THead, TRest...>(names);
    }

    //@todo: create behaviour signals for the behaviour to be fused
    std::stringstream activity_name;
    std::stringstream target_rating_name;
    activity_name << "A " << this->input_activities.size();
    target_rating_name << "R " << this->input_target_ratings.size();

    this->input_activities.push_back(tInputActivity(this, activity_name.str().c_str()));
    this->input_target_ratings.push_back(tInputTargetRating(this, target_rating_name.str().c_str()));

    //create inputs for the behaviour to be fused
    this->InnerCreateInputs <THead, TRest...>(names);
  }

  template <typename TLocalHead, typename ... TLocalRest>
  typename boost::enable_if_c < ((sizeof...(TLocalRest)) > 0), void >::type InnerCreateInputs(const std::vector <std::string>& names)
  {
    size_t index = sizeof...(TLocalRest);
    tInput < finroc::core::tCCPort <TLocalHead> > input(this, names [index]);
    this->input_vectors [index].push_back(input.GetWrapped());
    this->InnerCreateInputs <TLocalRest...> (names);
  }

  template <typename TLocalHead>
  void InnerCreateInputs(const std::vector <std::string>& names)
  {
    tInput < finroc::core::tCCPort <TLocalHead> > input(this, names [0]);
    this->input_vectors [0].push_back(input.GetWrapped());
  }

  template <typename TLocalHead, typename ... TLocalRest>
  typename boost::enable_if_c < ((sizeof...(TLocalRest)) > 0), void >::type InnerCreateOutputs(const std::vector <std::string>& names)
  {
    tOutput < finroc::core::tCCPort <TLocalHead> > output(this, names [this->output_vector.size()]);
    this->output_vector.push_back(output.GetWrapped());
    this->InnerCreateOutputs <TLocalRest...> (names);
  }

  template <typename TLocalHead>
  void InnerCreateOutputs(const std::vector <std::string>& names)
  {
    tOutput < finroc::core::tCCPort <TLocalHead> > output(this, names [this->output_vector.size()]);
    this->output_vector.push_back(output.GetWrapped());
  }

  //////////////////////
  // Weighted Sum Fusion
  //////////////////////
  void CalculateTransferFunctionWeightedSum(double activation, const tBehaviourSignalInfo& behaviour_signal_info)
  {
    this->InnerCalculateTransferFunctionWeightedSum <THead, TRest...> (activation, behaviour_signal_info);
  }

  template <typename TLocalHead, typename ... TLocalRest>
  typename boost::enable_if_c < ((sizeof...(TLocalRest)) > 0), void >::type InnerCalculateTransferFunctionWeightedSum(double activation, const tBehaviourSignalInfo& behaviour_signal_info)
  {

    size_t index = (sizeof...(TRest));
    std::vector < finroc::core::tAbstractPort* > & input_ports(this->input_vectors [index]);
    finroc::core::tAbstractPort* output_port = this->output_vector [index];

    this->InnerCalculateTransferFunctionWeightedSum <TLocalHead> (output_port, input_ports, activation, behaviour_signal_info);
    this->InnerCalculateTransferFunctionWeightedSum <TLocalRest...> (activation, behaviour_signal_info);
  }

  template <typename TLocalHead>
  void InnerCalculateTransferFunctionWeightedSum(double activation, const tBehaviourSignalInfo& behaviour_signal_info)
  {

    size_t index = 0;
    std::vector < finroc::core::tAbstractPort* > & input_ports(this->input_vectors [index]);
    finroc::core::tAbstractPort* output_port = this->output_vector [index];

    this->InnerCalculateTransferFunctionWeightedSum <TLocalHead> (output_port, input_ports, activation, behaviour_signal_info);
  }

  template <typename T>
  void InnerCalculateTransferFunctionWeightedSum(finroc::core::tAbstractPort* output_port,
      const std::vector < finroc::core::tAbstractPort* >& input_ports,
      double activation,
      const tBehaviourSignalInfo& behaviour_signal_info)
  {
    assert(this->input_activities.size() == input_ports.size());
    assert((int) this->input_activities.size() == behaviour_signal_info.number_of_values);

    typedef tInput < finroc::core::tCCPort <T> > tInputPort;

    typedef tOutput < finroc::core::tCCPort <T> > tOutputPort;

    T out;
    if (behaviour_signal_info.sum_of_activity > 0.)
    {
      for (int i = 0; i < behaviour_signal_info.number_of_values; ++i)
      {
        // out += output value <vector_element> of behaviour connected to CI starting at index <*it> * activity of this behaviour
        tInputPort* input = (tInputPort*) input_ports [i];
        const T* input_data = input->GetAutoLocked();
        out += (*input_data) * this->input_activities [i].GetDoubleRaw();

        //ControllerInput(*it + eFUSVEC_DIMENSION + vector_element) * ControllerInput(*it);
      }
      out *= (1. / behaviour_signal_info.sum_of_activity);
      //out /= sum_of_activity;
    }
    else
    {
      if (behaviour_signal_info.number_of_values > 0)
      {
        //all activities are zero -> average the control values
        for (int i = 0; i < behaviour_signal_info.number_of_values; ++i)
          //   for (it = this->start_indices_for_control_edges.begin(); it != start_indices_for_control_edges.end(); it++)
        {
          // out += output value <vector_element> of behaviour connected to CI starting at index <*it>
          //out += ControllerInput(*it + eFUSVEC_DIMENSION + vector_element);

          tInputPort* input = (tInputPort*) input_ports [i];
          const T* input_data = input->GetAutoLocked();
          out += (*input_data);
        }
        out *= (1. / behaviour_signal_info.number_of_values);
        //out /= (this->start_indices_for_control_edges.size());
      }
      else
      {
        out *= 0.;
      }
    }

    ((tOutputPort*) output_port)->Publish(out);
    //controller_output[ eCO_DIMENSION + vector_element ] = out;
  }

  //////////////////////
  // Weighted Fusion
  //////////////////////
  void CalculateTransferFunctionWeight(double activation, const tBehaviourSignalInfo& behaviour_signal_info)
  {
    this->InnerCalculateTransferFunctionWeight <THead, TRest...> (activation, behaviour_signal_info);
  }


  template <typename TLocalHead, typename ... TLocalRest>
  typename boost::enable_if_c < ((sizeof...(TLocalRest)) > 0), void >::type InnerCalculateTransferFunctionWeight(double activation, const tBehaviourSignalInfo& behaviour_signal_info)
  {
    size_t index = (sizeof...(TLocalRest));
    std::vector < finroc::core::tAbstractPort* > & input_ports(this->input_vectors [index]);
    finroc::core::tAbstractPort* output_port = this->output_vector [index];

    this->InnerCalculateTransferFunctionWeight <TLocalHead> (output_port, input_ports, activation, behaviour_signal_info);
    this->InnerCalculateTransferFunctionWeight <TLocalRest...> (activation, behaviour_signal_info);
  }

  template <typename TLocalHead>
  void InnerCalculateTransferFunctionWeight(double activation, const tBehaviourSignalInfo& behaviour_signal_info)
  {
    size_t index = 0;
    std::vector < finroc::core::tAbstractPort* > & input_ports(this->input_vectors [index]);
    finroc::core::tAbstractPort* output_port = this->output_vector [index];

    this->InnerCalculateTransferFunctionWeight <TLocalHead> (output_port, input_ports, activation, behaviour_signal_info);
  }

  template <typename T>
  void InnerCalculateTransferFunctionWeight(finroc::core::tAbstractPort* output_port,
      const std::vector < finroc::core::tAbstractPort* >& input_ports,
      double activation,
      const tBehaviourSignalInfo& behaviour_signal_info)
  {
    assert(this->input_activities.size() == input_ports.size());
    assert((int) this->input_activities.size() == behaviour_signal_info.number_of_values);

    typedef tInput < finroc::core::tCCPort <T> > tInputPort;

    typedef tOutput < finroc::core::tCCPort <T> > tOutputPort;

    T out;
    if (behaviour_signal_info.max_a > 0.)
    {
      size_t number_of_inputs = input_ports.size();
      for (size_t i = 0; i < number_of_inputs; ++i)
      {
        tInputPort* input = (tInputPort*) input_ports [i];
        const T* input_data = input->GetAutoLocked();
        out += (*input_data) * (this->input_activities [i].GetDoubleRaw() / behaviour_signal_info.max_a);
      }
    }

    ((tOutputPort*) output_port)->Publish(out);
  }

  //////////////////////
  // Maximum Fusion
  //////////////////////
  void CalculateTransferFunctionMax(double activation, const tBehaviourSignalInfo& behaviour_signal_info)
  {

    if (behaviour_signal_info.max_activity_index >= 0) // to prevent invalid memory access:
    {
      // write out output of dominant input:
      for (int vector_element = 0; vector_element < behaviour_signal_info.number_of_values; vector_element++)
      {
        output_vector [vector_element]->ForwardData(input_vectors [vector_element][behaviour_signal_info.max_activity_index]);
      }
    }
  }

}; // end of class mbbFusion

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "plugins/ibbc/mbbFusion.hpp"
#endif
