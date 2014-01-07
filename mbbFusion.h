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
/*!\file    mbbFusion.h
 *
 * \author  Tobias Föhst
 *
 * \date    2011-01-07
 *
 * \brief Contains mbbFusion
 *
 * \b mbbFusion
 *
 * This class implements the iB2C Fusion behavior that allows to arbitrate
 * between competing behaviors.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__ib2c__mbbFusion_h__
#define __plugins__ib2c__mbbFusion_h__

#include "plugins/ib2c/mbbFusionBase.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/util/tTypeList.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/data_ports/tPortPack.h"

//----------------------------------------------------------------------
// Debugging
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
// Class declaration
//----------------------------------------------------------------------
//!
/*!
 * This class implements the iB2C Fusion behavior that allows to arbitrate
 * between competing behaviors.
 */
template <typename ... TSignalTypes>
class mbbFusion : public ib2c::mbbFusionBase
{
  typedef rrlib::util::tTypeList<TSignalTypes...> tSignalTypes;

  struct tChannel
  {
    tInputActivityPort activity;
    tInputTargetRatingPort target_rating;
    data_ports::tPortPack<tInput, tSignalTypes> data;

    inline tChannel(mbbFusion *module, unsigned int group_index) :
      activity("Input Activity " + std::to_string(group_index + 1), module),
      target_rating("Input Target Rating " + std::to_string(group_index + 1), module),
      data(module, "Input " + std::to_string(group_index + 1) + ".")
    {
      this->activity.Init();
      this->target_rating.Init();
    }

    inline void ManagedDelete()
    {
      this->activity.GetWrapped()->ManagedDelete();
      this->target_rating.GetWrapped()->ManagedDelete();
      this->data.ManagedDelete();
    }
  };

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  std::vector<tChannel> input;

  data_ports::tPortPack<tOutput, tSignalTypes> output;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mbbFusion(core::tFrameworkElement *parent, const std::string &name = "Fusion",
            unsigned int number_of_input_modules = 1,
            tStimulationMode stimulation_mode = tStimulationMode::AUTO, unsigned int number_of_inhibition_ports = 0);

  virtual tInputActivityPort &InputActivity(size_t channel_index);

  virtual tInputTargetRatingPort &InputTargetRating(size_t channel_index);

  virtual core::tPortWrapperBase &InputPort(size_t channel_index, size_t port_index);

  virtual core::tPortWrapperBase &OutputPort(size_t port_index);

  virtual size_t MaxPortIndex() const
  {
    return tSignalTypes::cSIZE - 1;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  template <size_t Tindex, typename dummy = int>
  struct tDataPortFuser
  {
    static bool PerformFusion(mbbFusion *parent);
  };

  template <typename dummy>
  struct tDataPortFuser < static_cast<size_t>(-1), dummy >
  {
    static bool PerformFusion(mbbFusion *parent);
  };

  std::vector<double> input_activities;
  std::vector<double> input_target_ratings;
  size_t max_input_activity_index;

  void AdjustInputChannels();

  virtual void OnStaticParameterChange();

  virtual bool ProcessTransferFunction(double activation);

  virtual tActivity CalculateActivity(std::vector<tActivity> &derived_activities, double activation) const;

  virtual tTargetRating CalculateTargetRating(double activation) const;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "plugins/ib2c/mbbFusion.hpp"

#endif
