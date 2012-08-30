//
// You received this file as part of Finroc
// A framework for intelligent robot control
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

#include "plugins/ib2c/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <boost/lexical_cast.hpp>

#include "rrlib/util/tTypeList.h"

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
namespace ib2c
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
enum class tFusionMethod
{
  WINNER_TAKES_ALL,
  WEIGHTED_AVERAGE,
  WEIGHTED_SUM
};

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//!
/*!
 * This class implements the iB2C Fusion behavior that allows to arbitrate
 * between competing behaviors.
 */
template <typename ... TSignalTypes>
class mbbFusion : public ib2c::tModule
{
  static finroc::core::tStandardCreateModuleAction<mbbFusion> cCREATE_ACTION;

  typedef rrlib::util::tTypeList<TSignalTypes...> tSignalTypes;

  template < template <typename> class TPort, size_t Tindex = sizeof...(TSignalTypes) - 1 >
  struct tPortPack : public tPortPack < TPort, Tindex - 1 >
  {
    TPort<typename tSignalTypes::template tAt<Tindex>::tResult> port;

    inline tPortPack(mbbFusion *module, const std::string &name_prefix) :
      tPortPack < TPort, Tindex - 1 > (module, name_prefix),
      port(name_prefix + boost::lexical_cast<std::string>(Tindex + 1), module)
    {
      this->port.Init();
    }

    inline core::tPortWrapperBase &GetPort(size_t index)
    {
      assert(index < tSignalTypes::cSIZE);
      if (index == Tindex)
      {
        return this->port;
      }
      return tPortPack < TPort, Tindex - 1 >::GetPort(index);
    }

    inline void ManagedDelete()
    {
      this->port.GetWrapped()->ManagedDelete();
      tPortPack < TPort, Tindex - 1 >::ManagedDelete();
    }
  };

  template <template <typename> class TPort>
  struct tPortPack < TPort, -1 >
  {
    inline tPortPack(mbbFusion *module, const std::string &name_prefix) {}
    inline core::tPortWrapperBase &GetPort(size_t index)
    {
      return *reinterpret_cast<core::tPortWrapperBase *>(0);
    };
    inline void ManagedDelete() {}
  };

  typedef tMetaInput<tActivity> tInputActivityPort;
  typedef tMetaInput<tTargetRating> tInputTargetRatingPort;

  struct tChannel
  {
    tInputActivityPort activity;
    tInputTargetRatingPort target_rating;
    tPortPack<tInput> data;

    inline tChannel(mbbFusion *module, unsigned int group_index) :
      activity("Input Activity " + boost::lexical_cast<std::string>(group_index + 1), module),
      target_rating("Input Target Rating " + boost::lexical_cast<std::string>(group_index + 1), module),
      data(module, "Input " + boost::lexical_cast<std::string>(group_index + 1) + ".")
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

  tParameter<unsigned int> number_of_input_modules;

  tParameter<tFusionMethod> fusion_method;

  std::vector<tChannel> input;

  tPortPack<tOutput> output;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mbbFusion(core::tFrameworkElement *parent, const util::tString &name = "Fusion", unsigned int number_of_input_modules = 1);

  tInputActivityPort &InputActivity(size_t channel_index);

  tInputTargetRatingPort &InputTargetRating(size_t channel_index);

  core::tPortWrapperBase &InputPort(size_t channel_index, size_t port_index);

  core::tPortWrapperBase &OutputPort(size_t port_index);

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
  struct tDataPortFuser<sizeof...(TSignalTypes), dummy>
  {
    static bool PerformFusion(mbbFusion *parent);
  };

  std::vector<double> input_activities;
  std::vector<double> input_target_ratings;
  size_t max_input_activity_index;

  void AdjustInputChannels();

  virtual void EvaluateParameters();

  virtual bool ProcessTransferFunction(double activation);

  virtual tActivity CalculateActivity(std::vector<tActivity> &derived_activities, double activity) const;

  virtual tTargetRating CalculateTargetRating(double activation) const;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "plugins/ib2c/mbbFusion.hpp"

#endif
