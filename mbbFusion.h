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
/*!\file    mbbFusion.h
 *
 * \author  Bernd-Helge Schäfer
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

  template <template <typename> class TPort, int index, typename ... TTypes>
  struct tPortPack;

  template <template <typename> class TPort, int index, typename THead, typename ... TTail>
  struct tPortPack<TPort, index, THead, TTail...> : public tPortPack < TPort, index + 1, TTail... >
  {
    TPort<THead> port;
    inline tPortPack()
      : tPortPack < TPort, index + 1, TTail... > (),
        port(this, "Signal " + boost::lexical_cast<std::string>(index))
    {}
    inline tPortPack(int group_index)
      : tPortPack < TPort, index + 1, TTail... > (),
        port(this, "Signal " + boost::lexical_cast<std::string>(group_index) + "." + boost::lexical_cast<std::string>(index))
    {}
  };

  template <template <typename> class TPort, int index, typename THead>
  struct tPortPack<TPort, index, THead>
  {
    TPort<THead> port;
    inline tPortPack()
      : port(this, "Signal " + boost::lexical_cast<std::string>(index))
    {}
    inline tPortPack(int group_index)
      : port(this, "Signal " + boost::lexical_cast<std::string>(group_index) + "." + boost::lexical_cast<std::string>(index))
    {}
  };

  struct tChannel
  {
    tMetaInput activity;
    tMetaInput target_rating;
    tPortPack<tInput, 0, TSignalTypes...> data;
    tChannel(int group_index) : data(group_index) {}
  };

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tParameter<tFusionMethod> fusion_method;

  std::vector<tChannel> input;

  tPortPack<tOutput, 0, TSignalTypes...> output;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mbbFusion(core::tFrameworkElement *parent, const util::tString &name = "mbbFusion");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  double max_input_activity_index;
  double max_input_activity;
  double sum_of_input_activities;
  double min_input_target_rating;
  double max_input_target_rating;

  virtual void ProcessTransferFunction(double activation);

  virtual double CalculateActivity(std::vector<double> &derived_activities, double activity); // const; FIXME

  virtual double CalculateTargetRating(); // const; FIXME

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "plugins/ib2c/mbbFusion.hpp"

#endif
