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

  template <template <typename> class TPort, size_t Tindex = 0>
  struct tPortPack : public tPortPack < TPort, Tindex + 1 >
  {
    TPort<typename tSignalTypes::template tAt<Tindex>::tResult> port;
    inline tPortPack(mbbFusion *module, const std::string &name_prefix)
      : tPortPack < TPort, Tindex + 1 > (module, name_prefix),
        port(name_prefix + boost::lexical_cast<std::string>(Tindex + 1), module)
    {
      this->port.Init();
    }
    inline void ManagedDelete()
    {
      this->port.GetWrapped()->ManagedDelete();
      tPortPack < TPort, Tindex + 1 >::ManagedDelete();
    }
  };

  template <template <typename> class TPort>
  struct tPortPack<TPort, sizeof...(TSignalTypes)>
  {
    inline tPortPack(mbbFusion *module, const std::string &name_prefix)
  {}
  inline void ManagedDelete()
  {}
  };

  struct tChannel
  {
    tMetaInput activity;
    tMetaInput target_rating;
    tPortPack<tInput> data;
    inline tChannel(mbbFusion *module, unsigned int group_index)
      : activity("Input Activity " + boost::lexical_cast<std::string>(group_index), module),
        target_rating("Input Target Rating " + boost::lexical_cast<std::string>(group_index), module),
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

  mbbFusion(core::tFrameworkElement *parent, const util::tString &name = "mbbFusion");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  template <template <typename> class TPort, size_t Tport_index>
  class tPortPackAccessor
  {
  public:
    typedef TPort<typename tSignalTypes::template tAt<Tport_index>::tResult> tPortType;
    inline static tPortType &GetPort(tPortPack<TPort> &port_pack)
    {
      return ExtractPort(port_pack);
    }

  private:
    template <size_t Tlevel>
    inline static tPortType &ExtractPort(tPortPack<TPort, Tlevel> &port_pack)
    {
      return ExtractPort(static_cast < tPortPack < TPort, Tlevel + 1 > & >(port_pack));
    }

    inline static tPortType &ExtractPort(tPortPack<TPort, Tport_index> &port_pack)
    {
      return port_pack.port;
    }
  };

  class tDataPortFuser
  {
  public:
    static bool ForwardFusedValues(mbbFusion *parent)
    {
      return Iterate(parent, tPortPackAccessor<tInput, 0>(), tPortPackAccessor<tOutput, 0>());
    }

  private:
    template <size_t Tindex>
    inline static bool Iterate(mbbFusion *parent, tPortPackAccessor<tInput, Tindex> input_accessor, tPortPackAccessor<tOutput, Tindex> output_accessor)
    {
      bool success = PerformFusion(parent, input_accessor, output_accessor);
      return success && Iterate(parent, tPortPackAccessor < tInput, Tindex + 1 > (), tPortPackAccessor < tOutput, Tindex + 1 > ());
    }
    inline static bool Iterate(mbbFusion *parent, tPortPackAccessor < tInput, sizeof...(TSignalTypes) - 1 > input_accessor, tPortPackAccessor < tOutput, sizeof...(TSignalTypes) - 1 > output_accessor)
    {
      return PerformFusion(parent, input_accessor, output_accessor);
    }
    template <size_t Tindex>
    static bool PerformFusion(mbbFusion *parent, tPortPackAccessor<tInput, Tindex> input_accessor, tPortPackAccessor<tOutput, Tindex> output_accessor);
  };

  size_t max_input_activity_index;
  double max_input_activity;
  double sum_of_input_activities;
  double min_input_target_rating;
  double max_input_target_rating;

  virtual void ParametersChanged();

  virtual bool ProcessTransferFunction(double activation);

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
