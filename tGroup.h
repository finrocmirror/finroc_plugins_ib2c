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
/*!\file    plugins/ib2c/tGroup.h
 *
 * \author  Tobias Föhst
 *
 * \date    2012-12-07
 *
 * \brief Contains tGroup
 *
 * \b tGroup
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__ib2c__tGroup_h__
#define __plugins__ib2c__tGroup_h__

#include "plugins/structure/tCompositeComponent.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/data_ports/tProxyPort.h"
#include "rrlib/thread/tTask.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/ib2c/tMetaSignal.h"
#include "plugins/ib2c/tModule.h"

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
 *
 */
class tGroup : public structure::tCompositeComponent
{

  tInterface *meta_input;
  tInterface *meta_output;

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  inline tInterface &GetMetaInputs()
  {
    return *this->meta_input;
  }

  inline tInterface &GetInputs()
  {
    return GetInterface(cINPUT_INTERFACE_INFO, GetFlag(tFlag::SHARED));
  }

  inline tInterface &GetMetaOutputs()
  {
    return *this->meta_output;
  }

  inline tInterface &GetOutputs()
  {
    return GetInterface(cOUTPUT_INTERFACE_INFO, GetFlag(tFlag::SHARED));
  }

  template <typename T>
  class tMetaInput : public structure::tConveniencePort<data_ports::tProxyPort<T, false>, tGroup, tInterface, &tGroup::GetMetaInputs>
  {
  public:
    template<typename ... TPortParameters>
    explicit tMetaInput(const TPortParameters &... port_parameters) :
      structure::tConveniencePort<data_ports::tProxyPort<T, false>, tGroup, tInterface, &tGroup::GetMetaInputs>(port_parameters...)
    {}
  };

  typedef tMetaInput<tStimulation> tStimulationPort;
  typedef tMetaInput<tInhibition> tInhibitionPort;

  template <typename T>
  class tMetaOutput : public structure::tConveniencePort<data_ports::tProxyPort<T, true>, tGroup, tInterface, &tGroup::GetMetaOutputs>
  {
  public:
    template<typename ... TPortParameters>
    explicit tMetaOutput(const TPortParameters &... port_parameters) :
      structure::tConveniencePort<data_ports::tProxyPort<T, true>, tGroup, tInterface, &tGroup::GetMetaOutputs>(port_parameters...)
    {}
  };

  typedef tMetaOutput<tActivity> tActivityPort;
  typedef tMetaOutput<tTargetRating> tTargetRatingPort;

  class tStatusPort : public structure::tConveniencePort<data_ports::tProxyPort<tStatus, true>, tGroup, tInterface, &tGroup::GetMetaOutputs>
  {
  public:
    template <typename ... TPortParameters>
    explicit tStatusPort(const TPortParameters &... port_parameters) :
      structure::tConveniencePort<data_ports::tProxyPort<tStatus, true>, tGroup, tInterface, &tGroup::GetMetaOutputs>(port_parameters...)
    {}
  };

  template <typename T>
  class tInput : public structure::tConveniencePort<data_ports::tProxyPort<T, false>, tGroup, tInterface, &tGroup::GetInputs>
  {
  public:
    template<typename ... TPortParameters>
    explicit tInput(const TPortParameters &... port_parameters) :
      structure::tConveniencePort<data_ports::tProxyPort<T, false>, tGroup, tInterface, &tGroup::GetInputs>(port_parameters...)
    {}
  };

  template <typename T>
  class tOutput : public structure::tConveniencePort<data_ports::tProxyPort<T, true>, tGroup, tInterface, &tGroup::GetOutputs>
  {
  public:
    template<typename ... TPortParameters>
    explicit tOutput(const TPortParameters &... port_parameters) :
      structure::tConveniencePort<data_ports::tProxyPort<T, true>, tGroup, tInterface, &tGroup::GetOutputs>(port_parameters...)
    {}
  };

  tStaticParameter<size_t> number_of_inhibition_ports;

  tParameter <size_t> number_of_cycles_with_suppressed_warnings;
  tParameter<tStimulationMode> stimulation_mode;

  tStimulationPort stimulation;
  std::vector<tInhibitionPort> inhibition;

  tActivityPort activity;
  std::vector<tActivityPort> derived_activity;
  tTargetRatingPort target_rating;

  tStatusPort status;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /*!
   * \param parent                  Parent
   * \param name                    Name of module
   * \param structure_config_file   XML
   * \param share_ports             Share group's ports so that they can be accessed from other runtime environments?
   * \param extra_flags             Any extra flags for group
   */
  tGroup(core::tFrameworkElement *parent, const std::string &name,
         tStimulationMode stimulation_mode, unsigned int number_of_inhibition_ports,
         const std::string &structure_config_file = "",
         bool share_ports = false, tFlags extra_flags = tFlags());

  inline const tInhibitionPort &AddInhibition(const std::string &name)
  {
    this->inhibition.push_back(tInhibitionPort("(I) " + name, this));
    this->inhibition.back().Init();
    this->number_of_inhibition_ports.Set(this->inhibition.size());
    return this->inhibition.back();
  }

  inline const tModule &CharacteristicModule() const
  {
    assert(this->characteristic_module);
    return *this->characteristic_module;
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Static interface info on data port interfaces */
  static const tInterfaceInfo cOUTPUT_INTERFACE_INFO, cINPUT_INTERFACE_INFO, cMETA_OUTPUT_INTERFACE_INFO, cMETA_INPUT_INTERFACE_INFO;

  virtual void OnStaticParameterChange() override;

  void RegisterCharacteristicModule(tModule *module);

  inline tModule &CharacteristicModule()
  {
    assert(this->characteristic_module);
    return *this->characteristic_module;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  tModule *characteristic_module;

  void ConnectCharacteristicModule();
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
