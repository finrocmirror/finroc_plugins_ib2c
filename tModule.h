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
/*!\file    plugins/ib2c/tModule.h
 *
 * \author  Bernd-Helge Schaefer
 * \author  Tobias Föhst
 *
 * \date    2010-12-31
 *
 * \brief Contains tModule
 *
 * \b tModule
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__ib2c__tModule_h__
#define __plugins__ib2c__tModule_h__

#include "core/structure/tModuleBase.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
//#include <vector>

//#include "core/port/cc/tPortNumericBounded.h"

//#include "rrlib/logging/definitions.h"

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

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//!
/*!
 *
 */
class tModule : public core::structure::tModuleBase
{

  typedef core::structure::tConveniencePort<double, tModule, core::tPort<double>> tMetaSignalPort;

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  struct tMetaInput : public tMetaSignalPort
  {
    template<typename ... TPortParameters>
    explicit tMetaInput(const TPortParameters &... port_parameters)
      : tMetaSignalPort(GetContainer, port_parameters...)
    {
//      this->SetBounds(core::tBounds<double>(0, 1));
    }

  private:
    static tFrameworkElement *GetContainer(tModule *module)
    {
      return module->input;
    }
  };

  struct tMetaOutput : public tMetaSignalPort
  {
    template<typename ... TPortParameters>
    explicit tMetaOutput(const TPortParameters &... port_parameters)
      : tMetaSignalPort(GetContainer, port_parameters.../*, core::tBounds<double>(0, 1)*/) // FIXME
    {
//      this->SetBounds(core::tBounds<double>(0, 1));
    }

  private:
    static tFrameworkElement *GetContainer(tModule *module)
    {
      return module->output;
    }
  };

  template <typename T = double>
  class tInput : public core::structure::tConveniencePort<T, tModule, core::tPort<T>>
  {
  public:
    template<typename ... TPortParameters>
    explicit tInput(const TPortParameters &... port_parameters)
      : core::structure::tConveniencePort<T, tModule, core::tPort<T>>(GetContainer, port_parameters...)
    {}

  private:
    static tFrameworkElement *GetContainer(tModule *module)
    {
      return module->input;
    }
  };

  template <typename T = double>
  class tOutput : public core::structure::tConveniencePort<T, tModule, core::tPort<T>>
  {
  public:
    template<typename ... TPortParameters>
    explicit tOutput(const TPortParameters &... port_parameters)
      : core::structure::tConveniencePort<T, tModule, core::tPort<T>>(GetContainer, port_parameters...)
    {}

  private:
    static tFrameworkElement *GetContainer(tModule *module)
    {
      return module->output;
    }
  };

  tMetaInput stimulation;
  std::vector<tMetaInput> inhibition;

  tMetaOutput activity;
  std::vector<tMetaOutput> derived_activity;
  tMetaOutput target_rating;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tModule(core::tFrameworkElement *parent, const util::tString &name);

  inline const tMetaInput &AddInhibitionSignal(const util::tString &name)
  {
    this->inhibition.push_back(tMetaInput(this, name));
    return this->inhibition.back();
  }

  inline const tMetaOutput &AddDerivedActivitySignal(const util::tString &name)
  {
    this->derived_activity.push_back(tMetaOutput(this, name));
    return this->derived_activity.back();
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  class UpdateTask : public util::tTask
  {
    tModule *const module;
  public:
    UpdateTask(tModule *module);
    virtual void ExecuteTask();
  };

  core::tPortGroup *input;
  core::tPortGroup *output;
  UpdateTask update_task;

  bool input_changed;

  double last_activation;

  double CalculateActivation(); //const FIXME

  double CalculateInhibition(); //const FIXME

  virtual void ProcessTransferFunction(double activation) = 0;

  virtual double CalculateActivity(std::vector<double> &derived_activities, double activation) = 0; // const = 0; FIXME

  virtual double CalculateTargetRating() = 0; // const = 0; FIXME

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
