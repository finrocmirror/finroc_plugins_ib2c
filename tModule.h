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
#include "rrlib/thread/tTask.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/ib2c/tMetaSignal.h"
#include "plugins/ib2c/typeutil.h"

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
enum class tStimulationMode
{
  AUTO,
  ENABLED,
  DISABLED
};

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//!
/*!
 *
 */
class tModule : public core::structure::tModuleBase
{

  core::tPortGroup *meta_input;
  core::tPortGroup *input;
  core::tPortGroup *meta_output;
  core::tPortGroup *output;

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  template <typename TSignal>
  class tMetaSignalPort : public core::structure::tConveniencePort<TSignal, tModule, core::tPort<TSignal>>  // FIXME: can be replaced by template alias with gcc 4.7
  {
  public:
    template<typename ... TPortParameters>
    explicit tMetaSignalPort(const TPortParameters &... port_parameters) :
      core::structure::tConveniencePort<TSignal, tModule, core::tPort<TSignal>>(port_parameters...)
    {}
  };

  template <typename TSignal>
  struct tMetaInput : public tMetaSignalPort<TSignal>
  {
    template<typename ... TPortParameters>
    explicit tMetaInput(const TPortParameters &... port_parameters) :
      tMetaSignalPort<TSignal>(GetContainer, port_parameters..., core::tBounds<TSignal>(0, 1, false))
    {}

  private:
    static tFrameworkElement *GetContainer(tModule *module)
    {
      return module->meta_input;
    }
  };

  typedef tMetaInput<tStimulation> tStimulationPort;
  typedef tMetaInput<tInhibition> tInhibitionPort;

  template <typename TSignal>
  struct tMetaOutput : public tMetaSignalPort<TSignal>
  {
    template<typename ... TPortParameters>
    explicit tMetaOutput(const TPortParameters &... port_parameters) :
      tMetaSignalPort<TSignal>(GetContainer, port_parameters..., core::tBounds<TSignal>(0, 1, false))
    {}

  private:
    static tFrameworkElement *GetContainer(tModule *module)
    {
      return module->meta_output;
    }
  };

  typedef tMetaOutput<tActivity> tActivityPort;
  typedef tMetaOutput<tTargetRating> tTargetRatingPort;

  typedef tMetaOutput<double> tActivationPort;

  template <typename T = double>
  class tInput : public core::structure::tConveniencePort<T, tModule, core::tPort<T>>
  {
  public:
    template<typename ... TPortParameters>
    explicit tInput(const TPortParameters &... port_parameters) :
      core::structure::tConveniencePort<T, tModule, core::tPort<T>>(GetContainer, port_parameters...)
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
    explicit tOutput(const TPortParameters &... port_parameters) :
      core::structure::tConveniencePort<T, tModule, core::tPort<T>>(GetContainer, port_parameters...)
    {}

  private:
    static tFrameworkElement *GetContainer(tModule *module)
    {
      return module->output;
    }
  };

  tParameter<tStimulationMode> stimulation_mode;
  tParameter<size_t> number_of_inhibition_ports;

  tStimulationPort stimulation;
  std::vector<tInhibitionPort> inhibition;

  tActivityPort activity;
  std::vector<tActivityPort> derived_activity;
  tTargetRatingPort target_rating;

  tActivationPort activation;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tModule(core::tFrameworkElement *parent, const util::tString &name);

  inline const tInhibitionPort &RegisterInhibition(const util::tString &name)
  {
    this->inhibition.push_back(tInhibitionPort("(I) " + name, this));
    this->inhibition.back().Init();
    this->number_of_inhibition_ports.Publish(this->inhibition.size());
    return this->inhibition.back();
  }

  inline finroc::core::tPortGroup &GetMetaInputs()
  {
    return *this->meta_input;
  }

  inline finroc::core::tPortGroup &GetInputs()
  {
    return *this->input;
  }

  inline finroc::core::tPortGroup &GetMetaOutputs()
  {
    return *this->meta_output;
  }

  inline finroc::core::tPortGroup &GetOutputs()
  {
    return *this->output;
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  virtual void EvaluateParameters();

  inline const tActivityPort &RegisterDerivedActivity(const util::tString &name)
  {
    this->derived_activity.push_back(tActivityPort(this, "(A) " + name));
    this->derived_activity.back().Init();
    return this->derived_activity.back();
  }

  /*!
   * May be called in ProcessTransferFunction() method to check
   * whether any input port has changed, since last call to ProcessTransferFunction().
   *
   * (Changed flags are reset automatically)
   */
  bool InputChanged()
  {
    return input_changed;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  class UpdateTask : public rrlib::thread::tTask
  {
    tModule *const module;
  public:
    UpdateTask(tModule *module);
    virtual void ExecuteTask();
    inline const tFrameworkElement &GetLogDescription() const
    {
      return this->module->GetLogDescription();
    }
  };

  UpdateTask update_task;

  bool input_changed;

  double last_activation;
  tActivity last_activity;
  tTargetRating last_target_rating;

  double CalculateActivation() const;

  tInhibition CalculateInhibition() const;

  void CheckActivityLimitation(tActivity activity, double activation);

  void CheckGoalStateActivity(tActivity activity, tTargetRating target_rating, double activation);

  void CheckDerivedActivities(std::vector<tActivity> &derived_activities, tActivity activity);

  virtual bool ProcessTransferFunction(double activation) = 0;

  virtual tActivity CalculateActivity(std::vector<tActivity> &derived_activities, double activation) const = 0;

  virtual tTargetRating CalculateTargetRating(double activation) const = 0;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
