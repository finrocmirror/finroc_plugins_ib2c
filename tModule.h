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
/*!\file    plugins/ib2c/tModule.h
 *
 * \author  Bernd-Helge Schäfer
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

#include "plugins/structure/tModuleBase.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/thread/tTask.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/ib2c/tMetaSignal.h"
#include "plugins/ib2c/tStatus.h"

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
class tModule : public structure::tModuleBase
{
  friend class tGroup;

  core::tPortGroup *meta_input;
  core::tPortGroup *input;
  core::tPortGroup *meta_output;
  core::tPortGroup *output;

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  inline core::tPortGroup &GetMetaInputs()
  {
    return *this->meta_input;
  }

  inline core::tPortGroup &GetInputs()
  {
    return *this->input;
  }

  inline core::tPortGroup &GetMetaOutputs()
  {
    return *this->meta_output;
  }

  inline core::tPortGroup &GetOutputs()
  {
    return *this->output;
  }

  template <typename TSignal>
  class tMetaInput : public structure::tConveniencePort<data_ports::tInputPort<TSignal>, tModule, core::tPortGroup, &tModule::GetMetaInputs>  // FIXME: can be replaced by template alias with gcc 4.7
  {
  public:
    template <typename ... TPortParameters>
    explicit tMetaInput(const TPortParameters &... port_parameters) :
      structure::tConveniencePort<data_ports::tInputPort<TSignal>, tModule, core::tPortGroup, &tModule::GetMetaInputs>(port_parameters..., data_ports::tBounds<TSignal>(0, 1, false))
    {}
  };

  typedef tMetaInput<tStimulation> tStimulationPort;
  typedef tMetaInput<tInhibition> tInhibitionPort;

  template <typename TSignal>
  class tMetaOutput : public structure::tConveniencePort<data_ports::tOutputPort<TSignal>, tModule, core::tPortGroup, &tModule::GetMetaOutputs>  // FIXME: can be replaced by template alias with gcc 4.7
  {
  public:
    template <typename ... TPortParameters>
    explicit tMetaOutput(const TPortParameters &... port_parameters) :
      structure::tConveniencePort<data_ports::tOutputPort<TSignal>, tModule, core::tPortGroup, &tModule::GetMetaOutputs>(port_parameters..., data_ports::tBounds<TSignal>(0, 1, false))
    {}
  };

  typedef tMetaOutput<tActivity> tActivityPort;
  typedef tMetaOutput<tTargetRating> tTargetRatingPort;

  class tStatusPort : public structure::tConveniencePort<data_ports::tOutputPort<tStatus>, tModule, core::tPortGroup, &tModule::GetMetaOutputs>
  {
  public:
    template <typename ... TPortParameters>
    explicit tStatusPort(const TPortParameters &... port_parameters) :
      structure::tConveniencePort<data_ports::tOutputPort<tStatus>, tModule, core::tPortGroup, &tModule::GetMetaOutputs>(port_parameters...)
    {}
  };

  template <typename T>
  class tInput : public structure::tConveniencePort<data_ports::tInputPort<T>, tModule, core::tPortGroup, &tModule::GetInputs>
  {
  public:
    template <typename ... TPortParameters>
    explicit tInput(const TPortParameters &... port_parameters) :
      structure::tConveniencePort<data_ports::tInputPort<T>, tModule, core::tPortGroup, &tModule::GetInputs>(port_parameters...)
    {
      this->RegisterActivityTransferInput(this);
    }

  private:
    void RegisterActivityTransferInput(...)
    {};
    void RegisterActivityTransferInput(tInput<tActivity> *port)
    {
      static_cast<tModule *>(this->GetWrapped()->GetParent()->GetParent())->activity_transfer_inputs.push_back(*port);
      std::cout << port->GetName() << std::endl;
    }
  };

  template <typename T>
  class tOutput : public structure::tConveniencePort<data_ports::tOutputPort<T>, tModule, core::tPortGroup, &tModule::GetOutputs>
  {
  public:
    template <typename ... TPortParameters>
    explicit tOutput(const TPortParameters &... port_parameters) :
      structure::tConveniencePort<data_ports::tOutputPort<T>, tModule, core::tPortGroup, &tModule::GetOutputs>(port_parameters...)
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

  tModule(core::tFrameworkElement *parent, const std::string &name,
          tStimulationMode stimulation_mode, unsigned int number_of_inhibition_ports,
          bool share_output_ports = false, bool share_input_ports = false);

  inline const tInhibitionPort &AddInhibition(const std::string &name)
  {
    this->inhibition.push_back(tInhibitionPort("(I) " + name, this));
    this->inhibition.back().Init();
    this->number_of_inhibition_ports.Set(this->inhibition.size());
    return this->inhibition.back();
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  virtual ~tModule();

  virtual void OnStaticParameterChange() override;

  virtual void OnParameterChange() override;

  inline const tActivityPort &AddDerivedActivity(const std::string &name)
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

  bool WarnNow() const
  {
    if (this->cycles_since_last_warning > this->number_of_cycles_with_suppressed_warnings.Get())
    {
      const_cast<tModule *>(this)->cycles_since_last_warning = 0;
    }
    return this->cycles_since_last_warning == 0;
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
    virtual void ExecuteTask() override;
    inline const tFrameworkElement &GetLogDescription() const
    {
      return this->module->GetLogDescription();
    }
  };

  UpdateTask update_task;

  bool input_changed;

  unsigned int cycles_since_last_warning;
  double last_activation;
  tActivity last_activity;
  tTargetRating last_target_rating;

  std::vector<tInput<tActivity>> activity_transfer_inputs;

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
