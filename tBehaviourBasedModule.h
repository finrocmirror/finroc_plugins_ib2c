//
// You received this file as part of Finroc
// A framework for innovative robot control
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
/*!\file    tBehaviourBasedModule.h
 *
 * \author  Bernd-Helge Schaefer
 *
 * \date    2010-12-31
 *
 * \brief Contains tBehaviourBasedModule
 *
 * \b tBehaviourBasedModule
 *
 */
//----------------------------------------------------------------------
#ifndef _ibbc__tBehaviourBasedModule_h_
#define _ibbc__tBehaviourBasedModule_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include <vector>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

#include "core/structure/tModule.h"
#include "core/port/cc/tPortNumericBounded.h"

#include "rrlib/logging/definitions.h"
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
//!
/*!
 *
 */
class tBehaviourBasedModule : public finroc::core::structure::tModule
{

public:

  /* tIbbcModule */
  /* tIBBCModule */
  tBehaviourBasedModule(finroc::core::tFrameworkElement *parent, const finroc::util::tString &name);

  ////////////////
  // output behaviour signal declarations
  ////////////////
  struct tOutputBehaviourSignal : public tOutput <finroc::core::tPortNumericBounded>
  {
    tOutputBehaviourSignal(tBehaviourBasedModule *parent, const finroc::util::tString &name)
        : tOutput <finroc::core::tPortNumericBounded> (parent, name)
    {
      this->SetBounds(finroc::core::tBounds(0., 1.));
    }
  };

  typedef tOutputBehaviourSignal tActivity;

  typedef tOutputBehaviourSignal tDerivedActivity;

  typedef tOutputBehaviourSignal tTargetRating;

  ////////////////
  // input behaviour signals declarations
  ////////////////
  struct tInputBehaviourSignal : public tInput <finroc::core::tPortNumericBounded>
  {
    tInputBehaviourSignal(tBehaviourBasedModule *parent, const finroc::util::tString &name)
        : tInput <finroc::core::tPortNumericBounded> (parent, name)
    {
      this->SetBounds(finroc::core::tBounds(0., 1.));
    }
  };

  typedef tInputBehaviourSignal tStimulation;

  typedef tInputBehaviourSignal tInhibition;

  ////////////////////
  // Public Methods
  ////////////////////

  virtual void Update();

  const tInhibition& RegisterInhibition(const finroc::util::tString &name)
  {
    this->inhibitions.push_back(tInhibition(this, ("(I) ") + name));
    return this->inhibitions.back();
  }

protected:

  size_t RegisterDerivedActivity(const finroc::util::tString &name)
  {
    this->derived_activities.push_back(tDerivedActivity(this, name));
    this->derived_activity_values.push_back(0.);
    return (this->derived_activities.size() - 1);
  }

  /*!
   * \brief Calculates the activation of this behaviour-based module.
   */
  virtual double CalculateActivation()
  {
    return this->stimulation.GetDoubleRaw() *(1. - this->CalculateInhibition(this->inhibitions));
  }

private:

  virtual double CalculateActivity(std::vector <double>& derived_activities,
                                   double activation) = 0;

  virtual double CalculateTargetRating() = 0;

  virtual void CalculateTransferFunction(double activation) = 0;

  /////////////////
  // output behaviour signals
  /////////////////

  tActivity activity;

  std::vector <tDerivedActivity> derived_activities;

  std::vector <double> derived_activity_values;

  tTargetRating target_rating;

  /////////////////
  // input behaviour signals
  /////////////////

  tStimulation stimulation;

  std::vector <tInhibition> inhibitions;

  /////////////////
  // internal behaviour signals
  /////////////////

  double activation;

  inline double CalculateInhibition(std::vector <tInhibition> inhibitions)    //const  <- why is GetDoubleRaw not const?
  {
    double inhibition = 0;
    for (auto iter = this->derived_activities.begin();
         iter != this->derived_activities.end();
         ++iter)
    {
      double value = (*iter).GetDoubleRaw();
      if (inhibition > value)
      {
        inhibition = value;
      }
    }
    return inhibition;
  }

  inline void PublishBehaviourSignals(double activity_value,
                                      double target_rating_value,
                                      const std::vector <double>& derived_activity_values)
  {
    this->activity.Publish(activity_value);
    this->target_rating.Publish(target_rating_value);

    assert(this->derived_activities.size() ==
           derived_activity_values.size());

    auto port = this->derived_activities.begin();
    auto value = derived_activity_values.begin();
    for (;
         port != this->derived_activities.end();
         ++port, ++value)
    {
      (*port).Publish((*value));
    }
  }

  virtual void CheckBoundaries(double behaviour_signal)
  {
    assert(behaviour_signal >= 0.);
    assert(behaviour_signal <= 1.);
  }

  virtual bool AssertIbbcPrinciples(bool strict = false)
  {
    bool success = true;

    // check that derived activities do not exceed the activity itself
    double activity_value = this->activity.GetDoubleRaw();
    for (auto iter = this->derived_activities.begin();
         iter != this->derived_activities.end();
         ++iter)
    {
      double derived_activity = (*iter).GetDoubleRaw();
      if (derived_activity > activity_value)
      {
        FINROC_LOG_STREAM(rrlib::logging::eLL_DEBUG) << "Derived activity \"" << (*iter).GetDescription() << "\": " << derived_activity << " greater Activity: " << activity_value;

        success = false;
      }
    }

    if (strict && !success)
    {
      assert(false);
    }

    return success;
  }

};
//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
