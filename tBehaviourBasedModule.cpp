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
/*!\file    tBehaviourBasedModule.cpp
 *
 * \author  Bernd-Helge Schaefer
 *
 * \date    2010-12-31
 *
 */
//----------------------------------------------------------------------
#include "plugins/ibbc/tBehaviourBasedModule.h"

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
// Namespace usage
//----------------------------------------------------------------------
using namespace finroc::core::structure;
using namespace finroc::ibbc;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tBehaviourBasedModule constructors
//----------------------------------------------------------------------
tBehaviourBasedModule::tBehaviourBasedModule(finroc::core::tFrameworkElement *parent, const finroc::util::tString &name) :
    tModule(parent, name),
    activity(this, "Activity"),
    target_rating(this, "Target Rating"),
    stimulation(this, "Stimulation")
{

}

//----------------------------------------------------------------------
// tBehaviourBasedModule Update
//----------------------------------------------------------------------
void tBehaviourBasedModule::Update()
{
  this->activation = this->CalculateActivation();

  double activity_value = this->CalculateActivity(this->derived_activity_values,
                          this->activation);


  double target_rating_value = this->CalculateTargetRating();

  this->CalculateTransferFunction(this->activation);

  this->AssertBoundaries(activity_value);
  this->AssertBoundaries(target_rating_value);
  for (auto iter = this->derived_activity_values.begin();
       iter != this->derived_activity_values.end();
       ++iter)
  {
    this->AssertBoundaries(*iter);
  }

  this->PublishBehaviourSignals(activity_value, target_rating_value, this->derived_activity_values);

  //@todo: naming IBBC ? tIBBCModule?
  this->AssertIbbcPrinciples();
}
