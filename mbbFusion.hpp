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
/*!\file    mbbFusion.cpp
 *
 * \author
 *
 * \date    2011-01-07
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace finroc::ibbc;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
template <typename THead, typename ... TRest>
finroc::core::tStandardCreateModuleAction< mbbFusion <THead, TRest... > > mbbFusion <THead, TRest... >::cCREATE_ACTION("Fusion");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mbbFusion constructors
//----------------------------------------------------------------------
template <typename THead, typename ... TRest>
mbbFusion <THead, TRest... >::mbbFusion(finroc::core::tFrameworkElement *parent, const finroc::util::tString &name)
    : tBehaviourBasedModule(parent, name)
{

  // this->CreateOutputs (&value_2);
}

//----------------------------------------------------------------------
// mbbFusion CalculateActivity
//----------------------------------------------------------------------
template <typename THead, typename ... TRest>
double mbbFusion <THead, TRest... >::CalculateActivity(std::vector <double>& derived_activities,
    double iota)
{
  return 0.;
}

//----------------------------------------------------------------------
// mbbFusion CalculateTargetRating
//----------------------------------------------------------------------
template <typename THead, typename ... TRest>
double mbbFusion <THead, TRest... >::CalculateTargetRating()
{
  return 0.5;
}

//----------------------------------------------------------------------
// mbbFusion CalculateTransferFunction
//----------------------------------------------------------------------
template <typename THead, typename ... TRest>
void mbbFusion <THead, TRest... >::CalculateTransferFunction(double iota)
{}
