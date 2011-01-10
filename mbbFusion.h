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
/*!\file    mbbFusion.h
 *
 * \author  Bernd-Helge Schaefer
 *
 * \date    2011-01-07
 *
 * \brief Contains mbbFusion
 *
 * \b mbbFusion
 *
 */
//----------------------------------------------------------------------
#ifndef _ibbc__mbbFusion_h_
#define _ibbc__mbbFusion_h_

#include "plugins/ibbc/tBehaviourBasedModule.h"
#include "core/port/tAbstractPort.h"
#include "core/port/std/tPort.h"

#include <vector>
#include <boost/utility/enable_if.hpp>

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
//! Short description of mbbFusion
/*! A more detailed description of mbbFusion which
 *   has not done yet!
 *
 */
template <typename THead, typename ... TRest>
class mbbFusion : public tBehaviourBasedModule
{
  static finroc::core::tStandardCreateModuleAction<mbbFusion> cCREATE_ACTION;
//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------

  virtual double CalculateActivity(std::vector <double>& derived_activities,
                                   double iota);

  virtual double CalculateTargetRating();

  virtual void CalculateTransferFunction(double iota);

  std::vector <finroc::core::tAbstractPort*> input_vector;

//----------------------------------------------------------------------
// Public methods
//----------------------------------------------------------------------
public:

  mbbFusion(finroc::core::tFrameworkElement *parent, const finroc::util::tString &name = "Fusion");

  /* template <typename THead, typename ... TRest> */
  /*   mbbFusion(finroc::core::tFrameworkElement *parent, const finroc::util::tString &name = "Fusion") { */

  /*   //    *this << head; */
  /*   //    this->Evaluate(rest...); */
  /*   //    return *this; */
  /* } */

  void CreateInputs(const std::vector <std::string>& names)
  {
    assert((sizeof...(TRest) + 1) == names.size());

    //create behaviour signals for the behaviour to be fused
    //this->input_vector.push_back ();
    this->InnerCreateInputs <THead, TRest...>(names);
  }

  template <typename TLocalHead, typename ... TLocalRest>
  typename boost::enable_if_c < ((sizeof...(TLocalRest)) > 0), void >::type InnerCreateInputs(const std::vector <std::string>& names)
  {

    tInput < finroc::core::tPort <TLocalHead> > (this, names [this->input_vector.size()]);
    //    this->input_vector.push_back ();

    this->InnerCreateInputs <TLocalRest...> (names);
  }

  template <typename TLocalHead>
  void InnerCreateInputs(const std::vector <std::string>& names)
  {
    //this->input_vector.push_back (tInput <TLocalHead> (names [this->input_vector.size ()]));
  }

  /* std::vector <finroc::core::tAbstractPort*>& CreateInputs (std::vector <finroc::core::tAbstractPort*>& input_vector, const THead& head, TRest... rest) { */
  /*   return this->CreateInputs (input_vector, rest...); */
  /* } */

  /* std::vector <finroc::core::tAbstractPort*>& CreateInputs (std::vector <finroc::core::tAbstractPort*>& input_vector, const THead& head) { */
  /*   return input_vector; */
  /* } */

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "plugins/ibbc/mbbFusion.hpp"
#endif
