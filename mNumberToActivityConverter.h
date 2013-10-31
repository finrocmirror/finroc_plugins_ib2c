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
/*!\file    plugins/ib2c/mNumberToActivityConverter.h
 *
 * \author  Tobias Föhst
 *
 * \date    2012-09-19
 *
 * \brief Contains mNumberToActivityConverter
 *
 * \b mNumberToActivityConverter
 *
 * This module acts as adapter and converts each numeric type into an
 * ib2c::tActivity to allow connecting arbitrary numeric ports to the
 * stimulation and inhibition ports at network edges.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__ib2c__mNumberToActivityConverter_h__
#define __plugins__ib2c__mNumberToActivityConverter_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/ib2c/tMetaSignal.h"

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
//! SHORT_DESCRIPTION
/*!
 * This module acts as adapter and converts each numeric type into an
 * ib2c::tActivity to allow connecting arbitrary numeric ports to the
 * stimulation and inhibition ports at network edges.
 */
class mNumberToActivityConverter : public structure::tModule
{
  static runtime_construction::tStandardCreateModuleAction<mNumberToActivityConverter> cCREATE_ACTION;

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tParameter<unsigned int> number_of_ports;

  std::vector<tInput<double>> input;

  std::vector<tOutput<tActivity>> output;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mNumberToActivityConverter(core::tFrameworkElement *parent, const std::string &name = "NumberToActivityConverter");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mNumberToActivityConverter();

  virtual void OnParameterChange();

  virtual void Update();

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
