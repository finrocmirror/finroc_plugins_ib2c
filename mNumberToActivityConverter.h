//
// You received this file as part of Finroc
// A Framework for intelligent robot control
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
 * ib2c::tActivity to allow connecting arbitray numeric ports to the
 * stimulation and inhibition ports at network edges.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__ib2c__mNumberToActivityConverter_h__
#define __plugins__ib2c__mNumberToActivityConverter_h__

#include "core/structure/tModule.h"

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
 * ib2c::tActivity to allow connecting arbitray numeric ports to the
 * stimulation and inhibition ports at network edges.
 */
class mNumberToActivityConverter : public core::structure::tModule
{
  static core::tStandardCreateModuleAction<mNumberToActivityConverter> cCREATE_ACTION;

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tInput<double> input;

  tOutput<tActivity> output;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mNumberToActivityConverter(core::tFrameworkElement *parent, const util::tString &name = "NumberToActivityConverter");

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

  virtual void Update();

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
