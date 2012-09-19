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
/*!\file    plugins/ib2c/mNumberToActivityConverter.cpp
 *
 * \author  Tobias Föhst
 *
 * \date    2012-09-19
 *
 */
//----------------------------------------------------------------------
#include "plugins/ib2c/mNumberToActivityConverter.h"

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
// Const values
//----------------------------------------------------------------------
core::tStandardCreateModuleAction<mNumberToActivityConverter> mNumberToActivityConverter::cCREATE_ACTION("NumberToActivityConverter");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mNumberToActivityConverter constructor
//----------------------------------------------------------------------
mNumberToActivityConverter::mNumberToActivityConverter(core::tFrameworkElement *parent, const util::tString &name) :
  tModule(parent, name)
{}

//----------------------------------------------------------------------
// mNumberToActivityConverter destructor
//----------------------------------------------------------------------
mNumberToActivityConverter::~mNumberToActivityConverter()
{}

//----------------------------------------------------------------------
// mNumberToActivityConverter Update
//----------------------------------------------------------------------
void mNumberToActivityConverter::Update()
{
  if (this->InputChanged())
  {
    this->output.Publish(this->input.Get());
  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
