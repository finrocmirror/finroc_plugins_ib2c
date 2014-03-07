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
/*!\file    plugins/ib2c/tFunctions.h
 *
 * \author  Jochen Hirth
 *
 * \date    2014-03-07
 *
 * \brief   Contains tFunctions
 *
 * \b tFunctions
 *
 * This class is a rewrite of mca's tSigmoid. It provides sinuid and linear shaped functions, which are often used to calculate the activity of behaviors.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__ib2c__tFunctions_h__
#define __plugins__ib2c__tFunctions_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
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
//! SHORT_DESCRIPTION
/*!
 * This class is a rewrite of mca's tSigmoid. It provides sinuid and linear shaped functions, which are often used to determine the activity of behaviors.
 */
class tFunctions
{

  //----------------------------------------------------------------------
  // Public methods and typedefs
  //----------------------------------------------------------------------
public:
  enum tFunctionMode
  {
    eMODE_RISE,
    eMODE_FALL,
    eMODE_RISE_FALL,
    eMODE_FALL_RISE,
    eMODE_DIMENSION /*!< Endmarker and Dimension */
  };

  enum tFunctionShape
  {
    eSHP_SIGMOID,
    eSHP_LINEAR,
    eSHP_DIMENSION /*!< Endmarker and Dimension */
  };

  tFunctions(tFunctionMode mode,  tFunctionShape shape, double start1 = 0., double end1 = 0., double start2 = 0., double end2 = 0.);

  ~tFunctions();

  void SetFunctionMode(tFunctionMode mode);
  void SetFunctionShape(tFunctionShape shape);
  void SetParameters(double start1, double end1, double start2 = 0., double end2 = 0.);
  double Value(double input);

  //----------------------------------------------------------------------
  // Private fields and methods
  //----------------------------------------------------------------------
private:

  tFunctionMode mode;
  tFunctionShape shape;
  double start_rise, end_rise, start_fall, end_fall;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
