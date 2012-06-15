//
// You received this file as part of Finroc
// A framework for intelligent robot control
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
/*!\file    pTestBehaviours.cpp
 *
 * \author  Bernd-Helge Schäfer
 * \author  Tobias Föhst
 *
 * \date    2011-01-09
 *
 */
//----------------------------------------------------------------------
#include "core/default_main_wrapper.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/math/tPose2D.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/ib2c/mbbFusion.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
const char * const cPROGRAM_VERSION = "ver 1.0";
const char * const cPROGRAM_DESCRIPTION = "This program executes the TestBehaviors module/group.";

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// StartUp
//----------------------------------------------------------------------
void StartUp()
{}

//----------------------------------------------------------------------
// InitMainGroup
//----------------------------------------------------------------------
void InitMainGroup(finroc::core::tThreadContainer *main_thread, const rrlib::getopt::tOption& option)
{
  //mbbTestBehaviour* test_behaviour = new mbbTestBehaviour (this);

  typedef finroc::plugins::ib2c::mbbFusion <double, int> myFusion; //rrlib::math::tPose2D, rrlib::math::tPose2D
  myFusion *fusion = new myFusion(main_thread);

  //  std::vector <finroc::core::tAbstractPort*> port_handles;

  std::vector <std::string> names;
  names.push_back("Double Port 0");
  names.push_back("Int Port 0");
  fusion->CreateInputs(names);

  names.clear();
  names.push_back("Double Port 1");
  names.push_back("Int Port 1");

  fusion->CreateInputs(names);

  // test_behaviour->ConnectToFusion (port_handles);

  main_thread->SetCycleTime(500);
}
