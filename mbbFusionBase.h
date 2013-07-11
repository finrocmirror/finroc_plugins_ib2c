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
/*!\file    plugins/ib2c/mbbFusionBase.h
 *
 * \author  Tobias Föhst
 *
 * \date    2013-06-04
 *
 * \brief Contains mbbFusionBase
 *
 * \b mbbFusionBase
 *
 * An abstract interface for mbbFusion to interact with common data
 * without knowing about the specific template instance
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__ib2c__mbbFusionBase_h__
#define __plugins__ib2c__mbbFusionBase_h__

#include "plugins/ib2c/tModule.h"

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
enum class tFusionMethod
{
  WINNER_TAKES_ALL,
  WEIGHTED_AVERAGE,
  WEIGHTED_SUM
};

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * An abstract interface for mbbFusion to interact with common data
 * without knowing about the specific template instance
 */
class mbbFusionBase : public ib2c::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  typedef tMetaInput<tActivity> tInputActivityPort;
  typedef tMetaInput<tTargetRating> tInputTargetRatingPort;

  tStaticParameter<unsigned int> number_of_input_modules;

  tParameter<tFusionMethod> fusion_method;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mbbFusionBase(core::tFrameworkElement *parent, const std::string &name,
                unsigned int number_of_input_modules,
                tStimulationMode stimulation_mode, unsigned int number_of_inhibition_ports);

  virtual tInputActivityPort &InputActivity(size_t channel_index) = 0;

  virtual tInputTargetRatingPort &InputTargetRating(size_t channel_index) = 0;

  virtual core::tPortWrapperBase &InputPort(size_t channel_index, size_t port_index) = 0;

  virtual core::tPortWrapperBase &OutputPort(size_t port_index) = 0;

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  virtual ~mbbFusionBase();

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
