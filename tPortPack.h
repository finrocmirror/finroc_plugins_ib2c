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
/*!\file    plugins/ib2c/tPortPack.h
 *
 * \author  Tobias Föhst
 *
 * \date    2012-10-18
 *
 * \brief   Contains tPortPack
 *
 * \b tPortPack
 *
 * A group of several ports with different types.
 *
 */
//----------------------------------------------------------------------
#ifndef __plugins__ib2c__tPortPack_h__
#define __plugins__ib2c__tPortPack_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <boost/lexical_cast.hpp>

#include "core/tFrameworkElement.h"

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
//! A group of several ports with different types.
/*!
 * This class creates a list of instances of the given port template
 * inserting several types from a list.  It therefore creates a nested
 * structure of inherited classes and provides a method to access the
 * port on a specific layer at runtime.
 *
 * \param TPort       A port class template to use for every packed port
 * \param TTypeList   A list of the data types used in the ports. e.g. rrlib::util::tTypeList
 * \param Tindex      The pack creates ports using TTypeList[0] to TTypeList[Tindex]. This parameter must not be greater than TTypeList::cSIZE - 1 and is typically inferred and not set by the user.
 */
template < template <typename> class TPort, typename TTypeList, size_t Tindex = TTypeList::cSIZE - 1 >
class tPortPack : private tPortPack < TPort, TTypeList, Tindex - 1 >
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline tPortPack(core::tFrameworkElement *parent, const std::string &name_prefix) :
    tPortPack < TPort, TTypeList, Tindex - 1 > (parent, name_prefix),
    port(name_prefix + boost::lexical_cast<std::string>(Tindex + 1), parent)
  {
    this->port.Init();
  }

  template <typename TIterator>
  inline tPortPack(core::tFrameworkElement *parent, TIterator names_begin, TIterator names_end) :
    tPortPack < TPort, TTypeList, Tindex - 1 > (parent, names_begin, names_end - 1),
    port(names_end - 1, parent)
  {
    this->port.Init();
  }

  inline size_t NumberOfPorts() const
  {
    return Tindex + 1;
  }

  inline core::tPortWrapperBase &GetPort(size_t index)
  {
    assert(index < this->NumberOfPorts());
    if (index == Tindex)
    {
      return this->port;
    }
    return tPortPack < TPort, TTypeList, Tindex - 1 >::GetPort(index);
  }

  inline void ManagedDelete()
  {
    this->port.GetWrapped()->ManagedDelete();
    tPortPack < TPort, TTypeList, Tindex - 1 >::ManagedDelete();
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------

private:

  TPort<typename TTypeList::template tAt<Tindex>::tResult> port;

};

//! The partial specialization of tPortPack to terminate recursion
template <template <typename> class TPort, typename TTypeList>
struct tPortPack < TPort, TTypeList, -1 >
{
  inline tPortPack(core::tFrameworkElement *parent, const std::string &name_prefix)
  {}
  inline core::tPortWrapperBase &GetPort(size_t index)
  {
    return *reinterpret_cast<core::tPortWrapperBase *>(0);
  };
  inline void ManagedDelete()
  {}
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
