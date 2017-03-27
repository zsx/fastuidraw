/*!
 * \file painter_shader_data.hpp
 * \brief file painter_shader_data.hpp
 *
 * Copyright 2016 by Intel.
 *
 * Contact: kevin.rogovin@intel.com
 *
 * This Source Code Form is subject to the
 * terms of the Mozilla Public License, v. 2.0.
 * If a copy of the MPL was not distributed with
 * this file, You can obtain one at
 * http://mozilla.org/MPL/2.0/.
 *
 * \author Kevin Rogovin <kevin.rogovin@intel.com>
 *
 */


#pragma once

#include <fastuidraw/util/c_array.hpp>
#include <fastuidraw/util/util.hpp>

namespace fastuidraw
{
/*!\addtogroup Painter
  @{
 */

  /*!
    Common base class to PainterItemShaderData and
    PainterBlendShaderData to hold shader data for
    custom shaders. Derived classes CANNOT add any
    data or virtual functions. The class
    PainterShaderData is essentially a wrapper over
    a PainterShaderData::DataBase object that handles
    holding data and copying itself (for the purpose
    of copying PainterShaderData objects).
   */
  class FASTUIDRAW_API PainterShaderData
  {
  public:
    /*!
      Class that holds the actual data and packs the data.
      A class derived from PainterShaderData should set the
      field \ref m_data to point to an object derived from
      DataBase for the purpose of holding and packing data.
     */
    class DataBase
    {
    public:
      virtual
      ~DataBase()
      {}

      /*!
        To be implemented by a derived class to create
        a copy of itself.
       */
      virtual
      DataBase*
      copy(void) const = 0;

      /*!
        To be implemented by a derived class to return
        the length of the data needed to encode the data.
        Data is padded to be multiple of alignment.
        \param alignment alignment of the data store
               in units of generic_data, see
               PainterBackend::ConfigurationBase::alignment()
       */
      virtual
      unsigned int
      data_size(unsigned int alignment) const = 0;


      /*!
        To be implemtend by a derive dclass to pack its data.
        \param alignment alignment of the data store
               in units of generic_data, see
               PainterBackend::ConfigurationBase::alignment()
        \param dst place to which to pack data
      */
      virtual
      void
      pack_data(unsigned int alignment, c_array<generic_data> dst) const = 0;
    };

    /*!
      Ctor. A derived class from PainterShaderData
      should set \ref m_data.
     */
    PainterShaderData(void);

    /*!
      Copy ctor, calls DataBase::copy() to
      copy the data behind \ref m_data.
     */
    PainterShaderData(const PainterShaderData &obj);

    ~PainterShaderData();

    /*!
      Assignment operator
     */
    PainterShaderData&
    operator=(const PainterShaderData &rhs);

    /*!
      Returns the length of the data needed to encode the data.
      Data is padded to be multiple of alignment.
      \param alignment alignment of the data store
                       in units of generic_data, see
                       PainterBackend::ConfigurationBase::alignment()
    */
    unsigned int
    data_size(unsigned int alignment) const;

    /*!
      Pack the values of this object
      \param alignment alignment of the data store
                       in units of generic_data, see
                       PainterBackend::ConfigurationBase::alignment()
      \param dst place to which to pack data
    */
    void
    pack_data(unsigned int alignment, c_array<generic_data> dst) const;

    /*!
      Returns a pointer to the underlying object holding
      the data of the PainterShaderData.
     */
    const DataBase*
    data_base(void) const
    {
      return m_data;
    }

  protected:
    /*!
      Initialized as nullptr by the ctor PainterShaderData(void).
      A derived class of PainterShaderData should assign \ref
      m_data to point to an object derived from DataBase.
      That object is the object that is to determine the
      size of data to pack and how to pack the data into
      the data store buffer.
     */
    DataBase *m_data;
  };

  /*!
    PainterItemShaderData holds custom data for item shaders
   */
  class FASTUIDRAW_API PainterItemShaderData:public PainterShaderData
  {};

  /*!
    PainterBlendShaderData holds custom data for blend shaders
   */
  class FASTUIDRAW_API PainterBlendShaderData:public PainterShaderData
  {};
/*! @} */

} //namespace fastuidraw
