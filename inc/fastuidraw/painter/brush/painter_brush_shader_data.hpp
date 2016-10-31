/*!
 * \file painter_brush_shader_data.hpp
 * \brief file painter_brush_shader_data.hpp
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
#include <fastuidraw/image.hpp>
#include <fastuidraw/colorstop_atlas.hpp>

namespace fastuidraw
{
/*!\addtogroup Painter
  @{
 */

  /*!
    Base class to hold shader data for brush shaders.
    Derived classes CANNOT add any data or virtual
    functions. The class PainterBrushShaderData is
    essentially a wrapper over a PainterBrushShaderData::DataBase
    object that handles holding data and copying itself
    (for the purpose of copying PainterBrushShaderData
    objects).
   */
  class PainterBrushShaderData
  {
  public:
    /*!
      Class that holds the actual data and packs the data.
      A class derived from PainterBrushShaderData should set
      the field \ref m_data to point to an object derived
      from DataBase for the purpose of holding and packing data.
     */
    class DataBase
    {
    public:
      /*!
        Convenience typedef to avoid typing so much.
       */
      typedef reference_counted_ptr<const Image> ImageRef;

      /*!
        Convenience typedef to avoid typing so much.
       */
      typedef reference_counted_ptr<const ColorStopSequenceOnAtlas> ColorStopSequenceOnAtlasRef;

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
        To be optionally implemented by a derived class
        to return an array of references to those Image
        objects used in shading. Default implementation
        is to return an empty array.
       */
      virtual
      const_c_array<ImageRef>
      images(void) const
      {
        return const_c_array<ImageRef>();
      }

      /*!
        To be optionally implemented by a derived class
        to return an array of references to those
        ColorStopSquenceOnAtlas objects used in shading.
        Default implementation is to return an empty array.
       */
      virtual
      const_c_array<ColorStopSequenceOnAtlasRef>
      color_stops(void) const
      {
        return const_c_array<ColorStopSequenceOnAtlasRef>();
      }

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
        To be implemtend by a derived class to pack its data.
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
      Ctor. A derived class from PainterBrushShaderData
      should set \ref m_data.
     */
    PainterBrushShaderData(void);

    /*!
      Copy ctor, calls DataBase::copy() to
      copy the data behind \ref m_data.
     */
    PainterBrushShaderData(const PainterBrushShaderData &obj);

    ~PainterBrushShaderData();

    /*!
      Assignment operator
     */
    PainterBrushShaderData&
    operator=(const PainterBrushShaderData &rhs);

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
      the data of the PainterBrushShaderData.
     */
    const DataBase*
    data_base(void) const
    {
      return m_data;
    }

  protected:
    /*!
      Initialized as NULL by the ctor PainterBrushShaderData(void).
      A derived class of PainterBrushShaderData should assign \ref
      m_data to point to an object derived from DataBase.
      That object is the object that is to determine the
      size of data to pack and how to pack the data into
      the data store buffer.
     */
    DataBase *m_data;
  };
/*! @} */

} //namespace fastuidraw
