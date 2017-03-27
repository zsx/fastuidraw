/*!
 * \file painter_shader_group.hpp
 * \brief file painter_shader_group.hpp
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

#include <stdint.h>
#include <fastuidraw/util/blend_mode.hpp>

namespace fastuidraw
{
/*!\addtogroup PainterPacking
  @{
 */

  /*!
    A PainterShaderGroup gives to what groups the active shaders
    of a PainterPacker belong.
   */
  class FASTUIDRAW_API PainterShaderGroup:noncopyable
  {
  public:
    /*!
      The group (see PainterShader::group())
      of the active blend shader.
     */
    uint32_t
    blend_group(void) const;

    /*!
      The group (see PainterShader::group())
      of the active item shader.
     */
    uint32_t
    item_group(void) const;

    /*!
      The shading ID as returned by PainterBrush::shader()
      of the active brush.
     */
    uint32_t
    brush(void) const;

    /*!
      The BlendMode as packed by BlendMode::packed().
     */
    BlendMode::packed_value
    packed_blend_mode(void) const;

  protected:
    /*!
      Ctor, do NOT derive from PainterShaderGroup, doing
      so is asking for heaps of trouble and pain.
    */
    PainterShaderGroup(void) {}
  };

/*! @} */
}
