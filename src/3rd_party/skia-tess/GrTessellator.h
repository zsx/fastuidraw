/*
 * Copyright 2015 Google Inc.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#pragma once

#include <fastuidraw/util/util.hpp>
#include <fastuidraw/util/vecN.hpp>


/**
 * Provides utility functions for converting paths to a collection of triangles.
 */
namespace fastuidraw
{

class Tessellator:public fastuidraw::noncopyable
{
public:
  class point
  {
  public:
    double fX, fY;

    bool
    operator==(const point &rhs) const
    {
      return fX == rhs.fX && fY == rhs.fY;
    }
  };

  Tessellator(void);

  virtual
  ~Tessellator();

  void
  begin_contour(void);

  void
  add_vertex(const point &v);

  void
  end_contour(void);

  void
  generate_triangles(void);

  /* Called by generate_triangles() to request to get all
     the contours from which to tessellate. An implementaton
     will call add_contour(), add_vertex() and end_contour()
     to define the contours.
   */
  virtual
  void
  add_contours(void) = 0;

  virtual
  void
  add_triangle(int winding,
               const point &v0,
               const point &v1,
               const point &v2) = 0;

  virtual
  const dvec2&
  min_bounds(void) const = 0;

  virtual
  const dvec2&
  max_bounds(void) const = 0;

private:
  void *m_d;
};

}
