#pragma once

#include <fastuidraw/util/util.hpp>
#include <fastuidraw/util/vecN.hpp>

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

  Tessellator(const dvec2 &min_bb,
              const dvec2 &max_bb);

  virtual
  ~Tessellator();

  void
  begin_contour(void);

  void
  add_vertex(const point &v);

  void
  end_contour(void);

  /* call to signal that contours are done being added
     and for Tessellator to call add_triangle()
     repeatedly
   */
  void
  generate_triangles(void);

  virtual
  void
  add_triangle(int winding,
               const point &v0,
               const point &v1,
               const point &v2) = 0;

private:
  void *m_d;
};

}
