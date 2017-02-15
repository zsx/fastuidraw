/*
 * Heavily based upon src/gpu/GrTessellator.cpp from SKIA whose license is
 *
 * Copyright 2015 Google Inc.
 * Copyright (c) 2011 Google Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following disclaimer
 * in the documentation and/or other materials provided with the
 * distribution.
 *    * Neither the name of Google Inc. nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Changes added:
 *  - Change of SkPoint to fastuidraw::Tessellator::point
 *  - Anti-aliasing tessellation removed
 *  - Wireframe option removed
 *  - Change SkBlockAlloc to Allocator
 *  - Class interface fastuidraw::Tessellator, in doing so remove SkPath
 *  - Related changes to new interface fastuidraw::Tessellator for emit_triangles
 */

#include <vector>
#include <stdint.h>
#include <stdio.h>
#include <fastuidraw/util/fastuidraw_memory.hpp>

#include "skia-tess.hpp"

/*
 * There are six stages to the basic algorithm:
 *
 * 1) Linearize the path contours into piecewise linear segments (path_to_contours()).
 * 2) Build a mesh of edges connecting the vertices (build_edges()).
 * 3) Sort the vertices in Y (and secondarily in X) (merge_sort()).
 * 4) Simplify the mesh by inserting new vertices at intersecting edges (simplify()).
 * 5) Tessellate the simplified mesh into monotone polygons (tessellate()).
 * 6) Triangulate the monotone polygons directly into a vertex buffer (polys_to_triangles()).
 *
 * For screenspace antialiasing, the algorithm is modified as follows:
 *
 * Run steps 1-5 above to produce polygons.
 * 5b) Apply fill rules to extract boundary contours from the polygons (extract_boundaries()).
 * 5c) Simplify boundaries to remove "pointy" vertices which cause inversions (simplify_boundary()).
 * 5d) Displace edges by half a pixel inward and outward along their normals. Intersect to find
 *     new vertices, and set zero alpha on the exterior and one alpha on the interior. Build a new
 *     antialiased mesh from those vertices (boundary_to_aa_mesh()).
 * Run steps 3-6 above on the new mesh, and produce antialiased triangles.
 *
 * The vertex sorting in step (3) is a merge sort, since it plays well with the linked list
 * of vertices (and the necessity of inserting new vertices on intersection).
 *
 * Stages (4) and (5) use an active edge list, which a list of all edges for which the
 * sweep line has crossed the top vertex, but not the bottom vertex.  It's sorted
 * left-to-right based on the point where both edges are active (when both top vertices
 * have been seen, so the "lower" top vertex of the two). If the top vertices are equal
 * (shared), it's sorted based on the last point where both edges are active, so the
 * "upper" bottom vertex.
 *
 * The most complex step is the simplification (4). It's based on the Bentley-Ottman
 * line-sweep algorithm, but due to floating point inaccuracy, the intersection points are
 * not exact and may violate the mesh topology or active edge list ordering. We
 * accommodate this by adjusting the topology of the mesh and AEL to match the intersection
 * points. This occurs in three ways:
 *
 * A) Intersections may cause a shortened edge to no longer be ordered with respect to its
 *    neighbouring edges at the top or bottom vertex. This is handled by merging the
 *    edges (merge_collinear_edges()).
 * B) Intersections may cause an edge to violate the left-to-right ordering of the
 *    active edge list. This is handled by splitting the neighbour edge on the
 *    intersected vertex (cleanup_active_edges()).
 * C) Shortening an edge may cause an active edge to become inactive or an inactive edge
 *    to become active. This is handled by removing or inserting the edge in the active
 *    edge list (fix_active_state()).
 *
 * The tessellation steps (5) and (6) are based on "Triangulating Simple Polygons and
 * Equivalent Problems" (Fournier and Montuno); also a line-sweep algorithm. Note that it
 * currently uses a linked list for the active edge list, rather than a 2-3 tree as the
 * paper describes. The 2-3 tree gives O(lg N) lookups, but insertion and removal also
 * become O(lg N). In all the test cases, it was found that the cost of frequent O(lg N)
 * insertions and removals was greater than the cost of infrequent O(N) lookups with the
 * linked list implementation. With the latter, all removals are O(1), and most insertions
 * are O(1), since we know the adjacent edge in the active edge list based on the topology.
 * Only type 2 vertices (see paper) require the O(N) lookups, and these are much less
 * frequent. There may be other data structures worth investigating, however.
 *
 * Note that the orientation of the line sweep algorithms is determined by the aspect ratio of the
 * path bounds. When the path is taller than it is wide, we sort vertices based on increasing Y
 * coordinate, and secondarily by increasing X coordinate. When the path is wider than it is tall,
 * we sort by increasing X coordinate, but secondarily by *decreasing* Y coordinate. This is so
 * that the "left" and "right" orientation in the code remains correct (edges to the left are
 * increasing in Y; edges to the right are decreasing in Y). That is, the setting rotates 90
 * degrees counterclockwise, rather that transposing.
 */

#define LOGGING_ENABLED 0

#if LOGGING_ENABLED
#define LOG printf
#else
#define LOG(...)
#endif

#define ALLOC_NEW(Type, args, alloc) new(alloc.alloc(sizeof(Type))) Type args

namespace {

struct Vertex;
struct Edge;
struct Poly;

template <class T, T* T::*Prev, T* T::*Next>
void list_insert(T* t, T* prev, T* next, T** head, T** tail) {
    t->*Prev = prev;
    t->*Next = next;
    if (prev) {
        prev->*Next = t;
    } else if (head) {
        *head = t;
    }
    if (next) {
        next->*Prev = t;
    } else if (tail) {
        *tail = t;
    }
}

template <class T, T* T::*Prev, T* T::*Next>
void list_remove(T* t, T** head, T** tail) {
    if (t->*Prev) {
        t->*Prev->*Next = t->*Next;
    } else if (head) {
        *head = t->*Next;
    }
    if (t->*Next) {
        t->*Next->*Prev = t->*Prev;
    } else if (tail) {
        *tail = t->*Prev;
    }
    t->*Prev = t->*Next = nullptr;
}

/**
 * Vertices are used in three ways: first, the path contours are converted into a
 * circularly-linked list of Vertices for each contour. After edge construction, the same Vertices
 * are re-ordered by the merge sort according to the sweep_lt comparator (usually, increasing
 * in Y) using the same fPrev/fNext pointers that were used for the contours, to avoid
 * reallocation. Finally, MonotonePolys are built containing a circularly-linked list of
 * Vertices. (Currently, those Vertices are newly-allocated for the MonotonePolys, since
 * an individual Vertex from the path mesh may belong to multiple
 * MonotonePolys, so the original Vertices cannot be re-used.
 */

struct Vertex {
  Vertex(const fastuidraw::Tessellator::point& point)
    : fPoint(point), fPrev(nullptr), fNext(nullptr)
    , fFirstEdgeAbove(nullptr), fLastEdgeAbove(nullptr)
    , fFirstEdgeBelow(nullptr), fLastEdgeBelow(nullptr)
    , fProcessed(false)
#if LOGGING_ENABLED
    , fID (-1.0f)
#endif
    {}

    fastuidraw::Tessellator::point fPoint;           // Vertex position
    Vertex* fPrev;            // Linked list of contours, then Y-sorted vertices.
    Vertex* fNext;            // "
    Edge*   fFirstEdgeAbove;  // Linked list of edges above this vertex.
    Edge*   fLastEdgeAbove;   // "
    Edge*   fFirstEdgeBelow;  // Linked list of edges below this vertex.
    Edge*   fLastEdgeBelow;   // "
    bool    fProcessed;       // Has this vertex been seen in simplify()?
#if LOGGING_ENABLED
    float   fID;              // Identifier used for logging.
#endif
};

/***************************************************************************************/
class Allocator:fastuidraw::noncopyable
{
public:
  Allocator(void)
  {}

  ~Allocator()
  {
    for(std::vector<void*>::iterator iter = m_ps.begin(),
          end = m_ps.end(); iter != end; ++iter)
      {
        void *q(*iter);
        FASTUIDRAWfree(q);
      }
  }

  void*
  alloc(size_t bytes)
  {
    void *q;
    q = FASTUIDRAWmalloc(bytes);
    m_ps.push_back(q);
    return q;
  }

private:
  std::vector<void*> m_ps;
};


typedef bool (*CompareFunc)(const fastuidraw::Tessellator::point& a, const fastuidraw::Tessellator::point& b);

struct Comparator {
    CompareFunc sweep_lt;
    CompareFunc sweep_gt;
};

bool sweep_lt_horiz(const fastuidraw::Tessellator::point& a, const fastuidraw::Tessellator::point& b) {
    return a.fX == b.fX ? a.fY > b.fY : a.fX < b.fX;
}

bool sweep_lt_vert(const fastuidraw::Tessellator::point& a, const fastuidraw::Tessellator::point& b) {
    return a.fY == b.fY ? a.fX < b.fX : a.fY < b.fY;
}

bool sweep_gt_horiz(const fastuidraw::Tessellator::point& a, const fastuidraw::Tessellator::point& b) {
    return a.fX == b.fX ? a.fY < b.fY : a.fX > b.fX;
}

bool sweep_gt_vert(const fastuidraw::Tessellator::point& a, const fastuidraw::Tessellator::point& b) {
    return a.fY == b.fY ? a.fX > b.fX : a.fY > b.fY;
}

void
emit_triangle(Vertex* v0, Vertex* v1, Vertex* v2, int w,
              fastuidraw::Tessellator *q)
{
  q->add_triangle(w, v0->fPoint, v1->fPoint, v2->fPoint);
}

struct VertexList {
    VertexList() : fHead(nullptr), fTail(nullptr) {}
    Vertex* fHead;
    Vertex* fTail;
    void insert(Vertex* v, Vertex* prev, Vertex* next) {
        list_insert<Vertex, &Vertex::fPrev, &Vertex::fNext>(v, prev, next, &fHead, &fTail);
    }
    void append(Vertex* v) {
        insert(v, fTail, nullptr);
    }
    void prepend(Vertex* v) {
        insert(v, nullptr, fHead);
    }
    void close() {
        if (fHead && fTail) {
            fTail->fNext = fHead;
            fHead->fPrev = fTail;
        }
    }
};


// A line equation in implicit form. fA * x + fB * y + fC = 0, for all points (x, y) on the line.
struct Line {
    Line(Vertex* p, Vertex* q) : Line(p->fPoint, q->fPoint) {}
    Line(const fastuidraw::Tessellator::point& p, const fastuidraw::Tessellator::point& q)
        : fA(static_cast<double>(q.fY) - p.fY)      // a = dY
        , fB(static_cast<double>(p.fX) - q.fX)      // b = -dX
        , fC(static_cast<double>(p.fY) * q.fX -     // c = cross(q, p)
             static_cast<double>(p.fX) * q.fY) {}
    double dist(const fastuidraw::Tessellator::point& p) const {
        return fA * p.fX + fB * p.fY + fC;
    }
    double magSq() const {
        return fA * fA + fB * fB;
    }

    // Compute the intersection of two (infinite) Lines.
    bool intersect(const Line& other, fastuidraw::Tessellator::point* point) {
        double denom = fA * other.fB - fB * other.fA;
        if (denom == 0.0) {
            return false;
        }
        double scale = 1.0f / denom;
        point->fX = (fB * other.fC - other.fB * fC) * scale;
        point->fY = (other.fA * fC - fA * other.fC) * scale;
        return true;
    }
    double fA, fB, fC;
};

/**
 * An Edge joins a top Vertex to a bottom Vertex. Edge ordering for the list of "edges above" and
 * "edge below" a vertex as well as for the active edge list is handled by isLeftOf()/isRightOf().
 * Note that an Edge will give occasionally dist() != 0 for its own endpoints (because floating
 * point). For speed, that case is only tested by the callers which require it (e.g.,
 * cleanup_active_edges()). Edges also handle checking for intersection with other edges.
 * Currently, this converts the edges to the parametric form, in order to avoid doing a division
 * until an intersection has been confirmed. This is slightly slower in the "found" case, but
 * a lot faster in the "not found" case.
 *
 * The coefficients of the line equation stored in double precision to avoid catastrphic
 * cancellation in the isLeftOf() and isRightOf() checks. Using doubles ensures that the result is
 * correct in float, since it's a polynomial of degree 2. The intersect() function, being
 * degree 5, is still subject to catastrophic cancellation. We deal with that by assuming its
 * output may be incorrect, and adjusting the mesh topology to match (see comment at the top of
 * this file).
 */

struct Edge {
    Edge(Vertex* top, Vertex* bottom, int winding)
        : fWinding(winding)
        , fTop(top)
        , fBottom(bottom)
        , fLeft(nullptr)
        , fRight(nullptr)
        , fPrevEdgeAbove(nullptr)
        , fNextEdgeAbove(nullptr)
        , fPrevEdgeBelow(nullptr)
        , fNextEdgeBelow(nullptr)
        , fLeftPoly(nullptr)
        , fRightPoly(nullptr)
        , fLeftPolyPrev(nullptr)
        , fLeftPolyNext(nullptr)
        , fRightPolyPrev(nullptr)
        , fRightPolyNext(nullptr)
        , fUsedInLeftPoly(false)
        , fUsedInRightPoly(false)
        , fLine(top, bottom) {
        }
    int      fWinding;          // 1 == edge goes downward; -1 = edge goes upward.
    Vertex*  fTop;              // The top vertex in vertex-sort-order (sweep_lt).
    Vertex*  fBottom;           // The bottom vertex in vertex-sort-order.
    Edge*    fLeft;             // The linked list of edges in the active edge list.
    Edge*    fRight;            // "
    Edge*    fPrevEdgeAbove;    // The linked list of edges in the bottom Vertex's "edges above".
    Edge*    fNextEdgeAbove;    // "
    Edge*    fPrevEdgeBelow;    // The linked list of edges in the top Vertex's "edges below".
    Edge*    fNextEdgeBelow;    // "
    Poly*    fLeftPoly;         // The Poly to the left of this edge, if any.
    Poly*    fRightPoly;        // The Poly to the right of this edge, if any.
    Edge*    fLeftPolyPrev;
    Edge*    fLeftPolyNext;
    Edge*    fRightPolyPrev;
    Edge*    fRightPolyNext;
    bool     fUsedInLeftPoly;
    bool     fUsedInRightPoly;
    Line     fLine;
    double dist(const fastuidraw::Tessellator::point& p) const {
        return fLine.dist(p);
    }
    bool isRightOf(Vertex* v) const {
        return fLine.dist(v->fPoint) < 0.0;
    }
    bool isLeftOf(Vertex* v) const {
        return fLine.dist(v->fPoint) > 0.0;
    }
    void recompute() {
        fLine = Line(fTop, fBottom);
    }
    bool intersect(const Edge& other, fastuidraw::Tessellator::point* p) {
        LOG("intersecting %g -> %g with %g -> %g\n",
               fTop->fID, fBottom->fID,
               other.fTop->fID, other.fBottom->fID);
        if (fTop == other.fTop || fBottom == other.fBottom) {
            return false;
        }
        double denom = fLine.fA * other.fLine.fB - fLine.fB * other.fLine.fA;
        if (denom == 0.0) {
            return false;
        }
        double dx = static_cast<double>(fTop->fPoint.fX) - other.fTop->fPoint.fX;
        double dy = static_cast<double>(fTop->fPoint.fY) - other.fTop->fPoint.fY;
        double sNumer = -dy * other.fLine.fB - dx * other.fLine.fA;
        double tNumer = -dy * fLine.fB - dx * fLine.fA;
        // If (sNumer / denom) or (tNumer / denom) is not in [0..1], exit early.
        // This saves us doing the divide below unless absolutely necessary.
        if (denom > 0.0 ? (sNumer < 0.0 || sNumer > denom || tNumer < 0.0 || tNumer > denom)
                        : (sNumer > 0.0 || sNumer < denom || tNumer > 0.0 || tNumer < denom)) {
            return false;
        }
        double s = sNumer / denom;
        assert(s >= 0.0 && s <= 1.0);
        p->fX = fTop->fPoint.fX - s * fLine.fB;
        p->fY = fTop->fPoint.fY + s * fLine.fA;
        return true;
    }
};

struct EdgeList {
    EdgeList() : fHead(nullptr), fTail(nullptr), fNext(nullptr), fCount(0) {}
    Edge* fHead;
    Edge* fTail;
    EdgeList* fNext;
    int fCount;
    void insert(Edge* edge, Edge* prev, Edge* next) {
        list_insert<Edge, &Edge::fLeft, &Edge::fRight>(edge, prev, next, &fHead, &fTail);
        fCount++;
    }
    void append(Edge* e) {
        insert(e, fTail, nullptr);
    }
    void remove(Edge* edge) {
        list_remove<Edge, &Edge::fLeft, &Edge::fRight>(edge, &fHead, &fTail);
        fCount--;
    }
    void close() {
        if (fHead && fTail) {
            fTail->fRight = fHead;
            fHead->fLeft = fTail;
        }
    }
    bool contains(Edge* edge) const {
        return edge->fLeft || edge->fRight || fHead == edge;
    }
};

/***************************************************************************************/

struct Poly {
    Poly(Vertex* v, int winding)
        : fFirstVertex(v)
        , fWinding(winding)
        , fHead(nullptr)
        , fTail(nullptr)
        , fNext(nullptr)
        , fPartner(nullptr)
        , fCount(0)
    {
#if LOGGING_ENABLED
        static int gID = 0;
        fID = gID++;
        LOG("*** created Poly %d\n", fID);
#endif
    }
    typedef enum { kLeft_Side, kRight_Side } Side;
    struct MonotonePoly {
        MonotonePoly(Edge* edge, Side side)
            : fSide(side)
            , fFirstEdge(nullptr)
            , fLastEdge(nullptr)
            , fPrev(nullptr)
            , fNext(nullptr) {
            this->addEdge(edge);
        }
        Side          fSide;
        Edge*         fFirstEdge;
        Edge*         fLastEdge;
        MonotonePoly* fPrev;
        MonotonePoly* fNext;
        void addEdge(Edge* edge) {
            if (fSide == kRight_Side) {
                assert(!edge->fUsedInRightPoly);
                list_insert<Edge, &Edge::fRightPolyPrev, &Edge::fRightPolyNext>(
                    edge, fLastEdge, nullptr, &fFirstEdge, &fLastEdge);
                edge->fUsedInRightPoly = true;
            } else {
                assert(!edge->fUsedInLeftPoly);
                list_insert<Edge, &Edge::fLeftPolyPrev, &Edge::fLeftPolyNext>(
                    edge, fLastEdge, nullptr, &fFirstEdge, &fLastEdge);
                edge->fUsedInLeftPoly = true;
            }
        }

        void emit(int winding, fastuidraw::Tessellator *data) {
            Edge* e = fFirstEdge;
            e->fTop->fPrev = e->fTop->fNext = nullptr;
            VertexList vertices;
            vertices.append(e->fTop);
            while (e != nullptr) {
                e->fBottom->fPrev = e->fBottom->fNext = nullptr;
                if (kRight_Side == fSide) {
                    vertices.append(e->fBottom);
                    e = e->fRightPolyNext;
                } else {
                    vertices.prepend(e->fBottom);
                    e = e->fLeftPolyNext;
                }
            }
            Vertex* first = vertices.fHead;
            Vertex* v = first->fNext;
            while (v != vertices.fTail) {
                assert(v && v->fPrev && v->fNext);
                Vertex* prev = v->fPrev;
                Vertex* curr = v;
                Vertex* next = v->fNext;
                double ax = static_cast<double>(curr->fPoint.fX) - prev->fPoint.fX;
                double ay = static_cast<double>(curr->fPoint.fY) - prev->fPoint.fY;
                double bx = static_cast<double>(next->fPoint.fX) - curr->fPoint.fX;
                double by = static_cast<double>(next->fPoint.fY) - curr->fPoint.fY;
                if (ax * by - ay * bx >= 0.0) {
                    //data = emit_triangle(prev, curr, next, aaParams, data);
                    emit_triangle(prev, curr, next, winding, data);
                    v->fPrev->fNext = v->fNext;
                    v->fNext->fPrev = v->fPrev;
                    if (v->fPrev == first) {
                        v = v->fNext;
                    } else {
                        v = v->fPrev;
                    }
                } else {
                    v = v->fNext;
                }
            }
            return;
        }
    };
    Poly* addEdge(Edge* e, Side side, Allocator& alloc) {
        LOG("addEdge (%g -> %g) to poly %d, %s side\n",
               e->fTop->fID, e->fBottom->fID, fID, side == kLeft_Side ? "left" : "right");
        Poly* partner = fPartner;
        Poly* poly = this;
        if (side == kRight_Side) {
            if (e->fUsedInRightPoly) {
                return this;
            }
        } else {
            if (e->fUsedInLeftPoly) {
                return this;
            }
        }
        if (partner) {
            fPartner = partner->fPartner = nullptr;
        }
        if (!fTail) {
            fHead = fTail = ALLOC_NEW(MonotonePoly, (e, side), alloc);
            fCount += 2;
        } else if (e->fBottom == fTail->fLastEdge->fBottom) {
            return poly;
        } else if (side == fTail->fSide) {
            fTail->addEdge(e);
            fCount++;
        } else {
            e = ALLOC_NEW(Edge, (fTail->fLastEdge->fBottom, e->fBottom, 1), alloc);
            fTail->addEdge(e);
            fCount++;
            if (partner) {
                partner->addEdge(e, side, alloc);
                poly = partner;
            } else {
                MonotonePoly* m = ALLOC_NEW(MonotonePoly, (e, side), alloc);
                m->fPrev = fTail;
                fTail->fNext = m;
                fTail = m;
            }
        }
        return poly;
    }
    void emit(fastuidraw::Tessellator *data) {
        if (fCount < 3) {
            return;
        }
        LOG("emit() %d, size %d\n", fID, fCount);
        for (MonotonePoly* m = fHead; m != nullptr; m = m->fNext) {
            m->emit(fWinding, data);
        }
    }
    Vertex* lastVertex() const { return fTail ? fTail->fLastEdge->fBottom : fFirstVertex; }
    Vertex* fFirstVertex;
    int fWinding;
    MonotonePoly* fHead;
    MonotonePoly* fTail;
    Poly* fNext;
    Poly* fPartner;
    int fCount;
#if LOGGING_ENABLED
    int fID;
#endif
};

/***************************************************************************************/

bool coincident(const fastuidraw::Tessellator::point& a, const fastuidraw::Tessellator::point& b) {
    return a == b;
}

Poly* new_poly(Poly** head, Vertex* v, int winding, Allocator& alloc) {
    Poly* poly = ALLOC_NEW(Poly, (v, winding), alloc);
    poly->fNext = *head;
    *head = poly;
    return poly;
}

Vertex*
append_point_to_contour(const fastuidraw::Tessellator::point& p,
                        Vertex* prev, Vertex** head,
                        Allocator& alloc)
{
    Vertex* v = ALLOC_NEW(Vertex, (p), alloc);
#if LOGGING_ENABLED
    static float gID = 0.0f;
    v->fID = gID++;
#endif
    if (prev) {
        prev->fNext = v;
        v->fPrev = prev;
    } else {
        *head = v;
    }
    return v;
}


Edge* new_edge(Vertex* prev, Vertex* next, Allocator& alloc, Comparator& c,
               int winding_scale = 1) {
    int winding = c.sweep_lt(prev->fPoint, next->fPoint) ? winding_scale : -winding_scale;
    Vertex* top = winding < 0 ? next : prev;
    Vertex* bottom = winding < 0 ? prev : next;
    return ALLOC_NEW(Edge, (top, bottom, winding), alloc);
}

void remove_edge(Edge* edge, EdgeList* edges) {
    LOG("removing edge %g -> %g\n", edge->fTop->fID, edge->fBottom->fID);
    assert(edges->contains(edge));
    edges->remove(edge);
}

void insert_edge(Edge* edge, Edge* prev, EdgeList* edges) {
    LOG("inserting edge %g -> %g\n", edge->fTop->fID, edge->fBottom->fID);
    assert(!edges->contains(edge));
    Edge* next = prev ? prev->fRight : edges->fHead;
    edges->insert(edge, prev, next);
}

void find_enclosing_edges(Vertex* v, EdgeList* edges, Edge** left, Edge** right) {
    if (v->fFirstEdgeAbove) {
        *left = v->fFirstEdgeAbove->fLeft;
        *right = v->fLastEdgeAbove->fRight;
        return;
    }
    Edge* next = nullptr;
    Edge* prev;
    for (prev = edges->fTail; prev != nullptr; prev = prev->fLeft) {
        if (prev->isLeftOf(v)) {
            break;
        }
        next = prev;
    }
    *left = prev;
    *right = next;
}

void find_enclosing_edges(Edge* edge, EdgeList* edges, Comparator& c, Edge** left, Edge** right) {
    Edge* prev = nullptr;
    Edge* next;
    for (next = edges->fHead; next != nullptr; next = next->fRight) {
        if ((c.sweep_gt(edge->fTop->fPoint, next->fTop->fPoint) && next->isRightOf(edge->fTop)) ||
            (c.sweep_gt(next->fTop->fPoint, edge->fTop->fPoint) && edge->isLeftOf(next->fTop)) ||
            (c.sweep_lt(edge->fBottom->fPoint, next->fBottom->fPoint) &&
             next->isRightOf(edge->fBottom)) ||
            (c.sweep_lt(next->fBottom->fPoint, edge->fBottom->fPoint) &&
             edge->isLeftOf(next->fBottom))) {
            break;
        }
        prev = next;
    }
    *left = prev;
    *right = next;
}

void fix_active_state(Edge* edge, EdgeList* activeEdges, Comparator& c) {
    if (activeEdges && activeEdges->contains(edge)) {
        if (edge->fBottom->fProcessed || !edge->fTop->fProcessed) {
            remove_edge(edge, activeEdges);
        }
    } else if (edge->fTop->fProcessed && !edge->fBottom->fProcessed) {
        Edge* left;
        Edge* right;
        find_enclosing_edges(edge, activeEdges, c, &left, &right);
        insert_edge(edge, left, activeEdges);
    }
}

void insert_edge_above(Edge* edge, Vertex* v, Comparator& c) {
    if (edge->fTop->fPoint == edge->fBottom->fPoint ||
        c.sweep_gt(edge->fTop->fPoint, edge->fBottom->fPoint)) {
        return;
    }
    LOG("insert edge (%g -> %g) above vertex %g\n", edge->fTop->fID, edge->fBottom->fID, v->fID);
    Edge* prev = nullptr;
    Edge* next;
    for (next = v->fFirstEdgeAbove; next; next = next->fNextEdgeAbove) {
        if (next->isRightOf(edge->fTop)) {
            break;
        }
        prev = next;
    }
    list_insert<Edge, &Edge::fPrevEdgeAbove, &Edge::fNextEdgeAbove>(
        edge, prev, next, &v->fFirstEdgeAbove, &v->fLastEdgeAbove);
}

void insert_edge_below(Edge* edge, Vertex* v, Comparator& c) {
    if (edge->fTop->fPoint == edge->fBottom->fPoint ||
        c.sweep_gt(edge->fTop->fPoint, edge->fBottom->fPoint)) {
        return;
    }
    LOG("insert edge (%g -> %g) below vertex %g\n", edge->fTop->fID, edge->fBottom->fID, v->fID);
    Edge* prev = nullptr;
    Edge* next;
    for (next = v->fFirstEdgeBelow; next; next = next->fNextEdgeBelow) {
        if (next->isRightOf(edge->fBottom)) {
            break;
        }
        prev = next;
    }
    list_insert<Edge, &Edge::fPrevEdgeBelow, &Edge::fNextEdgeBelow>(
        edge, prev, next, &v->fFirstEdgeBelow, &v->fLastEdgeBelow);
}

void remove_edge_above(Edge* edge) {
    LOG("removing edge (%g -> %g) above vertex %g\n", edge->fTop->fID, edge->fBottom->fID,
        edge->fBottom->fID);
    list_remove<Edge, &Edge::fPrevEdgeAbove, &Edge::fNextEdgeAbove>(
        edge, &edge->fBottom->fFirstEdgeAbove, &edge->fBottom->fLastEdgeAbove);
}

void remove_edge_below(Edge* edge) {
    LOG("removing edge (%g -> %g) below vertex %g\n", edge->fTop->fID, edge->fBottom->fID,
        edge->fTop->fID);
    list_remove<Edge, &Edge::fPrevEdgeBelow, &Edge::fNextEdgeBelow>(
        edge, &edge->fTop->fFirstEdgeBelow, &edge->fTop->fLastEdgeBelow);
}

void erase_edge_if_zero_winding(Edge* edge, EdgeList* edges) {
    if (edge->fWinding != 0) {
        return;
    }
    LOG("erasing edge (%g -> %g)\n", edge->fTop->fID, edge->fBottom->fID);
    remove_edge_above(edge);
    remove_edge_below(edge);
    if (edges && edges->contains(edge)) {
        remove_edge(edge, edges);
    }
}

void merge_collinear_edges(Edge* edge, EdgeList* activeEdges, Comparator& c);

void set_top(Edge* edge, Vertex* v, EdgeList* activeEdges, Comparator& c) {
    remove_edge_below(edge);
    edge->fTop = v;
    edge->recompute();
    insert_edge_below(edge, v, c);
    fix_active_state(edge, activeEdges, c);
    merge_collinear_edges(edge, activeEdges, c);
}

void set_bottom(Edge* edge, Vertex* v, EdgeList* activeEdges, Comparator& c) {
    remove_edge_above(edge);
    edge->fBottom = v;
    edge->recompute();
    insert_edge_above(edge, v, c);
    fix_active_state(edge, activeEdges, c);
    merge_collinear_edges(edge, activeEdges, c);
}

void merge_edges_above(Edge* edge, Edge* other, EdgeList* activeEdges, Comparator& c) {
    if (coincident(edge->fTop->fPoint, other->fTop->fPoint)) {
        LOG("merging coincident above edges (%g, %g) -> (%g, %g)\n",
            edge->fTop->fPoint.fX, edge->fTop->fPoint.fY,
            edge->fBottom->fPoint.fX, edge->fBottom->fPoint.fY);
        other->fWinding += edge->fWinding;
        erase_edge_if_zero_winding(other, activeEdges);
        edge->fWinding = 0;
        erase_edge_if_zero_winding(edge, activeEdges);
    } else if (c.sweep_lt(edge->fTop->fPoint, other->fTop->fPoint)) {
        other->fWinding += edge->fWinding;
        erase_edge_if_zero_winding(other, activeEdges);
        set_bottom(edge, other->fTop, activeEdges, c);
    } else {
        edge->fWinding += other->fWinding;
        erase_edge_if_zero_winding(edge, activeEdges);
        set_bottom(other, edge->fTop, activeEdges, c);
    }
}

void merge_edges_below(Edge* edge, Edge* other, EdgeList* activeEdges, Comparator& c) {
    if (coincident(edge->fBottom->fPoint, other->fBottom->fPoint)) {
        LOG("merging coincident below edges (%g, %g) -> (%g, %g)\n",
            edge->fTop->fPoint.fX, edge->fTop->fPoint.fY,
            edge->fBottom->fPoint.fX, edge->fBottom->fPoint.fY);
        other->fWinding += edge->fWinding;
        erase_edge_if_zero_winding(other, activeEdges);
        edge->fWinding = 0;
        erase_edge_if_zero_winding(edge, activeEdges);
    } else if (c.sweep_lt(edge->fBottom->fPoint, other->fBottom->fPoint)) {
        edge->fWinding += other->fWinding;
        erase_edge_if_zero_winding(edge, activeEdges);
        set_top(other, edge->fBottom, activeEdges, c);
    } else {
        other->fWinding += edge->fWinding;
        erase_edge_if_zero_winding(other, activeEdges);
        set_top(edge, other->fBottom, activeEdges, c);
    }
}

void merge_collinear_edges(Edge* edge, EdgeList* activeEdges, Comparator& c) {
    if (edge->fPrevEdgeAbove && (edge->fTop == edge->fPrevEdgeAbove->fTop ||
                                 !edge->fPrevEdgeAbove->isLeftOf(edge->fTop))) {
        merge_edges_above(edge, edge->fPrevEdgeAbove, activeEdges, c);
    } else if (edge->fNextEdgeAbove && (edge->fTop == edge->fNextEdgeAbove->fTop ||
                                        !edge->isLeftOf(edge->fNextEdgeAbove->fTop))) {
        merge_edges_above(edge, edge->fNextEdgeAbove, activeEdges, c);
    }
    if (edge->fPrevEdgeBelow && (edge->fBottom == edge->fPrevEdgeBelow->fBottom ||
                                 !edge->fPrevEdgeBelow->isLeftOf(edge->fBottom))) {
        merge_edges_below(edge, edge->fPrevEdgeBelow, activeEdges, c);
    } else if (edge->fNextEdgeBelow && (edge->fBottom == edge->fNextEdgeBelow->fBottom ||
                                        !edge->isLeftOf(edge->fNextEdgeBelow->fBottom))) {
        merge_edges_below(edge, edge->fNextEdgeBelow, activeEdges, c);
    }
}

void split_edge(Edge* edge, Vertex* v, EdgeList* activeEdges, Comparator& c, Allocator& alloc);

void cleanup_active_edges(Edge* edge, EdgeList* activeEdges, Comparator& c, Allocator& alloc) {
    Vertex* top = edge->fTop;
    Vertex* bottom = edge->fBottom;
    if (edge->fLeft) {
        Vertex* leftTop = edge->fLeft->fTop;
        Vertex* leftBottom = edge->fLeft->fBottom;
        if (c.sweep_gt(top->fPoint, leftTop->fPoint) && !edge->fLeft->isLeftOf(top)) {
            split_edge(edge->fLeft, edge->fTop, activeEdges, c, alloc);
        } else if (c.sweep_gt(leftTop->fPoint, top->fPoint) && !edge->isRightOf(leftTop)) {
            split_edge(edge, leftTop, activeEdges, c, alloc);
        } else if (c.sweep_lt(bottom->fPoint, leftBottom->fPoint) &&
                   !edge->fLeft->isLeftOf(bottom)) {
            split_edge(edge->fLeft, bottom, activeEdges, c, alloc);
        } else if (c.sweep_lt(leftBottom->fPoint, bottom->fPoint) && !edge->isRightOf(leftBottom)) {
            split_edge(edge, leftBottom, activeEdges, c, alloc);
        }
    }
    if (edge->fRight) {
        Vertex* rightTop = edge->fRight->fTop;
        Vertex* rightBottom = edge->fRight->fBottom;
        if (c.sweep_gt(top->fPoint, rightTop->fPoint) && !edge->fRight->isRightOf(top)) {
            split_edge(edge->fRight, top, activeEdges, c, alloc);
        } else if (c.sweep_gt(rightTop->fPoint, top->fPoint) && !edge->isLeftOf(rightTop)) {
            split_edge(edge, rightTop, activeEdges, c, alloc);
        } else if (c.sweep_lt(bottom->fPoint, rightBottom->fPoint) &&
                   !edge->fRight->isRightOf(bottom)) {
            split_edge(edge->fRight, bottom, activeEdges, c, alloc);
        } else if (c.sweep_lt(rightBottom->fPoint, bottom->fPoint) &&
                   !edge->isLeftOf(rightBottom)) {
            split_edge(edge, rightBottom, activeEdges, c, alloc);
        }
    }
}

void split_edge(Edge* edge, Vertex* v, EdgeList* activeEdges, Comparator& c, Allocator& alloc) {
    LOG("splitting edge (%g -> %g) at vertex %g (%g, %g)\n",
        edge->fTop->fID, edge->fBottom->fID,
        v->fID, v->fPoint.fX, v->fPoint.fY);
    if (c.sweep_lt(v->fPoint, edge->fTop->fPoint)) {
        set_top(edge, v, activeEdges, c);
    } else if (c.sweep_gt(v->fPoint, edge->fBottom->fPoint)) {
        set_bottom(edge, v, activeEdges, c);
    } else {
        Edge* newEdge = ALLOC_NEW(Edge, (v, edge->fBottom, edge->fWinding), alloc);
        insert_edge_below(newEdge, v, c);
        insert_edge_above(newEdge, edge->fBottom, c);
        set_bottom(edge, v, activeEdges, c);
        cleanup_active_edges(edge, activeEdges, c, alloc);
        fix_active_state(newEdge, activeEdges, c);
        merge_collinear_edges(newEdge, activeEdges, c);
    }
}

Edge* connect(Vertex* prev, Vertex* next, Allocator& alloc, Comparator c,
              int winding_scale = 1) {
    Edge* edge = new_edge(prev, next, alloc, c, winding_scale);
    if (edge->fWinding > 0) {
        insert_edge_below(edge, prev, c);
        insert_edge_above(edge, next, c);
    } else {
        insert_edge_below(edge, next, c);
        insert_edge_above(edge, prev, c);
    }
    merge_collinear_edges(edge, nullptr, c);
    return edge;
}

void merge_vertices(Vertex* src, Vertex* dst, Vertex** head, Comparator& c) {
    LOG("found coincident verts at %g, %g; merging %g into %g\n", src->fPoint.fX, src->fPoint.fY,
        src->fID, dst->fID);
    for (Edge* edge = src->fFirstEdgeAbove; edge;) {
        Edge* next = edge->fNextEdgeAbove;
        set_bottom(edge, dst, nullptr, c);
        edge = next;
    }
    for (Edge* edge = src->fFirstEdgeBelow; edge;) {
        Edge* next = edge->fNextEdgeBelow;
        set_top(edge, dst, nullptr, c);
        edge = next;
    }
    list_remove<Vertex, &Vertex::fPrev, &Vertex::fNext>(src, head, nullptr);
}

Vertex* check_for_intersection(Edge* edge, Edge* other, EdgeList* activeEdges, Comparator& c,
                               Allocator& alloc) {
    fastuidraw::Tessellator::point p;
    if (!edge || !other) {
        return nullptr;
    }
    if (edge->intersect(*other, &p)) {
        Vertex* v;
        LOG("found intersection, pt is %g, %g\n", p.fX, p.fY);
        if (p == edge->fTop->fPoint || c.sweep_lt(p, edge->fTop->fPoint)) {
            split_edge(other, edge->fTop, activeEdges, c, alloc);
            v = edge->fTop;
        } else if (p == edge->fBottom->fPoint || c.sweep_gt(p, edge->fBottom->fPoint)) {
            split_edge(other, edge->fBottom, activeEdges, c, alloc);
            v = edge->fBottom;
        } else if (p == other->fTop->fPoint || c.sweep_lt(p, other->fTop->fPoint)) {
            split_edge(edge, other->fTop, activeEdges, c, alloc);
            v = other->fTop;
        } else if (p == other->fBottom->fPoint || c.sweep_gt(p, other->fBottom->fPoint)) {
            split_edge(edge, other->fBottom, activeEdges, c, alloc);
            v = other->fBottom;
        } else {
            Vertex* nextV = edge->fTop;
            while (c.sweep_lt(p, nextV->fPoint)) {
                nextV = nextV->fPrev;
            }
            while (c.sweep_lt(nextV->fPoint, p)) {
                nextV = nextV->fNext;
            }
            Vertex* prevV = nextV->fPrev;
            if (coincident(prevV->fPoint, p)) {
                v = prevV;
            } else if (coincident(nextV->fPoint, p)) {
                v = nextV;
            } else {
                v = ALLOC_NEW(Vertex, (p), alloc);
                LOG("inserting between %g (%g, %g) and %g (%g, %g)\n",
                    prevV->fID, prevV->fPoint.fX, prevV->fPoint.fY,
                    nextV->fID, nextV->fPoint.fX, nextV->fPoint.fY);
#if LOGGING_ENABLED
                v->fID = (nextV->fID + prevV->fID) * 0.5f;
#endif
                v->fPrev = prevV;
                v->fNext = nextV;
                prevV->fNext = v;
                nextV->fPrev = v;
            }
            split_edge(edge, v, activeEdges, c, alloc);
            split_edge(other, v, activeEdges, c, alloc);
        }
        return v;
    }
    return nullptr;
}

void sanitize_contours(Vertex** contours, int contourCnt) {
    for (int i = 0; i < contourCnt; ++i) {
        assert(contours[i]);
        for (Vertex* v = contours[i];;) {
            if (coincident(v->fPrev->fPoint, v->fPoint)) {
                LOG("vertex %g,%g coincident; removing\n", v->fPoint.fX, v->fPoint.fY);
                if (v->fPrev == v) {
                    contours[i] = nullptr;
                    break;
                }
                v->fPrev->fNext = v->fNext;
                v->fNext->fPrev = v->fPrev;
                if (contours[i] == v) {
                    contours[i] = v->fNext;
                }
                v = v->fPrev;
            } else {
                v = v->fNext;
                if (v == contours[i]) break;
            }
        }
    }
}

void merge_coincident_vertices(Vertex** vertices, Comparator& c) {
    for (Vertex* v = (*vertices)->fNext; v != nullptr; v = v->fNext) {
        if (c.sweep_lt(v->fPoint, v->fPrev->fPoint)) {
            v->fPoint = v->fPrev->fPoint;
        }
        if (coincident(v->fPrev->fPoint, v->fPoint)) {
            merge_vertices(v->fPrev, v, vertices, c);
        }
    }
}

// Stage 2: convert the contours to a mesh of edges connecting the vertices.

Vertex* build_edges(Vertex** contours, int contourCnt, Comparator& c, Allocator& alloc) {
    Vertex* vertices = nullptr;
    Vertex* prev = nullptr;
    for (int i = 0; i < contourCnt; ++i) {
        for (Vertex* v = contours[i]; v != nullptr;) {
            Vertex* vNext = v->fNext;
            connect(v->fPrev, v, alloc, c);
            if (prev) {
                prev->fNext = v;
                v->fPrev = prev;
            } else {
                vertices = v;
            }
            prev = v;
            v = vNext;
            if (v == contours[i]) break;
        }
    }
    if (prev) {
        prev->fNext = vertices->fPrev = nullptr;
    }
    return vertices;
}

// Stage 3: sort the vertices by increasing sweep direction.

Vertex* sorted_merge(Vertex* a, Vertex* b, Comparator& c);

void front_back_split(Vertex* v, Vertex** pFront, Vertex** pBack) {
    Vertex* fast;
    Vertex* slow;
    if (!v || !v->fNext) {
        *pFront = v;
        *pBack = nullptr;
    } else {
        slow = v;
        fast = v->fNext;

        while (fast != nullptr) {
            fast = fast->fNext;
            if (fast != nullptr) {
                slow = slow->fNext;
                fast = fast->fNext;
            }
        }

        *pFront = v;
        *pBack = slow->fNext;
        slow->fNext->fPrev = nullptr;
        slow->fNext = nullptr;
    }
}

void merge_sort(Vertex** head, Comparator& c) {
    if (!*head || !(*head)->fNext) {
        return;
    }

    Vertex* a;
    Vertex* b;
    front_back_split(*head, &a, &b);

    merge_sort(&a, c);
    merge_sort(&b, c);

    *head = sorted_merge(a, b, c);
}

Vertex* sorted_merge(Vertex* a, Vertex* b, Comparator& c) {
    VertexList vertices;

    while (a && b) {
        if (c.sweep_lt(a->fPoint, b->fPoint)) {
            Vertex* next = a->fNext;
            vertices.append(a);
            a = next;
        } else {
            Vertex* next = b->fNext;
            vertices.append(b);
            b = next;
        }
    }
    if (a) {
        vertices.insert(a, vertices.fTail, a->fNext);
    }
    if (b) {
        vertices.insert(b, vertices.fTail, b->fNext);
    }
    return vertices.fHead;
}

// Stage 4: Simplify the mesh by inserting new vertices at intersecting edges.

void simplify(Vertex* vertices, Comparator& c, Allocator& alloc) {
    LOG("simplifying complex polygons\n");
    EdgeList activeEdges;
    for (Vertex* v = vertices; v != nullptr; v = v->fNext) {
        if (!v->fFirstEdgeAbove && !v->fFirstEdgeBelow) {
            continue;
        }
#if LOGGING_ENABLED
        LOG("\nvertex %g: (%g,%g)\n", v->fID, v->fPoint.fX, v->fPoint.fY);
#endif
        Edge* leftEnclosingEdge = nullptr;
        Edge* rightEnclosingEdge = nullptr;
        bool restartChecks;
        do {
            restartChecks = false;
            find_enclosing_edges(v, &activeEdges, &leftEnclosingEdge, &rightEnclosingEdge);
            if (v->fFirstEdgeBelow) {
                for (Edge* edge = v->fFirstEdgeBelow; edge != nullptr; edge = edge->fNextEdgeBelow) {
                    if (check_for_intersection(edge, leftEnclosingEdge, &activeEdges, c, alloc)) {
                        restartChecks = true;
                        break;
                    }
                    if (check_for_intersection(edge, rightEnclosingEdge, &activeEdges, c, alloc)) {
                        restartChecks = true;
                        break;
                    }
                }
            } else {
                if (Vertex* pv = check_for_intersection(leftEnclosingEdge, rightEnclosingEdge,
                                                        &activeEdges, c, alloc)) {
                    if (c.sweep_lt(pv->fPoint, v->fPoint)) {
                        v = pv;
                    }
                    restartChecks = true;
                }

            }
        } while (restartChecks);
        for (Edge* e = v->fFirstEdgeAbove; e; e = e->fNextEdgeAbove) {
            remove_edge(e, &activeEdges);
        }
        Edge* leftEdge = leftEnclosingEdge;
        for (Edge* e = v->fFirstEdgeBelow; e; e = e->fNextEdgeBelow) {
            insert_edge(e, leftEdge, &activeEdges);
            leftEdge = e;
        }
        v->fProcessed = true;
    }
}

// Stage 5: Tessellate the simplified mesh into monotone polygons.

Poly* tessellate(Vertex* vertices, Allocator& alloc) {
    LOG("tessellating simple polygons\n");
    EdgeList activeEdges;
    Poly* polys = nullptr;
    for (Vertex* v = vertices; v != nullptr; v = v->fNext) {
        if (!v->fFirstEdgeAbove && !v->fFirstEdgeBelow) {
            continue;
        }
#if LOGGING_ENABLED
        LOG("\nvertex %g: (%g,%g)\n", v->fID, v->fPoint.fX, v->fPoint.fY);
#endif
        Edge* leftEnclosingEdge = nullptr;
        Edge* rightEnclosingEdge = nullptr;
        find_enclosing_edges(v, &activeEdges, &leftEnclosingEdge, &rightEnclosingEdge);
        Poly* leftPoly = nullptr;
        Poly* rightPoly = nullptr;
        if (v->fFirstEdgeAbove) {
            leftPoly = v->fFirstEdgeAbove->fLeftPoly;
            rightPoly = v->fLastEdgeAbove->fRightPoly;
        } else {
            leftPoly = leftEnclosingEdge ? leftEnclosingEdge->fRightPoly : nullptr;
            rightPoly = rightEnclosingEdge ? rightEnclosingEdge->fLeftPoly : nullptr;
        }
#if LOGGING_ENABLED
        LOG("edges above:\n");
        for (Edge* e = v->fFirstEdgeAbove; e; e = e->fNextEdgeAbove) {
            LOG("%g -> %g, lpoly %d, rpoly %d\n", e->fTop->fID, e->fBottom->fID,
                e->fLeftPoly ? e->fLeftPoly->fID : -1, e->fRightPoly ? e->fRightPoly->fID : -1);
        }
        LOG("edges below:\n");
        for (Edge* e = v->fFirstEdgeBelow; e; e = e->fNextEdgeBelow) {
            LOG("%g -> %g, lpoly %d, rpoly %d\n", e->fTop->fID, e->fBottom->fID,
                e->fLeftPoly ? e->fLeftPoly->fID : -1, e->fRightPoly ? e->fRightPoly->fID : -1);
        }
#endif
        if (v->fFirstEdgeAbove) {
            if (leftPoly) {
                leftPoly = leftPoly->addEdge(v->fFirstEdgeAbove, Poly::kRight_Side, alloc);
            }
            if (rightPoly) {
                rightPoly = rightPoly->addEdge(v->fLastEdgeAbove, Poly::kLeft_Side, alloc);
            }
            for (Edge* e = v->fFirstEdgeAbove; e != v->fLastEdgeAbove; e = e->fNextEdgeAbove) {
                Edge* leftEdge = e;
                Edge* rightEdge = e->fNextEdgeAbove;
                assert(rightEdge->isRightOf(leftEdge->fTop));
                remove_edge(leftEdge, &activeEdges);
                if (leftEdge->fRightPoly) {
                    leftEdge->fRightPoly->addEdge(e, Poly::kLeft_Side, alloc);
                }
                if (rightEdge->fLeftPoly) {
                    rightEdge->fLeftPoly->addEdge(e, Poly::kRight_Side, alloc);
                }
            }
            remove_edge(v->fLastEdgeAbove, &activeEdges);
            if (!v->fFirstEdgeBelow) {
                if (leftPoly && rightPoly && leftPoly != rightPoly) {
                    assert(leftPoly->fPartner == nullptr && rightPoly->fPartner == nullptr);
                    rightPoly->fPartner = leftPoly;
                    leftPoly->fPartner = rightPoly;
                }
            }
        }
        if (v->fFirstEdgeBelow) {
            if (!v->fFirstEdgeAbove) {
                if (leftPoly && rightPoly) {
                    if (leftPoly == rightPoly) {
                        if (leftPoly->fTail && leftPoly->fTail->fSide == Poly::kLeft_Side) {
                            leftPoly = new_poly(&polys, leftPoly->lastVertex(),
                                                 leftPoly->fWinding, alloc);
                            leftEnclosingEdge->fRightPoly = leftPoly;
                        } else {
                            rightPoly = new_poly(&polys, rightPoly->lastVertex(),
                                                 rightPoly->fWinding, alloc);
                            rightEnclosingEdge->fLeftPoly = rightPoly;
                        }
                    }
                    Edge* join = ALLOC_NEW(Edge, (leftPoly->lastVertex(), v, 1), alloc);
                    leftPoly = leftPoly->addEdge(join, Poly::kRight_Side, alloc);
                    rightPoly = rightPoly->addEdge(join, Poly::kLeft_Side, alloc);
                }
            }
            Edge* leftEdge = v->fFirstEdgeBelow;
            leftEdge->fLeftPoly = leftPoly;
            insert_edge(leftEdge, leftEnclosingEdge, &activeEdges);
            for (Edge* rightEdge = leftEdge->fNextEdgeBelow; rightEdge;
                 rightEdge = rightEdge->fNextEdgeBelow) {
                insert_edge(rightEdge, leftEdge, &activeEdges);
                int winding = leftEdge->fLeftPoly ? leftEdge->fLeftPoly->fWinding : 0;
                winding += leftEdge->fWinding;
                if (winding != 0) {
                    Poly* poly = new_poly(&polys, v, winding, alloc);
                    leftEdge->fRightPoly = rightEdge->fLeftPoly = poly;
                }
                leftEdge = rightEdge;
            }
            v->fLastEdgeBelow->fRightPoly = rightPoly;
        }
#if LOGGING_ENABLED
        LOG("\nactive edges:\n");
        for (Edge* e = activeEdges.fHead; e != nullptr; e = e->fRight) {
            LOG("%g -> %g, lpoly %d, rpoly %d\n", e->fTop->fID, e->fBottom->fID,
                e->fLeftPoly ? e->fLeftPoly->fID : -1, e->fRightPoly ? e->fRightPoly->fID : -1);
        }
#endif
    }
    return polys;
}


// This is a driver function which calls stages 2-5 in turn.

Vertex* contours_to_mesh(Vertex** contours, int contourCnt,
                         Comparator& c, Allocator& alloc) {
#if LOGGING_ENABLED
    for (int i = 0; i < contourCnt; ++i) {
        Vertex* v = contours[i];
        assert(v);
        LOG("path.moveTo(%20.20g, %20.20g);\n", v->fPoint.fX, v->fPoint.fY);
        for (v = v->fNext; v != contours[i]; v = v->fNext) {
            LOG("path.lineTo(%20.20g, %20.20g);\n", v->fPoint.fX, v->fPoint.fY);
        }
    }
#endif
    sanitize_contours(contours, contourCnt);
    return build_edges(contours, contourCnt, c, alloc);
}

Poly* mesh_to_polys(Vertex** vertices, Comparator& c, Allocator& alloc) {
    if (!vertices || !*vertices) {
        return nullptr;
    }

    // Sort vertices in Y (secondarily in X).
    merge_sort(vertices, c);
    merge_coincident_vertices(vertices, c);
#if LOGGING_ENABLED
    for (Vertex* v = *vertices; v != nullptr; v = v->fNext) {
        static float gID = 0.0f;
        v->fID = gID++;
    }
#endif
    simplify(*vertices, c, alloc);
    return tessellate(*vertices, alloc);
}

Poly* contours_to_polys(Vertex** contours, int contourCnt,
                        const fastuidraw::dvec2& wh,
                        Allocator& alloc) {
    Comparator c;
    if (wh.x() > wh.y()) {
        c.sweep_lt = sweep_lt_horiz;
        c.sweep_gt = sweep_gt_horiz;
    } else {
        c.sweep_lt = sweep_lt_vert;
        c.sweep_gt = sweep_gt_vert;
    }
    Vertex* mesh = contours_to_mesh(contours, contourCnt, c, alloc);
    Poly* polys = mesh_to_polys(&mesh, c, alloc);
    return polys;
}


  class TessellatorPrivate
  {
  public:
    explicit
    TessellatorPrivate(fastuidraw::Tessellator *p,
                       const fastuidraw::dvec2 &min_bb,
                       const fastuidraw::dvec2 &max_bb):
      m_p(p),
      m_min_bb(min_bb),
      m_max_bb(max_bb)
    {}

    void
    begin_contour(void);

    void
    add_vertex(const fastuidraw::Tessellator::point &v);

    void
    end_contour(void);

    void
    generate_triangles(void);

  private:
    class Builder
    {
    public:
      Builder(void):
        head(nullptr),
        prev(nullptr)
      {}

      Vertex *head, *prev;
      std::vector<Vertex*> m_contours;
    };

    fastuidraw::Tessellator *m_p;

    Builder m_builder;
    Allocator m_alloc;
    fastuidraw::dvec2 m_min_bb, m_max_bb;
  };

} // namespace

//////////////////////////////////////////
// TessellatorPrivate methods
void
TessellatorPrivate::
generate_triangles(void)
{
  //to polygons
  Poly *poly;
  poly = contours_to_polys(&m_builder.m_contours[0],
                           m_builder.m_contours.size(),
                           m_max_bb - m_min_bb, m_alloc);
  for(; poly; poly = poly->fNext)
    {
      poly->emit(m_p);
    }
}

void
TessellatorPrivate::
begin_contour(void)
{
}

void
TessellatorPrivate::
end_contour(void)
{
  if(m_builder.head)
    {
      m_builder.head->fPrev = m_builder.prev;
      m_builder.prev->fNext = m_builder.head;
      m_builder.m_contours.push_back(m_builder.head);
    }
  m_builder.head = m_builder.prev = nullptr;
}

void
TessellatorPrivate::
add_vertex(const fastuidraw::Tessellator::point &v)
{
  m_builder.prev = append_point_to_contour(v, m_builder.prev, &m_builder.head, m_alloc);
}

//////////////////////////////////////////////////
// fastuidraw::Tessellator methods
fastuidraw::Tessellator::
Tessellator(const dvec2 &min_bb,
            const dvec2 &max_bb)
{
  m_d = FASTUIDRAWnew TessellatorPrivate(this, min_bb, max_bb);
}

fastuidraw::Tessellator::
~Tessellator()
{
  TessellatorPrivate *d;
  d = static_cast<TessellatorPrivate*>(m_d);
  FASTUIDRAWdelete(d);
}

void
fastuidraw::Tessellator::
generate_triangles(void)
{
  TessellatorPrivate *d;
  d = static_cast<TessellatorPrivate*>(m_d);
  d->generate_triangles();
}

void
fastuidraw::Tessellator::
begin_contour(void)
{
  TessellatorPrivate *d;
  d = static_cast<TessellatorPrivate*>(m_d);
  d->begin_contour();
}

void
fastuidraw::Tessellator::
end_contour(void)
{
  TessellatorPrivate *d;
  d = static_cast<TessellatorPrivate*>(m_d);
  d->end_contour();
}

void
fastuidraw::Tessellator::
add_vertex(const point &v)
{
  TessellatorPrivate *d;
  d = static_cast<TessellatorPrivate*>(m_d);
  d->add_vertex(v);
}
