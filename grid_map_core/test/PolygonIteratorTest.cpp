/*
 * PolygonIteratorTest.cpp
 *
 *  Created on: Sep 19, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/GridMapMath.hpp"
#include "grid_map_core/Polygon.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "grid_map_core/iterators/PolygonIterator.hpp"

// Eigen
#include <Eigen/Core>

// gtest
#include <gtest/gtest.h>

// Limits
#include <cfloat>

// Vector
#include <vector>

using namespace std;
using namespace Eigen;
using namespace grid_map;

TEST(PolygonIterator, FullCover)
{
  vector<string> types;
  types.push_back("type");
  GridMap map(types);
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  Polygon polygon;
  polygon.addVertex(Position(-100.0, 100.0));
  polygon.addVertex(Position(100.0, 100.0));
  polygon.addVertex(Position(100.0, -100.0));
  polygon.addVertex(Position(-100.0, -100.0));

  PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  for (int i = 0; i < 37; ++i) ++iterator;

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(PolygonIterator, Outside)
{
  GridMap map({"types"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  Polygon polygon;
  polygon.addVertex(Position(99.0, 101.0));
  polygon.addVertex(Position(101.0, 101.0));
  polygon.addVertex(Position(101.0, 99.0));
  polygon.addVertex(Position(99.0, 99.0));

  PolygonIterator iterator(map, polygon);

  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(PolygonIterator, Square)
{
  GridMap map({"types"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  Polygon polygon;
  polygon.addVertex(Position(-1.0, 1.5));
  polygon.addVertex(Position(1.0, 1.5));
  polygon.addVertex(Position(1.0, -1.5));
  polygon.addVertex(Position(-1.0, -1.5));

  PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(PolygonIterator, TopLeftTriangle)
{
  GridMap map({"types"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  Polygon polygon;
  polygon.addVertex(Position(-40.1, 20.6));
  polygon.addVertex(Position(40.1, 20.4));
  polygon.addVertex(Position(-40.1, -20.6));

  PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(1, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  // TODO Extend.
}

TEST(PolygonIterator, MoveMap)
{
  GridMap map({"layer"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)
  map.move(Position(2.0, 0.0));

  Polygon polygon;
  polygon.addVertex(Position(6.1, 1.6));
  polygon.addVertex(Position(0.9, 1.6));
  polygon.addVertex(Position(0.9, -1.6));
  polygon.addVertex(Position(6.1, -1.6));
  PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  for (int i = 0; i < 4; ++i) ++iterator;

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  for (int i = 0; i < 8; ++i) ++iterator;

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(2, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

/*
 * Regression tests for an empty / out-of-range PolygonIterator.
 *
 * When a polygon overhangs a map edge, PolygonIterator::findSubmapParameters() bounds the
 * offending corner back into the map and converts it to an index. That conversion can yield an
 * index equal to the buffer size (one past the last cell) even though the bounded position is
 * inside the map. On a moved map (non-zero buffer start index) that index is silently wrapped to
 * the opposite edge of the ring buffer, which makes the submap size negative and leaves the
 * iterator empty. On a map that was never moved the index simply stays out of range and the
 * iterator hands indices outside the buffer to the caller.
 */

//! Brute force reference: every cell of the map whose center lies inside the polygon.
static std::vector<grid_map::Index> cellsInsidePolygon(const GridMap& map, const Polygon& polygon)
{
  std::vector<grid_map::Index> cells;
  for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
  {
    Position position;
    map.getPosition(*iterator, position);
    if (polygon.isInside(position)) cells.push_back(*iterator);
  }
  return cells;
}

TEST(PolygonIterator, MovedMapWithPolygonOverhangingMapEdge)
{
  GridMap map({"layer"});
  map.setGeometry(Length(33.746855726960831, 9.9433593878904709), 0.25,
                  Position(10.795406602280387, -22.994307697034245));
  map.move(Position(24.953846346189515, -38.957489169334572));
  ASSERT_FALSE((map.getStartIndex() == 0).all()) << "this test requires a moved (circular buffer) map";

  // Polygon overhanging the lower x and y edges of the map.
  Polygon polygon;
  polygon.addVertex(Position(13.526077842487485, -38.044727225921832));
  polygon.addVertex(Position(13.526077842487485, -47.214226670855084));
  polygon.addVertex(Position(2.3031839924109754, -47.214226670855084));
  polygon.addVertex(Position(2.3031839924109754, -38.044727225921832));

  const std::vector<grid_map::Index> expected = cellsInsidePolygon(map, polygon);
  ASSERT_FALSE(expected.empty()) << "the polygon must overlap the map for this test to mean anything";

  std::vector<grid_map::Index> visited;
  for (PolygonIterator iterator(map, polygon); !iterator.isPastEnd(); ++iterator) visited.push_back(*iterator);

  EXPECT_EQ(expected.size(), visited.size());
  for (size_t i = 0; i < std::min(expected.size(), visited.size()); ++i)
  {
    EXPECT_TRUE((expected.at(i) == visited.at(i)).all())
        << "cell " << i << ": expected (" << expected.at(i)(0) << ", " << expected.at(i)(1) << "), got ("
        << visited.at(i)(0) << ", " << visited.at(i)(1) << ")";
  }
}

/*
 * The same defect on a map that was never moved does not wrap the index; it leaves it out of range,
 * so PolygonIterator returns indices outside the buffer and PolygonIterator::isInside() reads an
 * uninitialised Position. That is undefined behaviour and therefore not asserted here - see
 * boundPositionToRange.PositionBelowLowerEdgeIsBoundedIntoTheMap in GridMapMathTest.cpp for the
 * deterministic form of that case.
 */
