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

// Algorithm
#include <algorithm>

// Pair
#include <utility>

// Random
#include <random>

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

/*!
 * Brute force reference: every cell of the map whose center lies inside the polygon. Sorted,
 * because GridMapIterator walks the buffer column by column while PolygonIterator walks the submap
 * row by row, so only the set of visited cells is comparable, not the order.
 */
static std::vector<std::pair<int, int>> cellsInsidePolygon(const GridMap& map, const Polygon& polygon)
{
  std::vector<std::pair<int, int>> cells;
  for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
  {
    Position position;
    map.getPosition(*iterator, position);
    if (polygon.isInside(position)) cells.emplace_back((*iterator)(0), (*iterator)(1));
  }
  std::sort(cells.begin(), cells.end());
  return cells;
}

//! The cells visited by a PolygonIterator, sorted so they can be compared with the reference.
static std::vector<std::pair<int, int>> cellsVisitedByIterator(const GridMap& map, const Polygon& polygon)
{
  std::vector<std::pair<int, int>> cells;
  for (PolygonIterator iterator(map, polygon); !iterator.isPastEnd(); ++iterator)
  {
    cells.emplace_back((*iterator)(0), (*iterator)(1));
  }
  std::sort(cells.begin(), cells.end());
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

  const std::vector<std::pair<int, int>> expected = cellsInsidePolygon(map, polygon);
  ASSERT_FALSE(expected.empty()) << "the polygon must overlap the map for this test to mean anything";

  const std::vector<std::pair<int, int>> visited = cellsVisitedByIterator(map, polygon);

  EXPECT_EQ(expected.size(), visited.size());
  EXPECT_TRUE(expected == visited) << "the iterator visited a different set of cells than the polygon covers";
}

/*
 * The same defect on a map that was never moved does not wrap the index; it leaves it out of range,
 * so PolygonIterator returns indices outside the buffer and PolygonIterator::isInside() reads an
 * uninitialised Position. That is undefined behaviour and therefore not asserted here - see
 * boundPositionToRange.PositionBelowLowerEdgeIsBoundedIntoTheMap in GridMapMathTest.cpp for the
 * deterministic form of that case.
 */

/*
 * Sweep of maps and polygons that straddle a map edge, checked against a brute force scan of the
 * whole map. This is the general form of PolygonIterator.MovedMapWithPolygonOverhangingMapEdge:
 * the failure only appears for map positions and polygon corners whose floating point
 * representation is unlucky, so a sweep is the only way to cover it properly. With these
 * parameters roughly 0.7% of the overlapping cases used to come back empty.
 */
TEST(PolygonIterator, MatchesBruteForceForPolygonsStraddlingMapEdges)
{
  std::mt19937 randomNumberGenerator(20250727);
  std::uniform_real_distribution<double> mapCentre(-50.0, 50.0);
  std::uniform_real_distribution<double> mapSide(20.0, 40.0);
  const double resolution = 0.5;

  size_t comparisons = 0;
  size_t emptyIterators = 0;
  size_t wrongCellSets = 0;
  size_t indicesOutsideBuffer = 0;

  for (int trial = 0; trial < 1200; ++trial)
  {
    const Length length(mapSide(randomNumberGenerator), mapSide(randomNumberGenerator));
    GridMap map({"layer"});
    map.setGeometry(length, resolution, Position(mapCentre(randomNumberGenerator), mapCentre(randomNumberGenerator)));
    if (trial % 4 != 0)
    {
      // Move the map so that the circular buffer start index is not at its default position.
      std::uniform_real_distribution<double> moveBy(-0.5 * length(0), 0.5 * length(0));
      map.move(map.getPosition() + Position(moveBy(randomNumberGenerator), moveBy(randomNumberGenerator)));
    }

    // A box centred anywhere near the map, so that it often overhangs one or more map edges.
    std::uniform_real_distribution<double> offsetX(-0.7 * length(0), 0.7 * length(0));
    std::uniform_real_distribution<double> offsetY(-0.7 * length(1), 0.7 * length(1));
    std::uniform_real_distribution<double> halfSide(0.3, 0.6 * std::min(length(0), length(1)));
    const Position centre =
        map.getPosition() + Position(offsetX(randomNumberGenerator), offsetY(randomNumberGenerator));
    const double halfX = halfSide(randomNumberGenerator);
    const double halfY = halfSide(randomNumberGenerator);
    Polygon polygon;
    polygon.addVertex(centre + Position(halfX, halfY));
    polygon.addVertex(centre + Position(halfX, -halfY));
    polygon.addVertex(centre + Position(-halfX, -halfY));
    polygon.addVertex(centre + Position(-halfX, halfY));

    const std::vector<std::pair<int, int>> expected = cellsInsidePolygon(map, polygon);
    if (expected.empty()) continue;
    ++comparisons;

    for (PolygonIterator iterator(map, polygon); !iterator.isPastEnd(); ++iterator)
    {
      if (!checkIfIndexInRange(*iterator, map.getSize())) ++indicesOutsideBuffer;
    }
    const std::vector<std::pair<int, int>> visited = cellsVisitedByIterator(map, polygon);

    if (visited.empty())
      ++emptyIterators;
    else if (visited != expected)
      ++wrongCellSets;
  }

  EXPECT_GT(comparisons, 500u) << "the sweep did not produce enough overlapping cases to be meaningful";
  EXPECT_EQ(0u, emptyIterators) << "of " << comparisons << " polygons overlapping the map";
  EXPECT_EQ(0u, wrongCellSets) << "of " << comparisons << " polygons overlapping the map";
  EXPECT_EQ(0u, indicesOutsideBuffer);
}
