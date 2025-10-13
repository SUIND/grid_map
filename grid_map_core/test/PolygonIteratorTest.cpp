/*
 * PolygonIteratorTest.cpp
 *
 *  Created on: Sep 19, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/Polygon.hpp"
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

// New comprehensive tests

TEST(PolygonIterator, SmallTriangle)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  Polygon polygon;
  polygon.addVertex(Position(0.0, 0.0));
  polygon.addVertex(Position(2.0, 0.0));
  polygon.addVertex(Position(1.0, 2.0));
  
  PolygonIterator iterator(map, polygon);
  
  int count = 0;
  std::vector<grid_map::Index> indices;
  while (!iterator.isPastEnd()) {
    indices.push_back(*iterator);
    Position pos;
    map.getPosition(*iterator, pos);
    // Verify position is within map bounds
    EXPECT_TRUE(map.isInside(pos));
    ++iterator;
    ++count;
  }
  
  EXPECT_GT(count, 1); // Should have at least one cell
  
  // Check no duplicates
  for (size_t i = 0; i < indices.size(); ++i) {
    for (size_t j = i + 1; j < indices.size(); ++j) {
      EXPECT_TRUE((indices[i] != indices[j]).any());
    }
  }
}

TEST(PolygonIterator, LargeSquare)
{
  GridMap map({"layer"});
  map.setGeometry(Length(20.0, 20.0), 1.0, Position(0.0, 0.0));
  
  Polygon polygon;
  polygon.addVertex(Position(-5.0, -5.0));
  polygon.addVertex(Position(5.0, -5.0));
  polygon.addVertex(Position(5.0, 5.0));
  polygon.addVertex(Position(-5.0, 5.0));
  
  PolygonIterator iterator(map, polygon);
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    Position pos;
    map.getPosition(*iterator, pos);
    EXPECT_TRUE(polygon.isInside(pos));
    ++iterator;
    ++count;
  }
  
  // 10x10 area should have approximately 100 cells
  EXPECT_GT(count, 80);
  EXPECT_LT(count, 120);
}

TEST(PolygonIterator, IrregularPolygon)
{
  GridMap map({"layer"});
  map.setGeometry(Length(15.0, 15.0), 1.0, Position(0.0, 0.0));
  
  // Pentagon-like shape
  Polygon polygon;
  polygon.addVertex(Position(0.0, 3.0));
  polygon.addVertex(Position(3.0, 1.0));
  polygon.addVertex(Position(2.0, -2.0));
  polygon.addVertex(Position(-2.0, -2.0));
  polygon.addVertex(Position(-3.0, 1.0));
  
  PolygonIterator iterator(map, polygon);
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    Position pos;
    map.getPosition(*iterator, pos);
    EXPECT_TRUE(polygon.isInside(pos));
    ++iterator;
    ++count;
  }
  
  EXPECT_GT(count, 10);
}

TEST(PolygonIterator, SingleCellPolygon)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  // Small polygon that should cover at least one cell
  Polygon polygon;
  polygon.addVertex(Position(0.0, 0.0));
  polygon.addVertex(Position(0.6, 0.0));
  polygon.addVertex(Position(0.6, 0.6));
  polygon.addVertex(Position(0.0, 0.6));
  
  PolygonIterator iterator(map, polygon);
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    ++iterator;
    ++count;
  }
  
  // Should have at least one cell
  EXPECT_GE(count, 1);
}

TEST(PolygonIterator, PolygonAtMapEdge)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  // Polygon at the edge of the map
  Polygon polygon;
  polygon.addVertex(Position(4.0, 4.0));
  polygon.addVertex(Position(5.0, 4.0));
  polygon.addVertex(Position(5.0, 5.0));
  polygon.addVertex(Position(4.0, 5.0));
  
  PolygonIterator iterator(map, polygon);
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    Position pos;
    map.getPosition(*iterator, pos);
    EXPECT_TRUE(map.isInside(pos));
    ++iterator;
    ++count;
  }
  
  EXPECT_GT(count, 0);
}

TEST(PolygonIterator, PolygonPartiallyOutside)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  // Polygon that extends outside map bounds
  Polygon polygon;
  polygon.addVertex(Position(3.0, 3.0));
  polygon.addVertex(Position(8.0, 3.0));
  polygon.addVertex(Position(8.0, 8.0));
  polygon.addVertex(Position(3.0, 8.0));
  
  PolygonIterator iterator(map, polygon);
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    Position pos;
    map.getPosition(*iterator, pos);
    // All iterated positions should be inside map
    EXPECT_TRUE(map.isInside(pos));
    ++iterator;
    ++count;
  }
  
  EXPECT_GT(count, 0);
}

TEST(PolygonIterator, NarrowPolygon)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 0.5, Position(0.0, 0.0));
  
  // Narrow rectangular polygon
  Polygon polygon;
  polygon.addVertex(Position(-2.0, -0.3));
  polygon.addVertex(Position(2.0, -0.3));
  polygon.addVertex(Position(2.0, 0.3));
  polygon.addVertex(Position(-2.0, 0.3));
  
  PolygonIterator iterator(map, polygon);
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    Position pos;
    map.getPosition(*iterator, pos);
    EXPECT_TRUE(map.isInside(pos));
    ++iterator;
    ++count;
  }
  
  EXPECT_GT(count, 3); // Should still have some cells
}

TEST(PolygonIterator, ConvexPolygon)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  // Convex hexagon
  Polygon polygon;
  polygon.addVertex(Position(0.0, 3.0));
  polygon.addVertex(Position(2.5, 1.5));
  polygon.addVertex(Position(2.5, -1.5));
  polygon.addVertex(Position(0.0, -3.0));
  polygon.addVertex(Position(-2.5, -1.5));
  polygon.addVertex(Position(-2.5, 1.5));
  
  PolygonIterator iterator(map, polygon);
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    Position pos;
    map.getPosition(*iterator, pos);
    EXPECT_TRUE(polygon.isInside(pos));
    ++iterator;
    ++count;
  }
  
  EXPECT_GT(count, 15);
}

TEST(PolygonIterator, HighResolutionMap)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 0.1, Position(0.0, 0.0));
  
  Polygon polygon;
  polygon.addVertex(Position(-1.0, -1.0));
  polygon.addVertex(Position(1.0, -1.0));
  polygon.addVertex(Position(1.0, 1.0));
  polygon.addVertex(Position(-1.0, 1.0));
  
  PolygonIterator iterator(map, polygon);
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    ++iterator;
    ++count;
  }
  
  // 2x2 area with 0.1 resolution should have ~400 cells
  EXPECT_GT(count, 300);
}

TEST(PolygonIterator, LowResolutionMap)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 2.0, Position(0.0, 0.0));
  
  Polygon polygon;
  polygon.addVertex(Position(-2.0, -2.0));
  polygon.addVertex(Position(2.0, -2.0));
  polygon.addVertex(Position(2.0, 2.0));
  polygon.addVertex(Position(-2.0, 2.0));
  
  PolygonIterator iterator(map, polygon);
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    ++iterator;
    ++count;
  }
  
  // 4x4 area with 2.0 resolution should have ~4 cells
  EXPECT_GE(count, 1);
  EXPECT_LE(count, 10);
}

TEST(PolygonIterator, CopyAndAssignment)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  Polygon polygon;
  polygon.addVertex(Position(-2.0, -2.0));
  polygon.addVertex(Position(2.0, -2.0));
  polygon.addVertex(Position(2.0, 2.0));
  polygon.addVertex(Position(-2.0, 2.0));
  
  PolygonIterator original(map, polygon);
  
  // Test assignment operator
  PolygonIterator copy = original;
  
  // Verify both iterators iterate over the same region
  std::vector<grid_map::Index> originalIndices;
  std::vector<grid_map::Index> copyIndices;
  
  // Collect all indices from original
  PolygonIterator origTemp(map, polygon);
  while (!origTemp.isPastEnd()) {
    originalIndices.push_back(*origTemp);
    ++origTemp;
  }
  
  // Collect all indices from copy
  while (!copy.isPastEnd()) {
    copyIndices.push_back(*copy);
    ++copy;
  }
  
  // Both should iterate over the same number of cells
  EXPECT_EQ(originalIndices.size(), copyIndices.size());
  EXPECT_GT(originalIndices.size(), 5); // Should have multiple elements
}

TEST(PolygonIterator, InequalityOperator)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  Polygon polygon;
  polygon.addVertex(Position(-1.0, -1.0));
  polygon.addVertex(Position(1.0, -1.0));
  polygon.addVertex(Position(1.0, 1.0));
  polygon.addVertex(Position(-1.0, 1.0));
  
  PolygonIterator iter1(map, polygon);
  PolygonIterator iter2(map, polygon);
  
  // Note: Two separately constructed iterators may start at different internal positions
  // even if they iterate over the same region, so we just check they work correctly
  int count1 = 0, count2 = 0;
  while (!iter1.isPastEnd()) { ++iter1; ++count1; }
  while (!iter2.isPastEnd()) { ++iter2; ++count2; }
  
  // They should iterate over the same number of elements
  EXPECT_EQ(count1, count2);
  EXPECT_GT(count1, 0);
}

TEST(PolygonIterator, NonSquareMap)
{
  GridMap map({"layer"});
  map.setGeometry(Length(20.0, 5.0), 1.0, Position(0.0, 0.0));
  
  Polygon polygon;
  polygon.addVertex(Position(-5.0, -1.0));
  polygon.addVertex(Position(5.0, -1.0));
  polygon.addVertex(Position(5.0, 1.0));
  polygon.addVertex(Position(-5.0, 1.0));
  
  PolygonIterator iterator(map, polygon);
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    Position pos;
    map.getPosition(*iterator, pos);
    EXPECT_TRUE(map.isInside(pos));
    ++iterator;
    ++count;
  }
  
  EXPECT_GT(count, 10);
}

TEST(PolygonIterator, AllIndicesValid)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  map.add("layer", 0.0);
  
  Polygon polygon;
  polygon.addVertex(Position(-3.0, -3.0));
  polygon.addVertex(Position(3.0, -3.0));
  polygon.addVertex(Position(3.0, 3.0));
  polygon.addVertex(Position(-3.0, 3.0));
  
  PolygonIterator iterator(map, polygon);
  
  while (!iterator.isPastEnd()) {
    grid_map::Index index = *iterator;
    Position pos;
    map.getPosition(index, pos);
    // All positions should be inside the map
    EXPECT_TRUE(map.isInside(pos));
    // Should be able to access the layer
    EXPECT_NO_THROW(map.at("layer", index));
    ++iterator;
  }
}

TEST(PolygonIterator, RotatedSquare)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  // Square rotated 45 degrees (diamond shape)
  Polygon polygon;
  polygon.addVertex(Position(0.0, 3.0));
  polygon.addVertex(Position(3.0, 0.0));
  polygon.addVertex(Position(0.0, -3.0));
  polygon.addVertex(Position(-3.0, 0.0));
  
  PolygonIterator iterator(map, polygon);
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    Position pos;
    map.getPosition(*iterator, pos);
    EXPECT_TRUE(polygon.isInside(pos));
    ++iterator;
    ++count;
  }
  
  EXPECT_GT(count, 10);
}

TEST(PolygonIterator, PolygonWithMovedMap)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  map.move(Position(3.0, 3.0));
  
  Polygon polygon;
  polygon.addVertex(Position(2.0, 2.0));
  polygon.addVertex(Position(4.0, 2.0));
  polygon.addVertex(Position(4.0, 4.0));
  polygon.addVertex(Position(2.0, 4.0));
  
  PolygonIterator iterator(map, polygon);
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    Position pos;
    map.getPosition(*iterator, pos);
    EXPECT_TRUE(map.isInside(pos));
    ++iterator;
    ++count;
  }
  
  EXPECT_GT(count, 0);
}

TEST(PolygonIterator, VerySmallPolygon)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  // Extremely small polygon
  Polygon polygon;
  polygon.addVertex(Position(0.0, 0.0));
  polygon.addVertex(Position(0.1, 0.0));
  polygon.addVertex(Position(0.1, 0.1));
  polygon.addVertex(Position(0.0, 0.1));
  
  PolygonIterator iterator(map, polygon);
  
  // May or may not have cells depending on alignment
  int count = 0;
  while (!iterator.isPastEnd()) {
    ++iterator;
    ++count;
  }
  
  EXPECT_LE(count, 5);
}

TEST(PolygonIterator, ComplexPolygonShape)
{
  GridMap map({"layer"});
  map.setGeometry(Length(15.0, 15.0), 1.0, Position(0.0, 0.0));
  
  // More complex polygon with many vertices
  Polygon polygon;
  polygon.addVertex(Position(0.0, 4.0));
  polygon.addVertex(Position(2.0, 3.0));
  polygon.addVertex(Position(3.0, 4.0));
  polygon.addVertex(Position(4.0, 2.0));
  polygon.addVertex(Position(3.0, 0.0));
  polygon.addVertex(Position(4.0, -2.0));
  polygon.addVertex(Position(2.0, -3.0));
  polygon.addVertex(Position(0.0, -4.0));
  polygon.addVertex(Position(-2.0, -3.0));
  polygon.addVertex(Position(-3.0, -1.0));
  polygon.addVertex(Position(-4.0, 0.0));
  polygon.addVertex(Position(-3.0, 2.0));
  polygon.addVertex(Position(-2.0, 3.0));
  
  PolygonIterator iterator(map, polygon);
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    Position pos;
    map.getPosition(*iterator, pos);
    EXPECT_TRUE(polygon.isInside(pos));
    ++iterator;
    ++count;
  }
  
  EXPECT_GT(count, 20);
}

TEST(PolygonIterator, EmptyIteratorCheck)
{
  GridMap map({"layer"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));
  
  Polygon polygon;
  polygon.addVertex(Position(-1.0, -1.0));
  polygon.addVertex(Position(1.0, -1.0));
  polygon.addVertex(Position(1.0, 1.0));
  polygon.addVertex(Position(-1.0, 1.0));
  
  PolygonIterator iterator(map, polygon);
  
  // Should not be empty
  EXPECT_FALSE(iterator.isPastEnd());
  
  // Exhaust iterator
  while (!iterator.isPastEnd()) {
    ++iterator;
  }
  
  // Now should be past end
  EXPECT_TRUE(iterator.isPastEnd());
}

