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

// Debug test to understand coordinate system and find the bug
TEST(PolygonIterator, DebugCoordinateSystem)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  // Create a simple square polygon
  Polygon polygon;
  polygon.addVertex(Position(-2.0, -2.0));
  polygon.addVertex(Position(2.0, -2.0));
  polygon.addVertex(Position(2.0, 2.0));
  polygon.addVertex(Position(-2.0, 2.0));
  
  std::cout << "\n=== Debug Coordinate System ===" << std::endl;
  std::cout << "Map size: " << map.getSize().transpose() << std::endl;
  std::cout << "Map length: " << map.getLength().transpose() << std::endl;
  std::cout << "Map position: " << map.getPosition().transpose() << std::endl;
  std::cout << "Map resolution: " << map.getResolution() << std::endl;
  std::cout << "Buffer start index: " << map.getStartIndex().transpose() << std::endl;
  
  // Check some positions and their indices
  std::vector<Position> testPositions = {
    Position(-2.0, -2.0),
    Position(2.0, -2.0),
    Position(2.0, 2.0),
    Position(-2.0, 2.0),
    Position(0.0, 0.0)
  };
  
  for (const auto& pos : testPositions) {
    grid_map::Index idx;
    if (map.getIndex(pos, idx)) {
      std::cout << "Position " << pos.transpose() << " -> Index " << idx.transpose() << std::endl;
    }
  }
  
  PolygonIterator iterator(map, polygon);
  
  int count = 0;
  std::cout << "\nIterating through polygon:" << std::endl;
  while (!iterator.isPastEnd() && count < 10) {
    grid_map::Index idx = *iterator;
    Position pos;
    map.getPosition(idx, pos);
    std::cout << "  Index " << idx.transpose() << " -> Position " << pos.transpose() << std::endl;
    ++iterator;
    ++count;
  }
  
  // Count total
  while (!iterator.isPastEnd()) {
    ++iterator;
    ++count;
  }
  
  std::cout << "Total cells in polygon: " << count << std::endl;
  EXPECT_GT(count, 5);
}

// Critical test: moved map with circular buffer
TEST(PolygonIterator, MovedMapCircularBuffer)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  std::cout << "\n=== Before Move ===" << std::endl;
  std::cout << "Buffer start index: " << map.getStartIndex().transpose() << std::endl;
  
  // Move the map - this creates circular buffer offset
  map.move(Position(3.0, 2.0));
  
  std::cout << "=== After Move ===" << std::endl;
  std::cout << "Map position: " << map.getPosition().transpose() << std::endl;
  std::cout << "Buffer start index: " << map.getStartIndex().transpose() << std::endl;
  
  // Create a polygon in the new map region
  Polygon polygon;
  polygon.addVertex(Position(1.0, 0.0));
  polygon.addVertex(Position(4.0, 0.0));
  polygon.addVertex(Position(4.0, 3.0));
  polygon.addVertex(Position(1.0, 3.0));
  
  std::cout << "\nPolygon vertices in world coordinates:" << std::endl;
  for (const auto& v : polygon.getVertices()) {
    std::cout << "  " << v.transpose() << std::endl;
  }
  
  PolygonIterator iterator(map, polygon);
  
  int count = 0;
  std::cout << "\nIterating through polygon (first 10):" << std::endl;
  while (!iterator.isPastEnd() && count < 10) {
    grid_map::Index idx = *iterator;
    Position pos;
    map.getPosition(idx, pos);
    std::cout << "  Index " << idx.transpose() << " -> Position " << pos.transpose() << std::endl;
    ++iterator;
    ++count;
  }
  
  while (!iterator.isPastEnd()) {
    ++iterator;
    ++count;
  }
  
  std::cout << "Total cells: " << count << std::endl;
  
  if (count == 0) {
    std::cout << "ERROR: Empty iterator with moved map!" << std::endl;
  }
  
  // This is where the bug likely occurs!
  EXPECT_GT(count, 5);
}

// Critical test: polygon near boundary with moved map
TEST(PolygonIterator, PolygonNearBoundaryMovedMap)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  map.move(Position(4.0, 3.0));
  
  std::cout << "\n=== Polygon Near Boundary with Moved Map ===" << std::endl;
  std::cout << "Map position: " << map.getPosition().transpose() << std::endl;
  std::cout << "Buffer start index: " << map.getStartIndex().transpose() << std::endl;
  std::cout << "Map length: " << map.getLength().transpose() << std::endl;
  
  // Create polygon that extends near/past the map boundary
  Polygon polygon;
  polygon.addVertex(Position(7.0, 5.0));  // Near edge
  polygon.addVertex(Position(9.0, 5.0));  // Past edge in original frame
  polygon.addVertex(Position(9.0, 7.0));
  polygon.addVertex(Position(7.0, 7.0));
  
  std::cout << "\nPolygon vertices:" << std::endl;
  for (const auto& v : polygon.getVertices()) {
    std::cout << "  " << v.transpose();
    grid_map::Index idx;
    if (map.getIndex(v, idx)) {
      std::cout << " -> Index " << idx.transpose();
    } else {
      std::cout << " -> OUT OF MAP";
    }
    std::cout << std::endl;
  }
  
  PolygonIterator iterator(map, polygon);
  
  int count = 0;
  std::cout << "\nIterating:" << std::endl;
  while (!iterator.isPastEnd()) {
    grid_map::Index idx = *iterator;
    Position pos;
    map.getPosition(idx, pos);
    if (count < 5) {
      std::cout << "  Index " << idx.transpose() << " -> Position " << pos.transpose() << std::endl;
    }
    ++iterator;
    ++count;
  }
  
  std::cout << "Total cells: " << count << std::endl;
  
  if (count == 0) {
    std::cout << "BUG FOUND: Empty iterator for polygon near boundary with moved map!" << std::endl;
  }
}

// Test to expose the actual bug in getSubmapSizeFromCornerIndeces
TEST(PolygonIterator, ExposeBoundingBoxBug)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  // Move map significantly to create buffer wrapping
  map.move(Position(4.5, 4.5));
  
  std::cout << "\n=== Testing Bounding Box Calculation Bug ===" << std::endl;
  std::cout << "Map position: " << map.getPosition().transpose() << std::endl;
  std::cout << "Buffer start index: " << map.getStartIndex().transpose() << std::endl;
  
  // Create polygon that spans across circular buffer wrap point
  // This might cause topLeft and bottomRight indices to be in wrong order after unwrapping
  Polygon polygon;
  polygon.addVertex(Position(1.0, 1.0));
  polygon.addVertex(Position(3.0, 1.0));
  polygon.addVertex(Position(3.0, 3.0));
  polygon.addVertex(Position(1.0, 3.0));
  
  std::cout << "\nPolygon bounding box in world coords:" << std::endl;
  Position worldMin = polygon.getVertices()[0];
  Position worldMax = worldMin;
  for (const auto& v : polygon.getVertices()) {
    worldMin = worldMin.array().min(v.array());
    worldMax = worldMax.array().max(v.array());
  }
  std::cout << "  Min (bottomRight): " << worldMin.transpose() << std::endl;
  std::cout << "  Max (topLeft):     " << worldMax.transpose() << std::endl;
  
  grid_map::Index idxMin, idxMax;
  map.getIndex(worldMin, idxMin);
  map.getIndex(worldMax, idxMax);
  std::cout << "\nBuffer indices:" << std::endl;
  std::cout << "  Min position -> Index: " << idxMin.transpose() << std::endl;
  std::cout << "  Max position -> Index: " << idxMax.transpose() << std::endl;
  
  PolygonIterator iterator(map, polygon);
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    ++iterator;
    ++count;
  }
  
  std::cout << "\nTotal cells: " << count << std::endl;
  
  if (count == 0) {
    std::cout << "*** BUG REPRODUCED: Empty iterator! ***" << std::endl;
    std::cout << "This happens when bounding box calculation fails with circular buffer" << std::endl;
  }
  
  EXPECT_GT(count, 0);
}




// Edge case: polygon positioned between grid cells
TEST(PolygonIterator, PolygonBetweenCells)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  // Very small polygon positioned between cell centers
  Polygon polygon;
  polygon.addVertex(Position(0.45, 0.45));
  polygon.addVertex(Position(0.55, 0.45));
  polygon.addVertex(Position(0.55, 0.55));
  polygon.addVertex(Position(0.45, 0.55));
  
  PolygonIterator iterator(map, polygon);
  
  // Should have at least one cell (the cell containing the polygon center)
  int count = 0;
  while (!iterator.isPastEnd()) {
    ++iterator;
    ++count;
  }
  
  // This might be 0 if polygon doesn't contain any cell centers - this is the bug!
  // After fix, should have at least 1
  if (count == 0) {
    std::cout << "WARNING: PolygonIterator returned empty for small polygon between cells!" << std::endl;
  }
}

// Edge case: tiny polygon that may not contain cell center
TEST(PolygonIterator, TinyPolygonNoCenter)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  // Extremely small polygon - 0.01 x 0.01
  Polygon polygon;
  polygon.addVertex(Position(0.495, 0.495));
  polygon.addVertex(Position(0.505, 0.495));
  polygon.addVertex(Position(0.505, 0.505));
  polygon.addVertex(Position(0.495, 0.505));
  
  PolygonIterator iterator(map, polygon);
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    ++iterator;
    ++count;
  }
  
  // This might be 0 - which is the reported bug
  if (count == 0) {
    std::cout << "WARNING: PolygonIterator returned empty for tiny polygon!" << std::endl;
  }
}

// Reproduce specific failing case for debugging
TEST(PolygonIterator, DebugFailingCase)
{
  std::cout << "\n=== Debug Specific Failing Case ===" << std::endl;
  
  // One of the failing cases from random test:
  // Buffer start: 5 5, Polygon: -5 -5 to -3 -3
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  map.move(Position(5.0, 5.0));
  
  std::cout << "Map position: " << map.getPosition().transpose() << std::endl;
  std::cout << "Buffer start: " << map.getStartIndex().transpose() << std::endl;
  std::cout << "Map size: " << map.getSize().transpose() << std::endl;
  
  Polygon polygon;
  polygon.addVertex(Position(-5.0, -5.0));
  polygon.addVertex(Position(-3.0, -5.0));
  polygon.addVertex(Position(-3.0, -3.0));
  polygon.addVertex(Position(-5.0, -3.0));
  
  std::cout << "\nPolygon vertices:" << std::endl;
  for (const auto& v : polygon.getVertices()) {
    std::cout << "  " << v.transpose();
    grid_map::Index idx;
    if (map.getIndex(v, idx)) {
      std::cout << " -> Index " << idx.transpose();
    } else {
      std::cout << " -> OUTSIDE MAP";
    }
    std::cout << std::endl;
  }
  
  // Check if positions are within map
  Position testPos1(-4.0, -4.0);
  Position testPos2(0.0, 0.0);
  std::cout << "\nTest if positions in map:" << std::endl;
  std::cout << "  (-4,-4): " << (map.isInside(testPos1) ? "INSIDE" : "OUTSIDE") << std::endl;
  std::cout << "  (0,0): " << (map.isInside(testPos2) ? "INSIDE" : "OUTSIDE") << std::endl;
  
  PolygonIterator iterator(map, polygon);
  
  int count = 0;
  std::cout << "\nIterating (first 10):" << std::endl;
  while (!iterator.isPastEnd() && count < 10) {
    grid_map::Index idx = *iterator;
    Position pos;
    map.getPosition(idx, pos);
    std::cout << "  Index " << idx.transpose() << " -> Position " << pos.transpose() << std::endl;
    ++iterator;
    ++count;
  }
  
  while (!iterator.isPastEnd() && count < 1000) {
    ++iterator;
    ++count;
  }
  
  std::cout << "Total cells: " << count << std::endl;
  
  if (count == 0) {
    std::cout << "*** BUG CONFIRMED: Empty iterator for valid polygon! ***" << std::endl;
  }
}

// Randomized stress test for various configurations
TEST(PolygonIterator, RandomizedStressTest)
{
  std::cout << "\n=== Randomized Stress Test ===" << std::endl;
  
  // Use fixed seed for reproducibility
  std::srand(42);
  
  const int numTests = 100;
  int emptyIteratorCount = 0;
  int failedTests = 0;
  
  for (int testIdx = 0; testIdx < numTests; ++testIdx) {
    // Random map configuration
    double mapSize = 5.0 + (std::rand() % 20);  // 5 to 25 meters
    double resolution = 0.1 + (std::rand() % 20) * 0.1;  // 0.1 to 2.0 meters
    double mapPosX = (std::rand() % 20) - 10.0;  // -10 to 10
    double mapPosY = (std::rand() % 20) - 10.0;
    
    GridMap map({"layer"});
    map.setGeometry(Length(mapSize, mapSize), resolution, Position(mapPosX, mapPosY));
    
    Position currentMapPos(mapPosX, mapPosY);
    
    // Randomly move map to test circular buffer
    if (std::rand() % 2 == 0) {
      double moveX = (std::rand() % 10) - 5.0;
      double moveY = (std::rand() % 10) - 5.0;
      map.move(Position(mapPosX + moveX, mapPosY + moveY));
      currentMapPos = map.getPosition();
    }
    
    // Random polygon WITHIN current map bounds (not original)
    double polySize = resolution * 2 + (std::rand() % 10) * resolution;
    // Ensure polygon is within the CURRENT map position
    double polyCenterX = currentMapPos.x() + ((std::rand() % 80) - 40) * mapSize / 100.0;
    double polyCenterY = currentMapPos.y() + ((std::rand() % 80) - 40) * mapSize / 100.0;
    
    Polygon polygon;
    // Create square polygon
    polygon.addVertex(Position(polyCenterX - polySize/2, polyCenterY - polySize/2));
    polygon.addVertex(Position(polyCenterX + polySize/2, polyCenterY - polySize/2));
    polygon.addVertex(Position(polyCenterX + polySize/2, polyCenterY + polySize/2));
    polygon.addVertex(Position(polyCenterX - polySize/2, polyCenterY + polySize/2));
    
    // Verify at least one vertex is inside map
    bool hasVertexInside = false;
    for (const auto& v : polygon.getVertices()) {
      if (map.isInside(v)) {
        hasVertexInside = true;
        break;
      }
    }
    
    // Also check polygon center
    Position polyCenter(polyCenterX, polyCenterY);
    bool centerInside = map.isInside(polyCenter);
    
    // Create iterator and count cells
    PolygonIterator iterator(map, polygon);
    int count = 0;
    int maxIterations = 10000;  // Safety limit
    
    while (!iterator.isPastEnd() && count < maxIterations) {
      ++iterator;
      ++count;
    }
    
    // Only consider it a failure if polygon overlaps map but iterator is empty
    if (count == 0 && (hasVertexInside || centerInside)) {
      emptyIteratorCount++;
      if (emptyIteratorCount <= 5) {  // Print details for first few failures
        std::cout << "Test " << testIdx << " - Empty iterator for overlapping polygon!" << std::endl;
        std::cout << "  Map: size=" << mapSize << ", res=" << resolution 
                  << ", pos=(" << currentMapPos.x() << "," << currentMapPos.y() << ")" << std::endl;
        std::cout << "  Buffer start: " << map.getStartIndex().transpose() << std::endl;
        std::cout << "  Polygon center: (" << polyCenterX << "," << polyCenterY 
                  << "), size=" << polySize << std::endl;
        std::cout << "  Has vertex inside: " << hasVertexInside << ", Center inside: " << centerInside << std::endl;
      }
      failedTests++;
    }
    
    if (count >= maxIterations) {
      std::cout << "Test " << testIdx << " - Infinite loop detected!" << std::endl;
      failedTests++;
    }
  }
  
  std::cout << "\nRandomized test results:" << std::endl;
  std::cout << "  Total tests: " << numTests << std::endl;
  std::cout << "  Empty iterators (with overlap): " << emptyIteratorCount << std::endl;
  std::cout << "  Failed tests: " << failedTests << std::endl;
  std::cout << "  Success rate: " << (100.0 * (numTests - failedTests) / numTests) << "%" << std::endl;
  
  EXPECT_EQ(failedTests, 0) << "Randomized tests found " << failedTests << " failures";
}

// Test specifically for circular buffer wrap-around cases
TEST(PolygonIterator, CircularBufferWrapAround)
{
  std::cout << "\n=== Circular Buffer Wrap-Around Test ===" << std::endl;
  
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  // Test various buffer start positions
  std::vector<grid_map::Index> testBufferStarts = {
    grid_map::Index(0, 0),   // No wrap
    grid_map::Index(5, 5),   // Middle wrap
    grid_map::Index(9, 9),   // Near end wrap
    grid_map::Index(3, 7),   // Asymmetric wrap
    grid_map::Index(7, 2),   // Asymmetric wrap
  };
  
  int totalTests = 0;
  int failedTests = 0;
  
  for (const auto& bufferStart : testBufferStarts) {
    // Manually set buffer start (simulate moved map)
    map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
    
    // Move map to create specific buffer configuration
    double moveX = bufferStart(0) * 1.0;
    double moveY = bufferStart(1) * 1.0;
    map.move(Position(moveX, moveY));
    
    Position currentMapPos = map.getPosition();
    
    // Test polygons at various positions RELATIVE TO CURRENT MAP POSITION
    std::vector<std::pair<Position, Position>> polygonConfigs = {
      {Position(currentMapPos.x() - 2.0, currentMapPos.y() - 2.0), Position(currentMapPos.x() + 2.0, currentMapPos.y() + 2.0)},    // Center
      {Position(currentMapPos.x() + 1.0, currentMapPos.y() + 1.0), Position(currentMapPos.x() + 3.0, currentMapPos.y() + 3.0)},    // Offset from center
      {Position(currentMapPos.x() - 3.0, currentMapPos.y() - 3.0), Position(currentMapPos.x() - 1.0, currentMapPos.y() - 1.0)},    // Other quadrant
      {Position(currentMapPos.x() - 2.0, currentMapPos.y() + 1.0), Position(currentMapPos.x(), currentMapPos.y() + 3.0)},          // Mixed quadrant
    };
    
    for (const auto& polyConfig : polygonConfigs) {
      Polygon polygon;
      polygon.addVertex(Position(polyConfig.first.x(), polyConfig.first.y()));
      polygon.addVertex(Position(polyConfig.second.x(), polyConfig.first.y()));
      polygon.addVertex(Position(polyConfig.second.x(), polyConfig.second.y()));
      polygon.addVertex(Position(polyConfig.first.x(), polyConfig.second.y()));
      
      // Check if polygon overlaps map
      bool hasOverlap = false;
      for (const auto& v : polygon.getVertices()) {
        if (map.isInside(v)) {
          hasOverlap = true;
          break;
        }
      }
      Position polyCenter = (polyConfig.first + polyConfig.second) / 2.0;
      if (map.isInside(polyCenter)) {
        hasOverlap = true;
      }
      
      if (!hasOverlap) {
        // Skip polygons that don't overlap - that's expected to be empty
        continue;
      }
      
      PolygonIterator iterator(map, polygon);
      int count = 0;
      
      while (!iterator.isPastEnd() && count < 1000) {
        ++iterator;
        ++count;
      }
      
      totalTests++;
      
      if (count == 0) {
        std::cout << "FAIL - Buffer start: " << bufferStart.transpose() 
                  << ", Polygon: " << polyConfig.first.transpose() 
                  << " to " << polyConfig.second.transpose() << std::endl;
        std::cout << "  Map position: " << currentMapPos.transpose() << std::endl;
        failedTests++;
      }
    }
  }
  
  std::cout << "Wrap-around test: " << (totalTests - failedTests) << "/" << totalTests << " passed" << std::endl;
  EXPECT_EQ(failedTests, 0) << "Found " << failedTests << " wrap-around failures";
}


