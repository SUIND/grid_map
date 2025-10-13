/*
 * LineIteratorTest.cpp
 *
 *  Created on: Sep 14, 2016
 *      Author: Dominic Jud
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/LineIterator.hpp"

// gtest
#include <gtest/gtest.h>

// Limits
#include <cfloat>

// Vector
#include <vector>

using namespace grid_map;

TEST(LineIterator, StartOutsideMap)
{
  GridMap map({"types"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));

  EXPECT_NO_THROW(LineIterator iterator(map, Position(2.0, 2.0), Position(0.0, 0.0)));
  LineIterator iterator(map, Position(2.0, 2.0), Position(0.0, 0.0));

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(2, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(LineIterator, EndOutsideMap)
{
  GridMap map({"types"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));

  EXPECT_NO_THROW(LineIterator iterator(map, Position(0.0, 0.0), Position(9.0, 6.0)));
  LineIterator iterator(map, Position(0.0, 0.0), Position(9.0, 6.0));

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(2, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(LineIterator, StartAndEndOutsideMap)
{
  GridMap map({"types"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));

  EXPECT_NO_THROW(LineIterator iterator(map, Position(-7.0, -9.0), Position(8.0, 8.0)));
  LineIterator iterator(map, Position(-7.0, -9.0), Position(8.0, 8.0));

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  ++iterator;
  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(LineIterator, StartAndEndOutsideMapWithoutIntersectingMap)
{
  GridMap map({"types"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));

  EXPECT_THROW(LineIterator iterator(map, Position(-8.0, 8.0), Position(8.0, 8.0)), std::invalid_argument);
}

TEST(LineIterator, MovedMap)
{
  GridMap map({"types"});
  map.setGeometry(Length(7.0, 5.0), 1.0, Position(0.0, 0.0));
  map.move(Position(2.0, 2.0));

  EXPECT_NO_THROW(LineIterator iterator(map, Position(0.0, 0.0), Position(2.0, 2.0)));
  LineIterator iterator(map, Position(0.0, 0.0), Position(2.0, 2.0));
  Position point;

  EXPECT_FALSE(iterator.isPastEnd());
  map.getPosition(*iterator, point);
  EXPECT_EQ(0, point.x());
  EXPECT_EQ(0, point.y());

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  map.getPosition(*iterator, point);
  EXPECT_EQ(1, point.x());
  EXPECT_EQ(1, point.y());

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  map.getPosition(*iterator, point);
  EXPECT_EQ(2, point.x());
  EXPECT_EQ(2, point.y());

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(LineIterator, StartAndEndOutsideMovedMap)
{
  GridMap map({"types"});
  map.setGeometry(Length(7.0, 5.0), 1.0, Position(0.0, 0.0));
  map.move(Position(2.0, 2.0));

  EXPECT_NO_THROW(LineIterator iterator(map, Position(0.0, 0.0), Position(8.0, 8.0)));
  LineIterator iterator(map, Position(0.0, 0.0), Position(8.0, 8.0));
  Position point;

  EXPECT_FALSE(iterator.isPastEnd());
  map.getPosition(*iterator, point);
  EXPECT_EQ(0, point.x());
  EXPECT_EQ(0, point.y());

  ++iterator;
  map.getPosition(*iterator, point);
  EXPECT_EQ(1, point.x());
  EXPECT_EQ(1, point.y());
  //
  ++iterator;
  map.getPosition(*iterator, point);
  EXPECT_EQ(2, point.x());
  EXPECT_EQ(2, point.y());
  //
  ++iterator;
  map.getPosition(*iterator, point);
  EXPECT_EQ(3, point.x());
  EXPECT_EQ(3, point.y());

  ++iterator;
  map.getPosition(*iterator, point);
  EXPECT_EQ(4, point.x());
  EXPECT_EQ(4, point.y());

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

// New comprehensive tests

TEST(LineIterator, HorizontalLine)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  // Horizontal line from left to right
  LineIterator iterator(map, Position(-3.0, 0.0), Position(3.0, 0.0));
  
  int count = 0;
  Position lastPos;
  while (!iterator.isPastEnd()) {
    Position pos;
    map.getPosition(*iterator, pos);
    EXPECT_NEAR(0.0, pos.y(), 0.6); // Should stay on same row
    if (count > 0) {
      EXPECT_GT(pos.x(), lastPos.x()); // Should move right
    }
    lastPos = pos;
    ++iterator;
    ++count;
  }
  EXPECT_GT(count, 0); // Should have at least one point
}

TEST(LineIterator, VerticalLine)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  // Vertical line from bottom to top
  LineIterator iterator(map, Position(0.0, -3.0), Position(0.0, 3.0));
  
  int count = 0;
  Position lastPos;
  while (!iterator.isPastEnd()) {
    Position pos;
    map.getPosition(*iterator, pos);
    EXPECT_NEAR(0.0, pos.x(), 0.6); // Should stay on same column
    if (count > 0) {
      EXPECT_GT(pos.y(), lastPos.y()); // Should move up
    }
    lastPos = pos;
    ++iterator;
    ++count;
  }
  EXPECT_GT(count, 0);
}

TEST(LineIterator, DiagonalLine)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  // Perfect 45-degree diagonal
  LineIterator iterator(map, Position(-2.0, -2.0), Position(2.0, 2.0));
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    Position pos;
    map.getPosition(*iterator, pos);
    // On a perfect diagonal, x and y should be approximately equal
    EXPECT_NEAR(pos.x(), pos.y(), 1.5);
    ++iterator;
    ++count;
  }
  EXPECT_GT(count, 3);
}

TEST(LineIterator, SinglePointLine)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  // Line with same start and end point
  LineIterator iterator(map, Position(1.0, 1.0), Position(1.0, 1.0));
  
  EXPECT_FALSE(iterator.isPastEnd());
  Position pos;
  map.getPosition(*iterator, pos);
  EXPECT_NEAR(1.0, pos.x(), 0.6);
  EXPECT_NEAR(1.0, pos.y(), 0.6);
  
  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(LineIterator, ShortLine)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 0.5, Position(0.0, 0.0));
  
  // Very short line (less than one cell)
  LineIterator iterator(map, Position(0.0, 0.0), Position(0.3, 0.3));
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    ++iterator;
    ++count;
  }
  EXPECT_GE(count, 1); // Should have at least one cell
  EXPECT_LE(count, 3); // Should not have many cells for a short line
}

TEST(LineIterator, LongDiagonalLine)
{
  GridMap map({"layer"});
  map.setGeometry(Length(20.0, 20.0), 1.0, Position(0.0, 0.0));
  
  // Long diagonal line across the map
  LineIterator iterator(map, Position(-9.0, -9.0), Position(9.0, 9.0));
  
  int count = 0;
  std::vector<Index> indices;
  while (!iterator.isPastEnd()) {
    indices.push_back(*iterator);
    ++iterator;
    ++count;
  }
  
  EXPECT_GT(count, 15); // Long line should have many points
  
  // Check no duplicate indices
  for (size_t i = 0; i < indices.size(); ++i) {
    for (size_t j = i + 1; j < indices.size(); ++j) {
      EXPECT_TRUE((indices[i] != indices[j]).any());
    }
  }
}

TEST(LineIterator, ReverseDirection)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  // Create line in both directions
  LineIterator forwardIter(map, Position(-2.0, -2.0), Position(2.0, 2.0));
  LineIterator reverseIter(map, Position(2.0, 2.0), Position(-2.0, -2.0));
  
  std::vector<Index> forwardIndices;
  std::vector<Index> reverseIndices;
  
  while (!forwardIter.isPastEnd()) {
    forwardIndices.push_back(*forwardIter);
    ++forwardIter;
  }
  
  while (!reverseIter.isPastEnd()) {
    reverseIndices.push_back(*reverseIter);
    ++reverseIter;
  }
  
  EXPECT_EQ(forwardIndices.size(), reverseIndices.size());
  
  // Check that reverse is actually reversed
  for (size_t i = 0; i < forwardIndices.size(); ++i) {
    size_t reverseIdx = reverseIndices.size() - 1 - i;
    EXPECT_EQ(forwardIndices[i](0), reverseIndices[reverseIdx](0));
    EXPECT_EQ(forwardIndices[i](1), reverseIndices[reverseIdx](1));
  }
}

TEST(LineIterator, NegativeToPositive)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  LineIterator iterator(map, Position(-4.0, -3.0), Position(3.0, 4.0));
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    // Indices returned by iterator should be valid within the map
    Position pos;
    map.getPosition(*iterator, pos);
    EXPECT_TRUE(map.isInside(pos));
    ++iterator;
    ++count;
  }
  EXPECT_GT(count, 5);
}

TEST(LineIterator, IndexConstructor)
{
  GridMap map({"layer"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));
  
  // Use index-based constructor with valid indices
  grid_map::Index start(2, 1);
  grid_map::Index end(5, 3);
  
  LineIterator iterator(map, start, end);
  
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(start(0), (*iterator)(0));
  EXPECT_EQ(start(1), (*iterator)(1));
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    Position pos;
    map.getPosition(*iterator, pos);
    EXPECT_TRUE(map.isInside(pos));
    ++iterator;
    ++count;
  }
  EXPECT_GT(count, 2);
}

TEST(LineIterator, CopyAndAssignment)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  LineIterator original(map, Position(-2.0, -2.0), Position(2.0, 2.0));
  
  // Test copy constructor (assignment operator)
  LineIterator copy = original;
  EXPECT_EQ((*original)(0), (*copy)(0));
  EXPECT_EQ((*original)(1), (*copy)(1));
  
  // Advance both to verify they are synchronized after copy
  grid_map::Index originalFirstIndex = *original;
  grid_map::Index copyFirstIndex = *copy;
  
  // They should start at the same position
  EXPECT_EQ(originalFirstIndex(0), copyFirstIndex(0));
  EXPECT_EQ(originalFirstIndex(1), copyFirstIndex(1));
  
  // Advance original
  ++original;
  
  // Copy should still be at the original position (not advanced)
  EXPECT_EQ((*copy)(0), copyFirstIndex(0));
  EXPECT_EQ((*copy)(1), copyFirstIndex(1));
}

TEST(LineIterator, InequalityOperator)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  LineIterator iter1(map, Position(-2.0, -2.0), Position(2.0, 2.0));
  LineIterator iter2(map, Position(-2.0, -2.0), Position(2.0, 2.0));
  
  EXPECT_FALSE(iter1 != iter2);
  
  ++iter2;
  EXPECT_TRUE(iter1 != iter2);
}

TEST(LineIterator, HighResolutionMap)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 0.1, Position(0.0, 0.0));
  
  LineIterator iterator(map, Position(-2.0, -1.0), Position(2.0, 1.0));
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    Position pos;
    map.getPosition(*iterator, pos);
    EXPECT_TRUE(map.isInside(pos));
    ++iterator;
    ++count;
  }
  
  // High resolution should give more points
  EXPECT_GT(count, 30);
}

TEST(LineIterator, LowResolutionMap)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 2.0, Position(0.0, 0.0));
  
  LineIterator iterator(map, Position(-4.0, -4.0), Position(4.0, 4.0));
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    Position pos;
    map.getPosition(*iterator, pos);
    EXPECT_TRUE(map.isInside(pos));
    ++iterator;
    ++count;
  }
  
  // Low resolution should give fewer points
  EXPECT_LE(count, 10);
  EXPECT_GT(count, 0);
}

TEST(LineIterator, NonSquareMap)
{
  GridMap map({"layer"});
  map.setGeometry(Length(20.0, 5.0), 1.0, Position(0.0, 0.0));
  
  LineIterator iterator(map, Position(-8.0, 0.0), Position(8.0, 0.0));
  
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

TEST(LineIterator, PartiallyOutsideStarting)
{
  GridMap map({"layer"});
  map.setGeometry(Length(6.0, 6.0), 1.0, Position(0.0, 0.0));
  
  // Start outside, end inside
  LineIterator iterator(map, Position(-5.0, 0.0), Position(1.0, 0.0));
  
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

TEST(LineIterator, PartiallyOutsideEnding)
{
  GridMap map({"layer"});
  map.setGeometry(Length(6.0, 6.0), 1.0, Position(0.0, 0.0));
  
  // Start inside, end outside
  LineIterator iterator(map, Position(-1.0, 0.0), Position(5.0, 0.0));
  
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

TEST(LineIterator, AllIndicesValid)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  map.add("layer", 0.0);
  
  LineIterator iterator(map, Position(-3.0, -3.0), Position(3.0, 3.0));
  
  while (!iterator.isPastEnd()) {
    grid_map::Index index = *iterator;
    Position pos;
    map.getPosition(index, pos);
    // All positions returned should be inside the map
    EXPECT_TRUE(map.isInside(pos));
    // Should be able to access the layer
    EXPECT_NO_THROW(map.at("layer", index));
    ++iterator;
  }
}

TEST(LineIterator, SteepLine)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  // Very steep line (mostly vertical)
  LineIterator iterator(map, Position(0.0, -4.0), Position(0.5, 4.0));
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    ++iterator;
    ++count;
  }
  EXPECT_GT(count, 5);
}

TEST(LineIterator, ShallowLine)
{
  GridMap map({"layer"});
  map.setGeometry(Length(10.0, 10.0), 1.0, Position(0.0, 0.0));
  
  // Very shallow line (mostly horizontal)
  LineIterator iterator(map, Position(-4.0, 0.0), Position(4.0, 0.5));
  
  int count = 0;
  while (!iterator.isPastEnd()) {
    ++iterator;
    ++count;
  }
  EXPECT_GT(count, 5);
}

TEST(LineIterator, EmptyIteratorCheck)
{
  GridMap map({"layer"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));
  
  LineIterator iterator(map, Position(0.0, 0.0), Position(2.0, 2.0));
  
  // Should not be empty
  EXPECT_FALSE(iterator.isPastEnd());
  
  // Exhaust iterator
  while (!iterator.isPastEnd()) {
    ++iterator;
  }
  
  // Now should be past end
  EXPECT_TRUE(iterator.isPastEnd());
}

