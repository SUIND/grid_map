/*
 * SubmapIteratorTest.cpp
 *
 *  Created on: Sep 15, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/GridMapMath.hpp"
#include "grid_map_core/iterators/SubmapIterator.hpp"

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

TEST(SubmapIterator, Simple)
{
  Eigen::Array2i submapTopLeftIndex(3, 1);
  Eigen::Array2i submapBufferSize(3, 2);
  Eigen::Array2i index;
  Eigen::Array2i submapIndex;

  vector<string> types;
  types.push_back("type");
  GridMap map(types);
  map.setGeometry(Array2d(8.1, 5.1), 1.0, Vector2d(0.0, 0.0));  // bufferSize(8, 5)

  SubmapIterator iterator(map, submapTopLeftIndex, submapBufferSize);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(submapTopLeftIndex(0), (*iterator)(0));
  EXPECT_EQ(submapTopLeftIndex(1), (*iterator)(1));
  EXPECT_EQ(0, iterator.getSubmapIndex()(0));
  EXPECT_EQ(0, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));
  EXPECT_EQ(0, iterator.getSubmapIndex()(0));
  EXPECT_EQ(1, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(0, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(1, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));
  EXPECT_EQ(2, iterator.getSubmapIndex()(0));
  EXPECT_EQ(0, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));
  EXPECT_EQ(2, iterator.getSubmapIndex()(0));
  EXPECT_EQ(1, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));
  EXPECT_EQ(2, iterator.getSubmapIndex()(0));
  EXPECT_EQ(1, iterator.getSubmapIndex()(1));
}

TEST(SubmapIterator, CircularBuffer)
{
  Eigen::Array2i submapTopLeftIndex(6, 3);
  Eigen::Array2i submapBufferSize(2, 4);
  Eigen::Array2i index;
  Eigen::Array2i submapIndex;

  vector<string> types;
  types.push_back("type");
  GridMap map(types);
  map.setGeometry(Length(8.1, 5.1), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)
  map.move(Position(-3.0, -2.0));                              // bufferStartIndex(3, 2)

  SubmapIterator iterator(map, submapTopLeftIndex, submapBufferSize);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(submapTopLeftIndex(0), (*iterator)(0));
  EXPECT_EQ(submapTopLeftIndex(1), (*iterator)(1));
  EXPECT_EQ(0, iterator.getSubmapIndex()(0));
  EXPECT_EQ(0, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));
  EXPECT_EQ(0, iterator.getSubmapIndex()(0));
  EXPECT_EQ(1, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));
  EXPECT_EQ(0, iterator.getSubmapIndex()(0));
  EXPECT_EQ(2, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));
  EXPECT_EQ(0, iterator.getSubmapIndex()(0));
  EXPECT_EQ(3, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(0, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(1, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(2, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(3, iterator.getSubmapIndex()(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));
  EXPECT_EQ(1, iterator.getSubmapIndex()(0));
  EXPECT_EQ(3, iterator.getSubmapIndex()(1));
}

/**
 * The submap should contain the same elements as before even after moving the underlying map.
 *
 *                                                          +----------------------------+
 *                                                          |                            |
 *                                                          |                            |
 *              +----------------------------+              |                            |
 *              |0  0  0  0  0  0  0  0  0  0|              |      0  0  0  0  0  0  0  0|
 *              |     +----+                 |              |           +----+           |
 * Submap       |1  1 |1  1| 1  1  1  1  1  1|              |      1  1 |1  1| 1  1  1  1|
 *           +------> |    |                 |              |           |    |           |
 *              |2  2 |2  2| 2  2  2  2  2  2|              |      2  2 |2  2| 2  2  2  2|
 *              |     +----+                 |              |           +----+           |
 *              |3  3  3  3  3  3  3  3  3  3|   Move       |      3  3  3  3  3  3  3  3|
 *              |                            |              |                            |
 *              |4  4  4  4  4  4  4  4  4  4| +--------->  |      4  4  4  4  4  4  4  4|
 *              |                            |              |                            |
 *              |5  5  5  5  5  5  5  5  5  5|              |      5  5  5  5  5  5  5  5|
 *              |                            |              |                            |
 *              |6  6  6  6  6  6  6  6  6  6|              |      6  6  6  6  6  6  6  6|
 *              |                            |              |                            |
 *              |7  7  7  7  7  7  7  7  7  7|              |      7  7  7  7  7  7  7  7|
 *              |                            |              +----------------------------+
 *              |8  8  8  8  8  8  8  8  8  8|
 *              |                            |
 *              |9  9  9  9  9  9  9  9  9  9|
 *              +----------------------------+
 */
TEST(SubmapIterator, InterleavedExecutionWithMove)
{
  grid_map::Index submapTopLeftIndex(3, 1);
  grid_map::Size submapSize(2, 2);

  GridMap map({"layer"});

  map.setGeometry(Length(10, 10), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  auto& layer = map.get("layer");

  // Initialize the layer as sketched.
  for (size_t colIndex = 0; colIndex < layer.cols(); colIndex++)
  {
    layer.col(colIndex).setConstant(colIndex);
  }

  std::cout << "(4,7) contains " << map.at("layer", {4, 7}) << std::endl;
  // Instantiate the submap iterator as sketched.
  SubmapIterator iterator(map, submapTopLeftIndex, submapSize);

  // check that the submap iterator returns {1,1,2,2}
  auto checkCorrectValues = [](std::array<double, 4> given) {
    int countOnes = 0, countTwos = 0;
    for (auto& value : given)
    {
      if (std::abs(value - 1.0) < 1e-6)
      {
        countOnes++;
      }
      else if (std::abs(value - 2.0) < 1e-6)
      {
        countTwos++;
      }
      else
      {
        FAIL() << "Submap iterator returned unexpected value.";
      }
    }
    EXPECT_EQ(countOnes, 2);
    EXPECT_EQ(countTwos, 2);
  };

  std::array<double, 4> returnedSequence;
  returnedSequence.fill(0);

  for (size_t submapIndex = 0; submapIndex < 4; submapIndex++)
  {
    returnedSequence.at(submapIndex) = map.at("layer", *iterator);
    ++iterator;
  }

  checkCorrectValues(returnedSequence);

  // Reset the iterator and now check that it still returns the same sequence when we move the map interleaved with
  // iterating.
  iterator = SubmapIterator(map, submapTopLeftIndex, submapSize);
  returnedSequence.fill(0);
  for (size_t submapIndex = 0; submapIndex < 4; submapIndex++)
  {
    if (submapIndex == 2)
    {
      // Now move the map as depicted.
      map.move(Position(2.0, 2.0));
    }
    returnedSequence.at(submapIndex) = map.at("layer", *iterator);
    ++iterator;
  }
  checkCorrectValues(returnedSequence);

  // TODO (mwulf, mgaertner): This behavior is not yet implemented:
  //
  //  // Reset the iterator and now check that the iterator throws? if the submap moved out of range.
  //  iterator = SubmapIterator(map, submapTopLeftIndex, submapSize);
  //
  //  EXPECT_ANY_THROW(for (size_t submapIndex = 0; submapIndex < 4; submapIndex++) {
  //    if (submapIndex == 2) {
  //      // Now move the map so that the submap gets out of range.
  //      map.move(Position(20.0, 20.0));
  //    }
  //    returnedSequence.at(submapIndex) = map.at("layer", *iterator);
  //    ++iterator;
  //  });
}

/*
 * Regression test: a submap spanning the corners of a polygon that overhangs a map edge.
 *
 * This is the same failure as PolygonIterator.MovedMapWithPolygonOverhangingMapEdge, observed one
 * layer down: bounding the overhanging corner into the map and converting it to an index yields an
 * index that wraps to the opposite edge of the ring buffer, so getSubmapSizeFromCornerIndeces()
 * returns a negative size and the iterator is degenerate.
 */
TEST(SubmapIterator, BoxSpanningCornersOverhangingMapEdgeOnMovedMap)
{
  GridMap map({"layer"});
  map.setGeometry(Length(33.746855726960831, 9.9433593878904709), 0.25,
                  Position(10.795406602280387, -22.994307697034245));
  map.move(Position(24.953846346189515, -38.957489169334572));
  ASSERT_FALSE((map.getStartIndex() == 0).all()) << "this test requires a moved (circular buffer) map";

  // Corners of a box overhanging the lower x and y edges of the map, bounded back into the map.
  Position topLeft(13.526077842487485, -38.044727225921832);
  Position bottomRight(2.3031839924109754, -47.214226670855084);
  boundPositionToRange(topLeft, map.getLength(), map.getPosition());
  boundPositionToRange(bottomRight, map.getLength(), map.getPosition());

  grid_map::Index topLeftIndex;
  grid_map::Index bottomRightIndex;
  EXPECT_TRUE(getIndexFromPosition(topLeftIndex, topLeft, map.getLength(), map.getPosition(), map.getResolution(),
                                   map.getSize(), map.getStartIndex()));
  EXPECT_TRUE(getIndexFromPosition(bottomRightIndex, bottomRight, map.getLength(), map.getPosition(),
                                   map.getResolution(), map.getSize(), map.getStartIndex()));

  const Size submapSize =
      getSubmapSizeFromCornerIndeces(topLeftIndex, bottomRightIndex, map.getSize(), map.getStartIndex());
  ASSERT_TRUE((submapSize > 0).all()) << "submap size is (" << submapSize(0) << ", " << submapSize(1)
                                      << "), which makes the iterator degenerate";

  size_t visited = 0;
  for (SubmapIterator iterator(map, topLeftIndex, submapSize); !iterator.isPastEnd(); ++iterator)
  {
    EXPECT_TRUE(checkIfIndexInRange(*iterator, map.getSize()));
    ++visited;
  }
  EXPECT_EQ(static_cast<size_t>(submapSize.prod()), visited);
}

/*
 * A submap with a non-positive size has nothing to iterate over. Before this was handled the
 * iterator started out not-past-end and returned one cell that is not part of any submap.
 */
TEST(SubmapIterator, NonPositiveSubmapSizeIsPastEndImmediately)
{
  GridMap map({"layer"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  EXPECT_TRUE(SubmapIterator(map, grid_map::Index(2, 1), Size(0, 0)).isPastEnd());
  EXPECT_TRUE(SubmapIterator(map, grid_map::Index(2, 1), Size(3, 0)).isPastEnd());
  EXPECT_TRUE(SubmapIterator(map, grid_map::Index(2, 1), Size(0, 3)).isPastEnd());
  EXPECT_TRUE(SubmapIterator(map, grid_map::Index(2, 1), Size(-4, 3)).isPastEnd());
  EXPECT_FALSE(SubmapIterator(map, grid_map::Index(2, 1), Size(1, 1)).isPastEnd());
}
