/*
 * PolygonIterator.hpp
 *
 *  Created on: Sep 19, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_core/iterators/PolygonIterator.hpp"

#include "grid_map_core/GridMapMath.hpp"

using namespace std;

namespace grid_map
{
PolygonIterator::PolygonIterator(const grid_map::GridMap& gridMap, const grid_map::Polygon& polygon) : polygon_(polygon)
{
  mapLength_ = gridMap.getLength();
  mapPosition_ = gridMap.getPosition();
  resolution_ = gridMap.getResolution();
  bufferSize_ = gridMap.getSize();
  bufferStartIndex_ = gridMap.getStartIndex();
  Index submapStartIndex;
  Size submapBufferSize;
  findSubmapParameters(polygon, submapStartIndex, submapBufferSize);
  internalIterator_ = std::make_shared<SubmapIterator>(gridMap, submapStartIndex, submapBufferSize);
  if (!isInside()) ++(*this);
}

PolygonIterator& PolygonIterator::operator=(const PolygonIterator& other)
{
  polygon_ = other.polygon_;
  internalIterator_ = other.internalIterator_;
  mapLength_ = other.mapLength_;
  mapPosition_ = other.mapPosition_;
  resolution_ = other.resolution_;
  bufferSize_ = other.bufferSize_;
  bufferStartIndex_ = other.bufferStartIndex_;
  return *this;
}

bool PolygonIterator::operator!=(const PolygonIterator& other) const
{
  return (internalIterator_ != other.internalIterator_);
}

const Index& PolygonIterator::operator*() const { return *(*internalIterator_); }

PolygonIterator& PolygonIterator::operator++()
{
  ++(*internalIterator_);
  if (internalIterator_->isPastEnd()) return *this;

  for (; !internalIterator_->isPastEnd(); ++(*internalIterator_))
  {
    if (isInside()) break;
  }

  return *this;
}

bool PolygonIterator::isPastEnd() const { return internalIterator_->isPastEnd(); }

bool PolygonIterator::isInside() const
{
  Position position;
  getPositionFromIndex(position, *(*internalIterator_), mapLength_, mapPosition_, resolution_, bufferSize_,
                       bufferStartIndex_);
  return polygon_.isInside(position);
}

void PolygonIterator::findSubmapParameters(const grid_map::Polygon& polygon, Index& startIndex, Size& bufferSize) const
{
  // Find bounding box in world/map coordinates
  // Note: "topLeft" means maximum x,y in world frame, "bottomRight" means minimum x,y
  Position topLeft = polygon_.getVertices()[0];
  Position bottomRight = topLeft;
  for (const auto& vertex : polygon_.getVertices())
  {
    topLeft = topLeft.array().max(vertex.array());        // Maximum world coordinates
    bottomRight = bottomRight.array().min(vertex.array()); // Minimum world coordinates
  }
  
  // Check if polygon is completely outside map bounds before clamping
  const double halfMapX = mapLength_(0) / 2.0;
  const double halfMapY = mapLength_(1) / 2.0;
  const Position mapMin = mapPosition_ - Position(halfMapX, halfMapY);
  const Position mapMax = mapPosition_ + Position(halfMapX, halfMapY);
  
  // If polygon bounding box doesn't intersect map, create empty iterator (zero size submap)
  if (bottomRight(0) > mapMax(0) || topLeft(0) < mapMin(0) ||
      bottomRight(1) > mapMax(1) || topLeft(1) < mapMin(1)) {
    // Polygon is completely outside - return zero size to create empty iterator
    startIndex = bufferStartIndex_;  // Use a valid index
    bufferSize = Size::Zero();       // Zero size = empty iterator
    return;
  }
  
  // Clamp positions to map range
  boundPositionToRange(topLeft, mapLength_, mapPosition_);
  boundPositionToRange(bottomRight, mapLength_, mapPosition_);
  
  // After clamping, verify positions are still valid and distinct
  const double minSeparation = resolution_ * 0.5;
  if ((topLeft - bottomRight).norm() < minSeparation) {
    // Positions collapsed to same point - polygon is likely outside or degenerate
    // Return zero size for empty iterator
    startIndex = bufferStartIndex_;
    bufferSize = Size::Zero();
    return;
  }
  
  // Convert world positions to grid indices
  // IMPORTANT: Due to the coordinate frame transformation (Index = -Position after offset/scale),
  // the relationship between world positions and grid indices is INVERTED:
  // - topLeft (max world coords) -> SMALLER index values
  // - bottomRight (min world coords) -> LARGER index values
  Index topLeftIndex, bottomRightIndex;
  getIndexFromPosition(topLeftIndex, topLeft, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  getIndexFromPosition(bottomRightIndex, bottomRight, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  
  // Calculate submap size
  // getSubmapSizeFromCornerIndeces expects topLeft (smaller index) first, bottomRight (larger index) second
  startIndex = topLeftIndex;
  bufferSize = getSubmapSizeFromCornerIndeces(topLeftIndex, bottomRightIndex, bufferSize_, bufferStartIndex_);
  
  // Validate that we got a positive size - protects against edge cases with circular buffer wrapping
  if ((bufferSize.array() <= 0).any()) {
    // This can happen in rare cases with circular buffer edge conditions
    // Return zero size for empty iterator rather than trying to fix it
    bufferSize = Size::Zero();
    startIndex = bufferStartIndex_;
  }
}

} /* namespace grid_map */
