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
  if (!findSubmapParameters(polygon, submapStartIndex, submapBufferSize))
  {
    // The polygon does not map to a region of the grid map, so there is nothing to iterate over.
    submapStartIndex.setZero();
    submapBufferSize.setZero();
  }
  internalIterator_ = std::make_shared<SubmapIterator>(gridMap, submapStartIndex, submapBufferSize);
  if (!isPastEnd() && !isInside()) ++(*this);
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
  if (!getPositionFromIndex(position, *(*internalIterator_), mapLength_, mapPosition_, resolution_, bufferSize_,
                            bufferStartIndex_))
  {
    return false;
  }
  return polygon_.isInside(position);
}

bool PolygonIterator::findSubmapParameters(const grid_map::Polygon& polygon, Index& startIndex, Size& bufferSize) const
{
  Position topLeft = polygon_.getVertices()[0];
  Position bottomRight = topLeft;
  for (const auto& vertex : polygon_.getVertices())
  {
    topLeft = topLeft.array().max(vertex.array());
    bottomRight = bottomRight.array().min(vertex.array());
  }
  boundPositionToRange(topLeft, mapLength_, mapPosition_);
  boundPositionToRange(bottomRight, mapLength_, mapPosition_);
  if (!getIndexFromPosition(startIndex, topLeft, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_))
  {
    return false;
  }
  Index endIndex;
  if (!getIndexFromPosition(endIndex, bottomRight, mapLength_, mapPosition_, resolution_, bufferSize_,
                            bufferStartIndex_))
  {
    return false;
  }
  bufferSize = getSubmapSizeFromCornerIndeces(startIndex, endIndex, bufferSize_, bufferStartIndex_);
  return (bufferSize > 0).all();
}

} /* namespace grid_map */
