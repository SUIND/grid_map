/*
 * GridMapMath.cpp
 *
 *  Created on: Dec 2, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_core/GridMapMath.hpp"

// fabs
#include <cmath>

// Limits
#include <limits>

using namespace std;

namespace grid_map
{
namespace internal
{
/*!
 * Gets the vector from the center of the map to the origin
 * of the map data structure.
 * @param[out] vectorToOrigin the vector from the center of the map the origin of the map data structure.
 * @param[in] mapLength the lengths in x and y direction.
 * @return true if successful.
 */
inline bool getVectorToOrigin(Vector& vectorToOrigin, const Length& mapLength)
{
  vectorToOrigin = (0.5 * mapLength).matrix();
  return true;
}

/*!
 * Gets the vector from the center of the map to the center
 * of the first cell of the map data.
 * @param[out] vectorToFirstCell the vector from the center of the cell to the center of the map.
 * @param[in] mapLength the lengths in x and y direction.
 * @param[in] resolution the resolution of the map.
 * @return true if successful.
 */
inline bool getVectorToFirstCell(Vector& vectorToFirstCell, const Length& mapLength, const double& resolution)
{
  Vector vectorToOrigin;
  getVectorToOrigin(vectorToOrigin, mapLength);

  // Vector to center of cell.
  vectorToFirstCell = (vectorToOrigin.array() - 0.5 * resolution).matrix();
  return true;
}

inline Eigen::Matrix2i getBufferOrderToMapFrameTransformation() { return -Eigen::Matrix2i::Identity(); }

inline Vector transformBufferOrderToMapFrame(const Index& index) { return {-index[0], -index[1]}; }

inline Eigen::Matrix2i getMapFrameToBufferOrderTransformation()
{
  return getBufferOrderToMapFrameTransformation().transpose();
}

inline Index transformMapFrameToBufferOrder(const Vector& vector) { return {-vector[0], -vector[1]}; }

inline Index transformMapFrameToBufferOrder(const Eigen::Vector2i& vector) { return {-vector[0], -vector[1]}; }

inline bool checkIfStartIndexAtDefaultPosition(const Index& bufferStartIndex)
{
  return ((bufferStartIndex == 0).all());
}

/*!
 * Checks if a position is within the map boundaries along one axis. This is the single axis form of
 * checkIfPositionWithinMap() and performs the same arithmetic, so both agree bit for bit.
 * @param[in] position the position along the axis.
 * @param[in] mapLength the length of the map along the axis.
 * @param[in] mapPosition the position of the map along the axis.
 * @return true if the position is within the map, false otherwise.
 */
inline bool checkIfPositionWithinMapAxis(const double position, const double mapLength, const double mapPosition)
{
  const double positionTransformed = -(position - mapPosition - 0.5 * mapLength);
  return positionTransformed >= 0.0 && positionTransformed < mapLength;
}

inline Vector getIndexVectorFromIndex(const Index& index, const Size& bufferSize, const Index& bufferStartIndex)
{
  Index unwrappedIndex;
  unwrappedIndex = getIndexFromBufferIndex(index, bufferSize, bufferStartIndex);
  return transformBufferOrderToMapFrame(unwrappedIndex);
}

inline Index getIndexFromIndexVector(const Vector& indexVector, const Size& bufferSize, const Index& bufferStartIndex)
{
  Index index = transformMapFrameToBufferOrder(indexVector);
  return getBufferIndexFromIndex(index, bufferSize, bufferStartIndex);
}

inline BufferRegion::Quadrant getQuadrant(const Index& index, const Index& bufferStartIndex)
{
  if (index[0] >= bufferStartIndex[0] && index[1] >= bufferStartIndex[1]) return BufferRegion::Quadrant::TopLeft;
  if (index[0] >= bufferStartIndex[0] && index[1] < bufferStartIndex[1]) return BufferRegion::Quadrant::TopRight;
  if (index[0] < bufferStartIndex[0] && index[1] >= bufferStartIndex[1]) return BufferRegion::Quadrant::BottomLeft;
  if (index[0] < bufferStartIndex[0] && index[1] < bufferStartIndex[1]) return BufferRegion::Quadrant::BottomRight;
  return BufferRegion::Quadrant::Undefined;
}

}  // namespace internal

using namespace internal;

bool getPositionFromIndex(Position& position, const Index& index, const Length& mapLength, const Position& mapPosition,
                          const double& resolution, const Size& bufferSize, const Index& bufferStartIndex)
{
  if (!checkIfIndexInRange(index, bufferSize)) return false;
  Vector offset;
  getVectorToFirstCell(offset, mapLength, resolution);
  position = mapPosition + offset + resolution * getIndexVectorFromIndex(index, bufferSize, bufferStartIndex);
  return true;
}

void getIndexFromPositionUnsafe(Index& index, const Position& position, const Length& mapLength,
                                const Position& mapPosition, const double& resolution, const Size& bufferSize,
                                const Index& bufferStartIndex)
{
  Vector offset;
  getVectorToOrigin(offset, mapLength);
  Vector indexVector = ((position - offset - mapPosition).array() / resolution).matrix();
  index = getIndexFromIndexVector(indexVector, bufferSize, bufferStartIndex);
}

bool getIndexFromPosition(Index& index, const Position& position, const Length& mapLength, const Position& mapPosition,
                          const double& resolution, const Size& bufferSize, const Index& bufferStartIndex)
{
  Vector offset;
  getVectorToOrigin(offset, mapLength);
  Vector indexVector = ((position - offset - mapPosition).array() / resolution).matrix();
  Index unwrappedIndex = transformMapFrameToBufferOrder(indexVector);
  const bool isWithinMap = checkIfPositionWithinMap(position, mapLength, mapPosition);

  if (isWithinMap)
  {
    // Rounding in the transformation above can place a position at a map edge one cell past the
    // boundary of the buffer. Bound the index here, because wrapping it into buffer order below
    // would turn it into a valid looking index on the opposite side of the circular buffer.
    for (int i = 0; i < unwrappedIndex.size(); i++)
    {
      if (unwrappedIndex(i) == bufferSize(i))
      {
        unwrappedIndex(i) = bufferSize(i) - 1;
      }
      else if (unwrappedIndex(i) == -1)
      {
        unwrappedIndex(i) = 0;
      }
    }
  }

  index = getBufferIndexFromIndex(unwrappedIndex, bufferSize, bufferStartIndex);
  // Check the unwrapped index: the wrapped one is in range whenever the buffer start index has
  // moved, which would mask an index that is actually outside the map.
  return isWithinMap && checkIfIndexInRange(unwrappedIndex, bufferSize);
}

bool checkIfPositionWithinMap(const Position& position, const Length& mapLength, const Position& mapPosition)
{
  Vector offset;
  getVectorToOrigin(offset, mapLength);
  Position positionTransformed =
      getMapFrameToBufferOrderTransformation().cast<double>() * (position - mapPosition - offset);

  if (positionTransformed.x() >= 0.0 && positionTransformed.y() >= 0.0 && positionTransformed.x() < mapLength(0) &&
      positionTransformed.y() < mapLength(1))
  {
    return true;
  }
  return false;
}

void getPositionOfDataStructureOrigin(const Position& position, const Length& mapLength, Position& positionOfOrigin)
{
  Vector vectorToOrigin;
  getVectorToOrigin(vectorToOrigin, mapLength);
  positionOfOrigin = position + vectorToOrigin;
}

bool getIndexShiftFromPositionShift(Index& indexShift, const Vector& positionShift, const double& resolution)
{
  Vector indexShiftVectorTemp = (positionShift.array() / resolution).matrix();
  Eigen::Vector2i indexShiftVector;

  for (int i = 0; i < indexShiftVector.size(); i++)
  {
    indexShiftVector[i] = static_cast<int>(indexShiftVectorTemp[i] + 0.5 * (indexShiftVectorTemp[i] > 0 ? 1 : -1));
  }

  indexShift = transformMapFrameToBufferOrder(indexShiftVector);
  return true;
}

bool getPositionShiftFromIndexShift(Vector& positionShift, const Index& indexShift, const double& resolution)
{
  positionShift = transformBufferOrderToMapFrame(indexShift) * resolution;
  return true;
}

bool checkIfIndexInRange(const Index& index, const Size& bufferSize)
{
  if (index[0] >= 0 && index[1] >= 0 && index[0] < bufferSize[0] && index[1] < bufferSize[1])
  {
    return true;
  }
  return false;
}

void boundIndexToRange(Index& index, const Size& bufferSize)
{
  for (int i = 0; i < index.size(); i++)
  {
    boundIndexToRange(index[i], bufferSize[i]);
  }
}

void boundIndexToRange(int& index, const int& bufferSize)
{
  if (index < 0)
    index = 0;
  else if (index >= bufferSize)
    index = bufferSize - 1;
}

void wrapIndexToRange(Index& index, const Size& bufferSize)
{
  for (int i = 0; i < index.size(); i++)
  {
    wrapIndexToRange(index[i], bufferSize[i]);
  }
}

void wrapIndexToRange(int& index, int bufferSize)
{
  // Try shortcuts before resorting to the expensive modulo operation.
  if (index < bufferSize)
  {
    if (index >= 0)
    {  // within the wanted range
      return;
    }
    else if (index >= -bufferSize)
    {  // Index is below range, but not more than one span of the range.
      index += bufferSize;
      return;
    }
    else
    {  // Index is largely below range.
      index = index % bufferSize;
      index += bufferSize;
    }
  }
  else if (index < bufferSize * 2)
  {  // Index is above range, but not more than one span of the range.
    index -= bufferSize;
    return;
  }
  else
  {  // Index is largely above range.
    index = index % bufferSize;
  }
}

void boundPositionToRange(Position& position, const Length& mapLength, const Position& mapPosition)
{
  Vector vectorToOrigin;
  getVectorToOrigin(vectorToOrigin, mapLength);

  for (int i = 0; i < position.size(); i++)
  {
    if (checkIfPositionWithinMapAxis(position(i), mapLength(i), mapPosition(i))) continue;

    // The map covers the half open interval (mapPosition - mapLength / 2, mapPosition + mapLength / 2],
    // so bound the position to the edge it lies beyond.
    position(i) =
        position(i) > mapPosition(i) ? mapPosition(i) + vectorToOrigin(i) : mapPosition(i) - vectorToOrigin(i);

    // Bounding to an edge is not enough on its own: the lower edge is not part of the map, and
    // rounding in the expression above can leave the position marginally outside either edge. A
    // position outside the map converts to an index outside the buffer, so step towards the center
    // of the map until the position is inside. This takes a few units in the last place at most.
    while (!checkIfPositionWithinMapAxis(position(i), mapLength(i), mapPosition(i)))
    {
      const double steppedPosition = std::nextafter(position(i), mapPosition(i));
      if (steppedPosition == position(i)) break;  // Degenerate map, there is no position inside it.
      position(i) = steppedPosition;
    }
  }
}

const Eigen::Matrix2i getBufferOrderToMapFrameAlignment()
{
  return getBufferOrderToMapFrameTransformation().array().abs().matrix();
}

bool getSubmapInformation(Index& submapTopLeftIndex, Size& submapBufferSize, Position& submapPosition,
                          Length& submapLength, Index& requestedIndexInSubmap, const Position& requestedSubmapPosition,
                          const Length& requestedSubmapLength, const Length& mapLength, const Position& mapPosition,
                          const double& resolution, const Size& bufferSize, const Index& bufferStartIndex)
{
  // (Top left / bottom right corresponds to the position in the matrix, not the map frame)
  const Eigen::Matrix2d halfTransform = 0.5 * getMapFrameToBufferOrderTransformation().cast<double>();

  // Corners of submap.
  Position topLeftPosition = requestedSubmapPosition - halfTransform * requestedSubmapLength.matrix();
  boundPositionToRange(topLeftPosition, mapLength, mapPosition);
  if (!getIndexFromPosition(submapTopLeftIndex, topLeftPosition, mapLength, mapPosition, resolution, bufferSize,
                            bufferStartIndex))
    return false;
  Index topLeftIndex;
  topLeftIndex = getIndexFromBufferIndex(submapTopLeftIndex, bufferSize, bufferStartIndex);

  Position bottomRightPosition = requestedSubmapPosition + halfTransform * requestedSubmapLength.matrix();
  boundPositionToRange(bottomRightPosition, mapLength, mapPosition);
  Index bottomRightIndex;
  if (!getIndexFromPosition(bottomRightIndex, bottomRightPosition, mapLength, mapPosition, resolution, bufferSize,
                            bufferStartIndex))
    return false;
  bottomRightIndex = getIndexFromBufferIndex(bottomRightIndex, bufferSize, bufferStartIndex);

  // Get the position of the top left corner of the generated submap.
  Position topLeftCorner;
  if (!getPositionFromIndex(topLeftCorner, submapTopLeftIndex, mapLength, mapPosition, resolution, bufferSize,
                            bufferStartIndex))
    return false;
  topLeftCorner -= halfTransform * Position::Constant(resolution);

  // Size of submap.
  submapBufferSize = bottomRightIndex - topLeftIndex + Index::Ones();

  // Length of the submap.
  submapLength = submapBufferSize.cast<double>() * resolution;

  // Position of submap.
  Vector vectorToSubmapOrigin;
  getVectorToOrigin(vectorToSubmapOrigin, submapLength);
  submapPosition = topLeftCorner - vectorToSubmapOrigin;

  // Get the index of the cell which corresponds the requested
  // position of the submap.
  return getIndexFromPosition(requestedIndexInSubmap, requestedSubmapPosition, submapLength, submapPosition, resolution,
                              submapBufferSize);
}

Size getSubmapSizeFromCornerIndeces(const Index& topLeftIndex, const Index& bottomRightIndex, const Size& bufferSize,
                                    const Index& bufferStartIndex)
{
  const Index unwrappedTopLeftIndex = getIndexFromBufferIndex(topLeftIndex, bufferSize, bufferStartIndex);
  const Index unwrappedBottomRightIndex = getIndexFromBufferIndex(bottomRightIndex, bufferSize, bufferStartIndex);
  return Size(unwrappedBottomRightIndex - unwrappedTopLeftIndex + Size::Ones());
}

bool getBufferRegionsForSubmap(std::vector<BufferRegion>& submapBufferRegions, const Index& submapIndex,
                               const Size& submapBufferSize, const Size& bufferSize, const Index& bufferStartIndex)
{
  if ((getIndexFromBufferIndex(submapIndex, bufferSize, bufferStartIndex) + submapBufferSize > bufferSize).any())
    return false;

  submapBufferRegions.clear();

  Index bottomRightIndex = submapIndex + submapBufferSize - Index::Ones();
  wrapIndexToRange(bottomRightIndex, bufferSize);

  BufferRegion::Quadrant quadrantOfTopLeft = getQuadrant(submapIndex, bufferStartIndex);
  BufferRegion::Quadrant quadrantOfBottomRight = getQuadrant(bottomRightIndex, bufferStartIndex);

  if (quadrantOfTopLeft == BufferRegion::Quadrant::TopLeft)
  {
    if (quadrantOfBottomRight == BufferRegion::Quadrant::TopLeft)
    {
      submapBufferRegions.push_back(BufferRegion(submapIndex, submapBufferSize, BufferRegion::Quadrant::TopLeft));
      return true;
    }

    if (quadrantOfBottomRight == BufferRegion::Quadrant::TopRight)
    {
      Size topLeftSize(submapBufferSize(0), bufferSize(1) - submapIndex(1));
      submapBufferRegions.push_back(BufferRegion(submapIndex, topLeftSize, BufferRegion::Quadrant::TopLeft));

      Index topRightIndex(submapIndex(0), 0);
      Size topRightSize(submapBufferSize(0), submapBufferSize(1) - topLeftSize(1));
      submapBufferRegions.push_back(BufferRegion(topRightIndex, topRightSize, BufferRegion::Quadrant::TopRight));
      return true;
    }

    if (quadrantOfBottomRight == BufferRegion::Quadrant::BottomLeft)
    {
      Size topLeftSize(bufferSize(0) - submapIndex(0), submapBufferSize(1));
      submapBufferRegions.push_back(BufferRegion(submapIndex, topLeftSize, BufferRegion::Quadrant::TopLeft));

      Index bottomLeftIndex(0, submapIndex(1));
      Size bottomLeftSize(submapBufferSize(0) - topLeftSize(0), submapBufferSize(1));
      submapBufferRegions.push_back(BufferRegion(bottomLeftIndex, bottomLeftSize, BufferRegion::Quadrant::BottomLeft));
      return true;
    }

    if (quadrantOfBottomRight == BufferRegion::Quadrant::BottomRight)
    {
      Size topLeftSize(bufferSize(0) - submapIndex(0), bufferSize(1) - submapIndex(1));
      submapBufferRegions.push_back(BufferRegion(submapIndex, topLeftSize, BufferRegion::Quadrant::TopLeft));

      Index topRightIndex(submapIndex(0), 0);
      Size topRightSize(bufferSize(0) - submapIndex(0), submapBufferSize(1) - topLeftSize(1));
      submapBufferRegions.push_back(BufferRegion(topRightIndex, topRightSize, BufferRegion::Quadrant::TopRight));

      Index bottomLeftIndex(0, submapIndex(1));
      Size bottomLeftSize(submapBufferSize(0) - topLeftSize(0), bufferSize(1) - submapIndex(1));
      submapBufferRegions.push_back(BufferRegion(bottomLeftIndex, bottomLeftSize, BufferRegion::Quadrant::BottomLeft));

      Index bottomRightIndex = Index::Zero();
      Size bottomRightSize(bottomLeftSize(0), topRightSize(1));
      submapBufferRegions.push_back(
          BufferRegion(bottomRightIndex, bottomRightSize, BufferRegion::Quadrant::BottomRight));
      return true;
    }
  }
  else if (quadrantOfTopLeft == BufferRegion::Quadrant::TopRight)
  {
    if (quadrantOfBottomRight == BufferRegion::Quadrant::TopRight)
    {
      submapBufferRegions.push_back(BufferRegion(submapIndex, submapBufferSize, BufferRegion::Quadrant::TopRight));
      return true;
    }

    if (quadrantOfBottomRight == BufferRegion::Quadrant::BottomRight)
    {
      Size topRightSize(bufferSize(0) - submapIndex(0), submapBufferSize(1));
      submapBufferRegions.push_back(BufferRegion(submapIndex, topRightSize, BufferRegion::Quadrant::TopRight));

      Index bottomRightIndex(0, submapIndex(1));
      Size bottomRightSize(submapBufferSize(0) - topRightSize(0), submapBufferSize(1));
      submapBufferRegions.push_back(
          BufferRegion(bottomRightIndex, bottomRightSize, BufferRegion::Quadrant::BottomRight));
      return true;
    }
  }
  else if (quadrantOfTopLeft == BufferRegion::Quadrant::BottomLeft)
  {
    if (quadrantOfBottomRight == BufferRegion::Quadrant::BottomLeft)
    {
      submapBufferRegions.push_back(BufferRegion(submapIndex, submapBufferSize, BufferRegion::Quadrant::BottomLeft));
      return true;
    }

    if (quadrantOfBottomRight == BufferRegion::Quadrant::BottomRight)
    {
      Size bottomLeftSize(submapBufferSize(0), bufferSize(1) - submapIndex(1));
      submapBufferRegions.push_back(BufferRegion(submapIndex, bottomLeftSize, BufferRegion::Quadrant::BottomLeft));

      Index bottomRightIndex(submapIndex(0), 0);
      Size bottomRightSize(submapBufferSize(0), submapBufferSize(1) - bottomLeftSize(1));
      submapBufferRegions.push_back(
          BufferRegion(bottomRightIndex, bottomRightSize, BufferRegion::Quadrant::BottomRight));
      return true;
    }
  }
  else if (quadrantOfTopLeft == BufferRegion::Quadrant::BottomRight)
  {
    if (quadrantOfBottomRight == BufferRegion::Quadrant::BottomRight)
    {
      submapBufferRegions.push_back(BufferRegion(submapIndex, submapBufferSize, BufferRegion::Quadrant::BottomRight));
      return true;
    }
  }

  return false;
}

bool incrementIndex(Index& index, const Size& bufferSize, const Index& bufferStartIndex)
{
  Index unwrappedIndex = getIndexFromBufferIndex(index, bufferSize, bufferStartIndex);

  // Increment index.
  if (unwrappedIndex(1) + 1 < bufferSize(1))
  {
    // Same row.
    unwrappedIndex[1]++;
  }
  else
  {
    // Next row.
    unwrappedIndex[0]++;
    unwrappedIndex[1] = 0;
  }

  // End of iterations reached.
  if (!checkIfIndexInRange(unwrappedIndex, bufferSize)) return false;

  // Return true iterated index.
  index = getBufferIndexFromIndex(unwrappedIndex, bufferSize, bufferStartIndex);
  return true;
}

bool incrementIndexForSubmap(Index& submapIndex, Index& index, const Index& submapTopLeftIndex,
                             const Size& submapBufferSize, const Size& bufferSize, const Index& bufferStartIndex)
{
  // Copy the data first, only copy it back if everything is within range.
  Index tempIndex = index;
  Index tempSubmapIndex = submapIndex;

  // Increment submap index.
  if (tempSubmapIndex[1] + 1 < submapBufferSize[1])
  {
    // Same row.
    tempSubmapIndex[1]++;
  }
  else
  {
    // Next row.
    tempSubmapIndex[0]++;
    tempSubmapIndex[1] = 0;
  }

  // End of iterations reached.
  if (!checkIfIndexInRange(tempSubmapIndex, submapBufferSize)) return false;

  // Get corresponding index in map.
  Index unwrappedSubmapTopLeftIndex = getIndexFromBufferIndex(submapTopLeftIndex, bufferSize, bufferStartIndex);
  tempIndex = getBufferIndexFromIndex(unwrappedSubmapTopLeftIndex + tempSubmapIndex, bufferSize, bufferStartIndex);

  // Copy data back.
  index = tempIndex;
  submapIndex = tempSubmapIndex;
  return true;
}

Index getIndexFromBufferIndex(const Index& bufferIndex, const Size& bufferSize, const Index& bufferStartIndex)
{
  if (checkIfStartIndexAtDefaultPosition(bufferStartIndex)) return bufferIndex;

  Index index = bufferIndex - bufferStartIndex;
  wrapIndexToRange(index, bufferSize);
  return index;
}

Index getBufferIndexFromIndex(const Index& index, const Size& bufferSize, const Index& bufferStartIndex)
{
  if (checkIfStartIndexAtDefaultPosition(bufferStartIndex)) return index;

  Index bufferIndex = index + bufferStartIndex;
  wrapIndexToRange(bufferIndex, bufferSize);
  return bufferIndex;
}

size_t getLinearIndexFromIndex(const Index& index, const Size& bufferSize, const bool rowMajor)
{
  if (!rowMajor) return index(1) * bufferSize(0) + index(0);
  return index(0) * bufferSize(1) + index(1);
}

Index getIndexFromLinearIndex(const size_t linearIndex, const Size& bufferSize, const bool rowMajor)
{
  if (!rowMajor) return Index((int)linearIndex % bufferSize(0), (int)linearIndex / bufferSize(0));
  return Index((int)linearIndex / bufferSize(1), (int)linearIndex % bufferSize(1));
}

void getIndicesForRegion(const Index& regionIndex, const Size& regionSize, std::vector<Index> indices)
{
  //  for (int i = line.index_; col < line.endIndex(); col++) {
  //    for (int i = 0; i < getSize()(0); i++) {
  //
  //    }
  //  }
}

void getIndicesForRegions(const std::vector<Index>& regionIndeces, const Size& regionSizes, std::vector<Index> indices)
{
}

bool colorValueToVector(const unsigned long& colorValue, Eigen::Vector3i& colorVector)
{
  colorVector(0) = (colorValue >> 16) & 0x0000ff;
  colorVector(1) = (colorValue >> 8) & 0x0000ff;
  colorVector(2) = colorValue & 0x0000ff;
  return true;
}

bool colorValueToVector(const unsigned long& colorValue, Eigen::Vector3f& colorVector)
{
  Eigen::Vector3i tempColorVector;
  colorValueToVector(colorValue, tempColorVector);
  colorVector = ((tempColorVector.cast<float>()).array() / 255.0).matrix();
  return true;
}

bool colorValueToVector(const float& colorValue, Eigen::Vector3f& colorVector)
{
  // cppcheck-suppress invalidPointerCast
  const unsigned long tempColorValue = *reinterpret_cast<const unsigned long*>(&colorValue);
  colorValueToVector(tempColorValue, colorVector);
  return true;
}

bool colorVectorToValue(const Eigen::Vector3i& colorVector, unsigned long& colorValue)
{
  colorValue = ((int)colorVector(0)) << 16 | ((int)colorVector(1)) << 8 | ((int)colorVector(2));
  return true;
}

void colorVectorToValue(const Eigen::Vector3i& colorVector, float& colorValue)
{
  unsigned long color = (colorVector(0) << 16) + (colorVector(1) << 8) + colorVector(2);
  // cppcheck-suppress invalidPointerCast
  colorValue = *reinterpret_cast<float*>(&color);
}

void colorVectorToValue(const Eigen::Vector3f& colorVector, float& colorValue)
{
  Eigen::Vector3i tempColorVector = (colorVector * 255.0).cast<int>();
  colorVectorToValue(tempColorVector, colorValue);
}

}  // namespace grid_map
