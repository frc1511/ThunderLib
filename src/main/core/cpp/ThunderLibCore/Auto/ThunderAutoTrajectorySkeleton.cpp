#include <ThunderLibCore/Auto/ThunderAutoTrajectorySkeleton.hpp>

#include <ThunderLibCore/Auto/ThunderAutoOutputTrajectory.hpp>
#include <ThunderLibCore/Math.hpp>
#include <gcem.hpp>

namespace thunder::core {

ThunderAutoTrajectorySkeletonWaypoint::HeadingControlPoints::HeadingControlPoints(
    const Point2d& centerPoint,
    const HeadingAngles& angles,
    const HeadingWeights& weights) {
  setIncomingPoint(centerPoint, angles.incomingAngle(), weights.incomingWeight());
  setOutgoingPoint(centerPoint, angles.outgoingAngle(), weights.outgoingWeight());
}

void ThunderAutoTrajectorySkeletonWaypoint::HeadingControlPoints::setIncomingPoint(
    const Point2d& centerPoint,
    const CanonicalAngle& incomingAngle,
    const double incomingWeight) {
  m_incomingPoint = centerPoint.extendAtAngle(incomingAngle, units::meter_t(incomingWeight));
}

void ThunderAutoTrajectorySkeletonWaypoint::HeadingControlPoints::setOutgoingPoint(
    const Point2d& centerPoint,
    const CanonicalAngle& outgoingAngle,
    const double outgoingWeight) {
  m_outgoingPoint = centerPoint.extendAtAngle(outgoingAngle, units::meter_t(outgoingWeight));
}

ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles::HeadingAngles(
    const Point2d& centerPoint,
    const HeadingControlPoints& controlPoints) {
  setIncomingAngle(centerPoint, controlPoints.incomingPoint());
  setOutgoingAngle(centerPoint, controlPoints.outgoingPoint());
}

void ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles::setIncomingAngle(
    const CanonicalAngle& angle,
    bool lockSupplementary /*= false*/) {
  m_incomingAngle = angle;
  if (lockSupplementary) {
    m_outgoingAngle = angle.supplementary();
  }
}

void ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles::setIncomingAngle(
    const Point2d& centerPoint,
    const Point2d& incomingControlPoint,
    bool lockSupplementary /*= false*/) {
  Displacement2d d = incomingControlPoint - centerPoint;
  setIncomingAngle(d.angle(), lockSupplementary);
}

void ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles::setOutgoingAngle(
    const CanonicalAngle& angle,
    bool lockSupplementary /*= false*/) {
  m_outgoingAngle = angle;
  if (lockSupplementary) {
    m_incomingAngle = angle.supplementary();
  }
}

void ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles::setOutgoingAngle(
    const Point2d& centerPoint,
    const Point2d& outgoingControlPoint,
    bool lockSupplementary /*= false*/) {
  Displacement2d d = outgoingControlPoint - centerPoint;
  setOutgoingAngle(d.angle(), lockSupplementary);
}

void ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles::setAngle(bool isOutgoing,
                                                                    const CanonicalAngle& angle,
                                                                    bool lockSupplementary /*= false*/) {
  if (isOutgoing) {
    setOutgoingAngle(angle, lockSupplementary);
  } else {
    setIncomingAngle(angle, lockSupplementary);
  }
}

void ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles::setAngle(bool isOutgoing,
                                                                    const Point2d& centerPoint,
                                                                    const Point2d& controlPoint,
                                                                    bool lockSupplementary /*= false*/) {
  if (isOutgoing) {
    setOutgoingAngle(centerPoint, controlPoint, lockSupplementary);
  } else {
    setIncomingAngle(centerPoint, controlPoint, lockSupplementary);
  }
}

ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights::HeadingWeights(double incoming, double outgoing) {
  setIncomingWeight(incoming);
  setOutgoingWeight(outgoing);
}

ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights::HeadingWeights(
    const Point2d& centerPoint,
    const HeadingControlPoints& controlPoints) {
  setIncomingWeight(centerPoint, controlPoints.incomingPoint());
  setOutgoingWeight(centerPoint, controlPoints.outgoingPoint());
}

void ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights::setIncomingWeight(double weight) {
  m_incomingWeight = std::max(weight, MIN_WEIGHT);
}

void ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights::setIncomingWeight(
    const Point2d& centerPoint,
    const Point2d& incomingControlPoint) {
  Displacement2d d = incomingControlPoint - centerPoint;
  setIncomingWeight(d.distance().value());
}

void ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights::setOutgoingWeight(double weight) {
  m_outgoingWeight = std::max(weight, MIN_WEIGHT);
}

void ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights::setOutgoingWeight(
    const Point2d& centerPoint,
    const Point2d& outgoingControlPoint) {
  Displacement2d d = outgoingControlPoint - centerPoint;
  setOutgoingWeight(d.distance().value());
}

void ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights::setWeight(bool isOutgoing, double weight) {
  if (isOutgoing) {
    setOutgoingWeight(weight);
  } else {
    setIncomingWeight(weight);
  }
}

void ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights::setWeight(bool isOutgoing,
                                                                      const Point2d& centerPoint,
                                                                      const Point2d& controlPoint) {
  if (isOutgoing) {
    setOutgoingWeight(centerPoint, controlPoint);
  } else {
    setIncomingWeight(centerPoint, controlPoint);
  }
}

ThunderAutoTrajectorySkeletonWaypoint::ThunderAutoTrajectorySkeletonWaypoint(Point2d position,
                                                                             HeadingAngles headings,
                                                                             HeadingWeights headingWeights)
    : m_position(position), m_headingWeights(headingWeights) {
  setHeadings(headings);
}

ThunderAutoTrajectorySkeletonWaypoint::ThunderAutoTrajectorySkeletonWaypoint(
    Point2d position,
    units::meters_per_second_t velocityOverride,
    HeadingAngles headings,
    HeadingWeights headingWeights)
    : m_position(position), m_headingWeights(headingWeights) {
  setMaxVelocityOverride(velocityOverride);
  setHeadings(headings);
}

ThunderAutoTrajectorySkeletonWaypoint::ThunderAutoTrajectorySkeletonWaypoint(Point2d position,
                                                                             CanonicalAngle outgoingHeading,
                                                                             HeadingWeights headingWeights)
    : ThunderAutoTrajectorySkeletonWaypoint(position,
                                            {outgoingHeading.supplementary(), outgoingHeading},
                                            headingWeights) {}

ThunderAutoTrajectorySkeletonWaypoint::ThunderAutoTrajectorySkeletonWaypoint(
    Point2d position,
    HeadingControlPoints controlPoints)
    : ThunderAutoTrajectorySkeletonWaypoint(position, HeadingAngles{}, HeadingWeights{}) {
  setHeadingControlPoints(controlPoints);
}

void ThunderAutoTrajectorySkeletonWaypoint::setHeadings(const HeadingAngles& headings) {
  if (!isStopped() && !headings.incomingAngle().isSupplementaryTo(headings.outgoingAngle())) {
    throw InvalidArgumentError::Construct(
        "When not stopped, incoming heading must be supplementary to outgoing heading: incoming = {}, "
        "outgoing = {}",
        headings.incomingAngle().degrees().value(), headings.outgoingAngle().degrees().value());
  }

  m_headings = headings;
}

void ThunderAutoTrajectorySkeletonWaypoint::setIncomingHeading(const CanonicalAngle& angle) {
  m_headings.setIncomingAngle(angle, isStopped() == false);
}

void ThunderAutoTrajectorySkeletonWaypoint::setOutgoingHeading(const CanonicalAngle& angle) {
  m_headings.setOutgoingAngle(angle, isStopped() == false);
}

void ThunderAutoTrajectorySkeletonWaypoint::setHeading(bool isOutgoing, const CanonicalAngle& angle) {
  m_headings.setAngle(isOutgoing, angle, isStopped() == false);
}

void ThunderAutoTrajectorySkeletonWaypoint::setHeadingWeights(const HeadingWeights& weights) {
  m_headingWeights = weights;
}

void ThunderAutoTrajectorySkeletonWaypoint::setIncomingHeadingWeight(double weight) {
  m_headingWeights.setIncomingWeight(weight);
}

void ThunderAutoTrajectorySkeletonWaypoint::setOutgoingHeadingWeight(double weight) {
  m_headingWeights.setOutgoingWeight(weight);
}

void ThunderAutoTrajectorySkeletonWaypoint::setHeadingWeight(bool isOutgoing, double weight) {
  m_headingWeights.setWeight(isOutgoing, weight);
}

ThunderAutoTrajectorySkeletonWaypoint::HeadingControlPoints
ThunderAutoTrajectorySkeletonWaypoint::headingControlPoints() const {
  return HeadingControlPoints(m_position, m_headings, m_headingWeights);
}

void ThunderAutoTrajectorySkeletonWaypoint::setHeadingControlPoints(
    const HeadingControlPoints& controlPoints) {
  setIncomingHeadingControlPoint(controlPoints.incomingPoint());
  setOutgoingHeadingControlPoint(controlPoints.outgoingPoint());
}

void ThunderAutoTrajectorySkeletonWaypoint::setIncomingHeadingControlPoint(const Point2d& point) {
  m_headings.setIncomingAngle(m_position, point, isStopped() == false);
  m_headingWeights.setIncomingWeight(m_position, point);
}

void ThunderAutoTrajectorySkeletonWaypoint::setOutgoingHeadingControlPoint(const Point2d& point) {
  m_headings.setOutgoingAngle(m_position, point, isStopped() == false);
  m_headingWeights.setOutgoingWeight(m_position, point);
}

bool ThunderAutoTrajectorySkeletonWaypoint::isStopped() const noexcept {
  return m_maxVelocityOverride.has_value() && *m_maxVelocityOverride == 0_mps;
}

void ThunderAutoTrajectorySkeletonWaypoint::setStopped(bool stopped) noexcept {
  if (stopped) {
    m_maxVelocityOverride = 0_mps;
  } else {
    m_maxVelocityOverride.reset();
    m_stopRotation = CanonicalAngle{0_deg};
  }
}

CanonicalAngle ThunderAutoTrajectorySkeletonWaypoint::stopRotation() const {
  if (!isStopped()) {
    throw LogicError::Construct("Cannot get stop rotation when not stopped");
  }
  return m_stopRotation;
}

void ThunderAutoTrajectorySkeletonWaypoint::setStopRotation(const CanonicalAngle& rotation) {
  if (!isStopped()) {
    throw LogicError::Construct("Cannot set stop rotation when not stopped");
  }
  m_stopRotation = rotation;
}

void ThunderAutoTrajectorySkeletonWaypoint::setMaxVelocityOverride(
    units::meters_per_second_t velocity) noexcept {
  velocity = std::clamp(velocity, 0_mps, 7_mps);

  const bool wasStopped = isStopped();
  m_maxVelocityOverride = velocity;

  if (wasStopped && velocity > 0_mps) {
    m_stopAction.clear();
    // Lock incoming heading to outgoing because not stopped anymore.
    setOutgoingHeading(m_headings.outgoingAngle());
  }
}

void ThunderAutoTrajectorySkeletonWaypoint::resetMaxVelocityOverride() noexcept {
  m_maxVelocityOverride = std::nullopt;
  m_stopAction.clear();
  setOutgoingHeading(m_headings.outgoingAngle());  // Fix headings
}

void ThunderAutoTrajectorySkeletonWaypoint::setLinkName(const std::string& link) {
  if (link.empty()) {
    throw InvalidArgumentError::Construct("Link name cannot be empty");
  }
  m_link = link;
}

void ThunderAutoTrajectorySkeletonWaypoint::removeLink() noexcept {
  m_link.clear();
}

ThunderAutoTrajectorySkeleton::const_iterator ThunderAutoTrajectorySkeleton::insertPoint(
    const_iterator it,
    const ThunderAutoTrajectorySkeletonWaypoint& point) {
  ThunderAutoTrajectoryPosition shiftStartPosition = static_cast<double>(std::distance(cbegin(), it));
  shiftRotationAndActionPositions(shiftStartPosition, 1.0);

  return m_points.insert(it, point);
}

ThunderAutoTrajectorySkeleton::const_iterator ThunderAutoTrajectorySkeleton::insertPoint(
    size_t index,
    const ThunderAutoTrajectorySkeletonWaypoint& point) {
  if (index > numPoints()) {
    throw OutOfRangeError::Construct("Index out of bounds");
  }

  ThunderAutoTrajectoryPosition shiftStartPosition = static_cast<double>(index);
  shiftRotationAndActionPositions(shiftStartPosition, 1.0);

  iterator it = std::next(begin(), index);
  return m_points.insert(it, point);
}

void ThunderAutoTrajectorySkeleton::prependPoint(const ThunderAutoTrajectorySkeletonWaypoint& point) {
  m_points.push_front(point);
  shiftRotationAndActionPositions(0.0, 1.0);
}

void ThunderAutoTrajectorySkeleton::appendPoint(const ThunderAutoTrajectorySkeletonWaypoint& point) {
  m_points.push_back(point);
}

ThunderAutoTrajectorySkeleton::const_iterator ThunderAutoTrajectorySkeleton::removePoint(const_iterator it) {
  if (numPoints() <= 2) {
    throw LogicError::Construct("Cannot remove point: trajectory must have at least two points");
  }
  size_t index = std::distance(cbegin(), it);
  const bool wasFirst = (index == 0);
  const bool wasLast = (index == numPoints() - 1);

  const_iterator nextIt = m_points.erase(it);

  ThunderAutoTrajectoryPosition shiftStartPosition = static_cast<double>(index);
  shiftRotationAndActionPositions(shiftStartPosition, -1.0);

  purgeOutOfBoundsRotationsAndActions();

  /**
   * Make the new start/end point not have a velocity override to prevent accidents. The user must manually
   * set the velocity override of the start/end point.
   */
  if (wasFirst) {
    m_points.front().resetMaxVelocityOverride();
  } else if (wasLast) {
    m_points.back().resetMaxVelocityOverride();
  }

  return nextIt;
}

ThunderAutoTrajectorySkeleton::const_iterator ThunderAutoTrajectorySkeleton::removePoint(size_t index) {
  if (index >= numPoints()) {
    throw OutOfRangeError::Construct("Index out of bounds");
  }
  if (numPoints() <= 2) {
    throw LogicError::Construct("Cannot remove point: trajectory must have at least two points");
  }
  const bool wasFirst = (index == 0);
  const bool wasLast = (index == numPoints() - 1);

  iterator it = std::next(begin(), index);
  const_iterator nextIt = m_points.erase(it);

  ThunderAutoTrajectoryPosition shiftStartPosition = static_cast<double>(index);
  shiftRotationAndActionPositions(shiftStartPosition, -1.0);

  purgeOutOfBoundsRotationsAndActions();

  /**
   * Make the new start/end point not have a velocity override to prevent accidents. The user must manually
   * set the velocity override of the start/end point.
   */
  if (wasFirst) {
    m_points.front().resetMaxVelocityOverride();
  } else if (wasLast) {
    m_points.back().resetMaxVelocityOverride();
  }

  return nextIt;
}

void ThunderAutoTrajectorySkeleton::clearPoints() {
  m_points.clear();
}

ThunderAutoTrajectorySkeletonWaypoint& ThunderAutoTrajectorySkeleton::getPoint(size_t index) {
  if (index >= numPoints()) {
    throw OutOfRangeError::Construct("Index out of bounds");
  }
  iterator it = std::next(begin(), index);
  return *it;
}

const ThunderAutoTrajectorySkeletonWaypoint& ThunderAutoTrajectorySkeleton::getPoint(size_t index) const {
  if (index >= numPoints()) {
    throw OutOfRangeError::Construct("Index out of bounds");
  }
  const_iterator it = std::next(cbegin(), index);
  return *it;
}

ThunderAutoTrajectoryBehavior ThunderAutoTrajectorySkeleton::getBehavior() const {
  if (numPoints() < 2) {
    throw LogicError::Construct("Cannot get behavior: trajectory has less than two points");
  }

  const ThunderAutoTrajectorySkeletonWaypoint& first = front();
  const ThunderAutoTrajectorySkeletonWaypoint& last = back();

  frc::Pose2d startPose{first.position().x, first.position().y, m_startRotation};
  frc::Pose2d endPose{last.position().x, last.position().y, m_endRotation};

  return ThunderAutoTrajectoryBehavior{.startPose = startPose, .endPose = endPose};
}

ThunderAutoTrajectoryAction& ThunderAutoTrajectorySkeleton::getAction(size_t index) {
  if (index >= m_actions.size()) {
    throw OutOfRangeError::Construct("Index out of bounds");
  }
  auto it = std::next(m_actions.begin(), index);
  return it->second;
}

const ThunderAutoTrajectoryAction& ThunderAutoTrajectorySkeleton::getAction(size_t index) const {
  if (index >= m_actions.size()) {
    throw OutOfRangeError::Construct("Index out of bounds");
  }
  auto it = std::next(m_actions.cbegin(), index);
  return it->second;
}

ThunderAutoTrajectoryRotation& ThunderAutoTrajectorySkeleton::getRotation(size_t index) {
  if (index >= m_rotations.size()) {
    throw OutOfRangeError::Construct("Index out of bounds");
  }
  auto it = std::next(m_rotations.begin(), index);
  return it->second;
}

const ThunderAutoTrajectoryRotation& ThunderAutoTrajectorySkeleton::getRotation(size_t index) const {
  if (index >= m_rotations.size()) {
    throw OutOfRangeError::Construct("Index out of bounds");
  }
  auto it = std::next(m_rotations.cbegin(), index);
  return it->second;
}

void ThunderAutoTrajectorySkeleton::reverseDirection() {
  struct RotationAtCoordinate {
    units::meter_t newDistance;
    CanonicalAngle rotation;
  };

  struct ActionAtCoordinate {
    units::meter_t newDistance;
    std::string action;
  };

  std::vector<RotationAtCoordinate> rotations;
  std::vector<ActionAtCoordinate> actions;

  // Get coordinates of trajectory rotations and actions

  std::unique_ptr<ThunderAutoPartialOutputTrajectory> builtTrajectory =
      BuildThunderAutoPartialOutputTrajectory(*this, kPreviewOutputTrajectorySettings);
  ThunderLibCoreAssert(builtTrajectory != nullptr);
  ThunderLibCoreAssert(!builtTrajectory->points.empty());

  const units::meter_t originalTrajectoryDistance = builtTrajectory->totalDistance;

  for (const auto& [position, rotation] : m_rotations) {
    size_t pointIndex = builtTrajectory->trajectoryPositionToPointIndex(position);
    ThunderLibCoreAssert(pointIndex < builtTrajectory->points.size());
    units::meter_t distance = builtTrajectory->points[pointIndex].distance;
    rotations.emplace_back(originalTrajectoryDistance - distance, rotation.angle);
  }

  for (const auto& [position, action] : m_actions) {
    size_t pointIndex = builtTrajectory->trajectoryPositionToPointIndex(position);
    ThunderLibCoreAssert(pointIndex < builtTrajectory->points.size());
    units::meter_t distance = builtTrajectory->points[pointIndex].distance;
    actions.emplace_back(originalTrajectoryDistance - distance, action.action);
  }

  m_rotations.clear();
  m_actions.clear();

  // Reverse order of waypoints

  std::reverse(m_points.begin(), m_points.end());

  // Swap incoming/outgoing headings and heading weights

  for (ThunderAutoTrajectorySkeletonWaypoint& waypoint : m_points) {
    ThunderAutoTrajectorySkeletonWaypoint::HeadingControlPoints headingControlPoints =
        waypoint.headingControlPoints();
    ThunderAutoTrajectorySkeletonWaypoint::HeadingControlPoints newHeadingControlPoints(
        headingControlPoints.outgoingPoint(), headingControlPoints.incomingPoint());
    waypoint.setHeadingControlPoints(newHeadingControlPoints);
  }

  builtTrajectory = BuildThunderAutoPartialOutputTrajectory(*this, kPreviewOutputTrajectorySettings);

  const units::meter_t newTrajectoryDistance = builtTrajectory->totalDistance;

  if (!DoubleEquals(newTrajectoryDistance.value(), originalTrajectoryDistance.value(), 1e-3)) {
    throw LogicError::Construct(
        "Trajectory of different length after reversing direction: original length: {}, new length: {}",
        originalTrajectoryDistance.value(), newTrajectoryDistance.value());
  }

  auto findClosestPoint = [&](units::meter_t distance) {
    auto closestPointIt =
        std::lower_bound(builtTrajectory->points.begin(), builtTrajectory->points.end(), distance,
                         [](const ThunderAutoPartialOutputTrajectoryPoint& point,
                            const units::meter_t& distance) { return point.distance < distance; });
    return closestPointIt;
  };

  for (const auto& [distance, angle] : rotations) {
    auto closestPointIt = findClosestPoint(distance);
    ThunderLibCoreAssert(closestPointIt != builtTrajectory->points.end());

    size_t closestPointIndex = closestPointIt - builtTrajectory->points.begin();
    ThunderAutoTrajectoryPosition rotationPosition =
        builtTrajectory->pointIndexToTrajectoryPosition(closestPointIndex);

    ThunderAutoTrajectoryRotation newRotation{.angle = angle};

    m_rotations.add(rotationPosition, newRotation);
  }

  for (const auto& [distance, action] : actions) {
    auto closestPointIt = findClosestPoint(distance);
    ThunderLibCoreAssert(closestPointIt != builtTrajectory->points.end());

    size_t closestPointIndex = closestPointIt - builtTrajectory->points.begin();
    ThunderAutoTrajectoryPosition actionPosition =
        builtTrajectory->pointIndexToTrajectoryPosition(closestPointIndex);

    ThunderAutoTrajectoryAction newAction{.action = action};

    m_actions.add(actionPosition, newAction);
  }

  std::swap(m_startAction, m_endAction);
  std::swap(m_startRotation, m_endRotation);
}

void ThunderAutoTrajectorySkeleton::separateRotations(
    units::meter_t minSeparationDistance,
    const ThunderAutoPartialOutputTrajectory* trajectoryPositionData) {
  if (!trajectoryPositionData) {
    throw InvalidArgumentError::Construct("Trajectory position data is null");
  }

  // A trajectory segment in between two fixed rotation targets (start/end rotations, or stop rotations).
  struct RotationSegment {
    ThunderAutoTrajectoryPosition startPosition;
    ThunderAutoTrajectoryPosition endPosition;
    std::vector<ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryRotation>::iterator> rotationIts;
  };

  std::vector<RotationSegment> segments(m_points.size() - 1);
  size_t head = 0;
  segments[head].startPosition = 0.0;

  if (m_points.size() > 2) {
    auto pointIt = std::next(m_points.cbegin());
    for (size_t i = 1; i < m_points.size() - 1; i++, pointIt++) {
      if (pointIt->isStopped()) {
        ThunderAutoTrajectoryPosition position = static_cast<double>(i);
        segments[head].endPosition = position;
        head++;
        segments[head].startPosition = position;
      }
    }
  }
  segments[head].endPosition = static_cast<double>(m_points.size() - 1);
  size_t numSegments = head + 1;

  head = 0;
  for (auto rotationIt = m_rotations.begin(); rotationIt != m_rotations.end(); rotationIt++) {
    while (segments[head].endPosition < rotationIt->first) {
      head++;
    }
    RotationSegment& segment = segments[head];
    segment.rotationIts.push_back(rotationIt);
  }

  for (head = 0; head < numSegments; head++) {
    const RotationSegment& segment = segments[head];

    const size_t segmentStartPointIndex =
        trajectoryPositionData->trajectoryPositionToPointIndex(segment.startPosition);
    const size_t segmentEndPointIndex =
        trajectoryPositionData->trajectoryPositionToPointIndex(segment.endPosition);
    const ThunderAutoPartialOutputTrajectoryPoint& segmentStartPoint =
        trajectoryPositionData->points.at(segmentStartPointIndex);
    const ThunderAutoPartialOutputTrajectoryPoint& segmentEndPoint =
        trajectoryPositionData->points.at(segmentEndPointIndex);
    units::meter_t segmentDistance = segmentEndPoint.distance - segmentStartPoint.distance;
    ThunderLibCoreAssert(segmentDistance > 0.0_m);

    const size_t numSegmentRotations = segment.rotationIts.size() + 1;
    if (numSegmentRotations == 1)
      continue;

    const units::meter_t segmentMinSeparationDistance =
        std::min(minSeparationDistance, segmentDistance / static_cast<double>(numSegmentRotations));
    const units::meter_t segmentTotalFreeDistance =
        segmentDistance - (segmentMinSeparationDistance * numSegmentRotations);

    units::meter_t segmentRemainingFreeDistance = segmentTotalFreeDistance;

    size_t lastPointIndex = segmentStartPointIndex;
    units::meter_t lastPositionDistance = segmentStartPoint.distance;

    // TODO: Make separation better by iterating inwards from both ends instead of iterating start->end.
    for (auto rotationIt : segment.rotationIts) {
      const size_t pointIndex = trajectoryPositionData->trajectoryPositionToPointIndex(rotationIt->first);
      const ThunderAutoPartialOutputTrajectoryPoint& point = trajectoryPositionData->points.at(pointIndex);
      units::meter_t positionDistance = point.distance;

      units::meter_t deltaDistance = positionDistance - lastPositionDistance;

      ThunderLibCoreAssert(deltaDistance >= 0.0_m);

      if (deltaDistance <= segmentMinSeparationDistance) {
        positionDistance = lastPositionDistance + segmentMinSeparationDistance;
      } else {
        units::meter_t addedDistance = deltaDistance - segmentMinSeparationDistance;
        if (addedDistance > segmentRemainingFreeDistance) {
          deltaDistance = segmentMinSeparationDistance + segmentRemainingFreeDistance;
          segmentRemainingFreeDistance = 0.0_m;
        } else {
          segmentRemainingFreeDistance -= addedDistance;
        }
        positionDistance = lastPositionDistance + deltaDistance;
      }

      auto lastPointIt = trajectoryPositionData->points.begin() + lastPointIndex;
      auto newPointIt = std::lower_bound(lastPointIt, trajectoryPositionData->points.end(), positionDistance,
                                         [](const ThunderAutoPartialOutputTrajectoryPoint& point,
                                            units::meter_t distance) { return point.distance < distance; });

      size_t newPointIndex = newPointIt - trajectoryPositionData->points.begin();

      ThunderAutoTrajectoryPosition position =
          trajectoryPositionData->pointIndexToTrajectoryPosition(newPointIndex);

      lastPointIndex = newPointIndex;
      lastPositionDistance = positionDistance;

      rotationIt = m_rotations.move(rotationIt, position);
    }
  }
}

void ThunderAutoTrajectorySkeleton::purgeOutOfBoundsRotationsAndActions() {
  ThunderAutoTrajectoryPosition maxPosition = static_cast<double>(m_points.size() - 1);

  for (auto it = m_rotations.begin(); it != m_rotations.end();) {
    auto& [position, angle] = *it;

    if (position <= 0.0 || position >= maxPosition) {
      ThunderLibCoreLogger::Info("Purging out-of-bounds rotation target at position {}", (double)it->first);
      it = m_rotations.remove(it);
    } else {
      ++it;
    }
  }

  for (auto it = m_actions.begin(); it != m_actions.end();) {
    auto& [position, action] = *it;

    if (position <= 0.0 || position >= maxPosition) {
      ThunderLibCoreLogger::Info("Purging out-of-bounds action at position {}", (double)it->first);
      it = m_actions.remove(it);
    } else {
      ++it;
    }
  }
}

void ThunderAutoTrajectorySkeleton::shiftRotationAndActionPositions(
    ThunderAutoTrajectoryPosition shiftStartPosition,
    double shiftAmount) {
  if (shiftAmount == 0.0)
    return;

  ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryRotation> newRotations;

  for (const auto& [position, rotation] : m_rotations) {
    ThunderAutoTrajectoryPosition newPosition = position;
    if (position >= shiftStartPosition) {
      newPosition.position += shiftAmount;
    }
    newRotations.add(newPosition, rotation);
  }

  m_rotations = std::move(newRotations);

  ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryAction> newActions;

  for (const auto& [position, action] : m_actions) {
    ThunderAutoTrajectoryPosition newPosition = position;
    if (position >= shiftStartPosition) {
      newPosition.position += shiftAmount;
    }
    newActions.add(newPosition, action);
  }

  m_actions = std::move(newActions);
}

static void to_json(wpi::json& json, const ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles& angles) {
  json = wpi::json{
      {"incoming_angle", angles.incomingAngle()},
      {"outgoing_angle", angles.outgoingAngle()},
  };
}

static void from_json(const wpi::json& json, ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles& angles) {
  CanonicalAngle incoming, outgoing;
  json.at("incoming_angle").get_to(incoming);
  json.at("outgoing_angle").get_to(outgoing);
  angles = ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles(incoming, outgoing);
}

static void to_json(wpi::json& json, const ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights& weights) {
  json = wpi::json{
      {"incoming_weight", weights.incomingWeight()},
      {"outgoing_weight", weights.outgoingWeight()},
  };
}

static void from_json(const wpi::json& json, ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights& weights) {
  double incomingWeight, outgoingWeight;
  json.at("incoming_weight").get_to(incomingWeight);
  json.at("outgoing_weight").get_to(outgoingWeight);
  weights = ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights(incomingWeight, outgoingWeight);
}

static void to_json(wpi::json& json, const ThunderAutoTrajectorySkeletonWaypoint& point) {
  json = wpi::json{
      {"position", point.position()},
      {"headings", point.headings()},
      {"heading_weights", point.headingWeights()},
      {"editor_locked", point.isEditorLocked()},
      {"link", point.linkName()},
  };
  if (point.hasMaxVelocityOverride()) {
    json["max_velocity_override"] = point.maxVelocityOverride()->value();

    if (point.isStopped()) {
      json["stop_rotation"] = point.stopRotation();
      if (!point.stopAction().empty()) {
        json["stop_action"] = point.stopAction();
      }
    }
  }
}

static void from_json(const wpi::json& json, ThunderAutoTrajectorySkeletonWaypoint& point) {
  point = ThunderAutoTrajectorySkeletonWaypoint();

  if (json.contains("max_velocity_override")) {
    double maxVelocityOverride;
    json.at("max_velocity_override").get_to(maxVelocityOverride);
    point.setMaxVelocityOverride(units::meters_per_second_t(maxVelocityOverride));

    if (point.isStopped()) {
      CanonicalAngle stopRotation;
      json.at("stop_rotation").get_to(stopRotation);
      point.setStopRotation(stopRotation);

      if (json.contains("stop_action")) {
        std::string stopAction;
        json.at("stop_action").get_to(stopAction);
        point.setStopAction(stopAction);
      }
    }
  }

  Point2d position;
  json.at("position").get_to(position);
  point.setPosition(position);

  ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles headings;
  json.at("headings").get_to(headings);
  point.setHeadings(headings);

  ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights headingWeights;
  json.at("heading_weights").get_to(headingWeights);
  point.setHeadingWeights(headingWeights);

  if (json.contains("link")) {
    std::string link = json.at("link").get<std::string>();
    if (!link.empty()) {
      point.setLinkName(link);
    }
  }

  if (json.contains("editor_locked")) {
    bool editorLocked = json.at("editor_locked").get<bool>();
    point.setEditorLocked(editorLocked);
  }
}

static void to_json(wpi::json& json, const ThunderAutoTrajectorySkeletonSettings& settings) {
  json = wpi::json{
      {"max_linear_velocity", settings.maxLinearVelocity.value()},
      {"max_linear_accel", settings.maxLinearAcceleration.value()},
      {"max_centripetal_accel", settings.maxCentripetalAcceleration.value()},
      {"max_angular_velocity", settings.maxAngularVelocity.value()},
      {"max_angular_accel", settings.maxAngularAcceleration.value()},
  };
}

static void from_json(const wpi::json& json, ThunderAutoTrajectorySkeletonSettings& settings) {
  settings = ThunderAutoTrajectorySkeletonSettings();

  double maxLinearVelocity, maxLinearAccel, maxCentripetalAccel, maxAngularVelocity, maxAngularAccel;
  json.at("max_linear_velocity").get_to(maxLinearVelocity);
  json.at("max_linear_accel").get_to(maxLinearAccel);
  json.at("max_centripetal_accel").get_to(maxCentripetalAccel);
  json.at("max_angular_velocity").get_to(maxAngularVelocity);
  json.at("max_angular_accel").get_to(maxAngularAccel);

  settings.maxLinearVelocity = units::meters_per_second_t(maxLinearVelocity);
  settings.maxLinearAcceleration = units::meters_per_second_squared_t(maxLinearAccel);
  settings.maxCentripetalAcceleration = units::meters_per_second_squared_t(maxCentripetalAccel);
  settings.maxAngularVelocity = units::radians_per_second_t(maxAngularVelocity);
  settings.maxAngularAcceleration = units::radians_per_second_squared_t(maxAngularAccel);
}

// Wrapper for pre-2026 settings to distinguish between from_json functions.
struct ThunderAutoTrajectorySkeletonSettingsPre2026 {
  ThunderAutoTrajectorySkeletonSettings settings;
};

static void from_json(const wpi::json& json, ThunderAutoTrajectorySkeletonSettingsPre2026& settingsWrapper) {
  ThunderAutoTrajectorySkeletonSettings& settings = settingsWrapper.settings;
  settings = ThunderAutoTrajectorySkeletonSettings();

  double maxLinearVelocity, maxLinearAccel, maxCentripetalAccel;
  json.at("max_linear_vel").get_to(maxLinearVelocity);
  json.at("max_linear_accel").get_to(maxLinearAccel);
  json.at("max_centripetal_accel").get_to(maxCentripetalAccel);

  settings.maxLinearVelocity = units::meters_per_second_t(maxLinearVelocity);
  settings.maxLinearAcceleration = units::meters_per_second_squared_t(maxLinearAccel);
  settings.maxCentripetalAcceleration = units::meters_per_second_squared_t(maxCentripetalAccel);
}

static void to_json(wpi::json& json, const ThunderAutoTrajectoryPosition& position) {
  json = (double)position.position;
}

static void from_json(const wpi::json& json, ThunderAutoTrajectoryPosition& position) {
  double pos;
  json.get_to(pos);
  position.position = pos;
}

template <typename T>
static void to_json(wpi::json& json, const ThunderAutoPositionedTrajectoryItemList<T>& list) {
  for (const auto& [pos, item] : list) {
    wpi::json itemJson;
    to_json(itemJson, item);
    json.push_back(wpi::json{{"position", pos}, {"item", itemJson}});
  }
}

template <typename T>
static void from_json(const wpi::json& json, ThunderAutoPositionedTrajectoryItemList<T>& list) {
  for (const auto& entry : json) {
    ThunderAutoTrajectoryPosition pos;
    entry.at("position").get_to(pos);
    T item;
    from_json(entry.at("item"), item);
    list.add(pos, item);
  }
}

static void to_json(wpi::json& json, const ThunderAutoTrajectoryRotation& rotation) {
  json = wpi::json{
      {"angle", rotation.angle},
      {"editor_locked", rotation.editorLocked},
  };
}

static void from_json(const wpi::json& json, ThunderAutoTrajectoryRotation& rotation) {
  json.at("angle").get_to(rotation.angle);
  if (json.contains("editor_locked")) {
    json.at("editor_locked").get_to(rotation.editorLocked);
  }
}

static void to_json(wpi::json& json, const ThunderAutoTrajectoryAction& action) {
  json = wpi::json{
      {"action", action.action},
      {"editor_locked", action.editorLocked},
      // TODO: zone end position
  };
}

static void from_json(const wpi::json& json, ThunderAutoTrajectoryAction& action) {
  json.at("action").get_to(action.action);
  if (json.contains("editor_locked")) {
    json.at("editor_locked").get_to(action.editorLocked);
  }
  // TODO: zone end position
}

void to_json(wpi::json& json, const ThunderAutoTrajectorySkeleton& trajectory) {
  json = wpi::json{
      {"settings", trajectory.settings()},
      {"points", trajectory.points()},
      {"start_rotation", trajectory.startRotation()},
      {"end_rotation", trajectory.endRotation()},
  };

  const auto& rotations = trajectory.rotations();
  if (!rotations.empty()) {
    json["rotations"] = rotations;
  }

  const std::string& startAction = trajectory.startAction();
  if (!startAction.empty()) {
    json["start_action"] = startAction;
  }

  const std::string& endAction = trajectory.endAction();
  if (!endAction.empty()) {
    json["end_action"] = endAction;
  }

  const auto& actions = trajectory.actions();
  if (!actions.empty()) {
    json["actions"] = actions;
  }

  const std::string& startBehaviorLink = trajectory.startBehaviorLinkName();
  if (!startBehaviorLink.empty()) {
    json["start_behavior_link"] = startBehaviorLink;
  }

  const std::string& endBehaviorLink = trajectory.endBehaviorLinkName();
  if (!endBehaviorLink.empty()) {
    json["end_behavior_link"] = endBehaviorLink;
  }
}

void ThunderAutoTrajectorySkeleton::fromJson(const wpi::json& json) {
  json.at("settings").get_to(m_settings);
  json.at("points").get_to(m_points);

  CanonicalAngle startRotation, endRotation;
  json.at("start_rotation").get_to(startRotation);
  json.at("end_rotation").get_to(endRotation);
  setStartRotation(startRotation);
  setEndRotation(endRotation);

  if (json.contains("start_behavior_link")) {
    json.at("start_behavior_link").get_to(m_startBehaviorLink);
  }

  if (json.contains("end_behavior_link")) {
    json.at("end_behavior_link").get_to(m_endBehaviorLink);
  }

  if (json.contains("rotations")) {
    json.at("rotations").get_to(m_rotations);
  }

  if (json.contains("start_action")) {
    json.at("start_action").get_to(m_startAction);
  }

  if (json.contains("end_action")) {
    json.at("end_action").get_to(m_endAction);
  }

  if (json.contains("actions")) {
    json.at("actions").get_to(m_actions);
  }
}

void ThunderAutoTrajectorySkeleton::fromJsonPre2026(const wpi::json& json,
                                                    std::span<std::string> availableActions,
                                                    std::span<std::string> availableWaypointLinks) {
  struct PositionedRotation {
    ThunderAutoTrajectoryPosition position;
    CanonicalAngle angle;
  };
  struct PositionedAction {
    ThunderAutoTrajectoryPosition position;
    std::string action;
  };

  struct Pre2026TrajectoryRotation {
    PositionedRotation positionedRotation;
    size_t waypointIndex;
    bool doNotAdd = false;
  };

  std::list<Pre2026TrajectoryRotation> rotations;
  std::list<PositionedAction> positionedActions;

  if (json.contains("points")) {
    wpi::json pointsJson = json.at("points");

    // Waypoints
    size_t pointIndex = 0;
    for (const wpi::json& pointJson : pointsJson) {
      ThunderAutoTrajectorySkeletonWaypoint point;

      double x, y;
      pointJson.at("x").get_to(x);
      pointJson.at("y").get_to(y);
      Point2d position = {units::meter_t(x), units::meter_t(y)};
      point.setPosition(position);

      bool stop = false;
      if (pointJson.contains("stop")) {
        stop = pointJson.at("stop").get<bool>();
        point.setStopped(stop);
      }

      double incomingHeadingRadians = 0.0, outgoingHeadingRadians = 0.0;
      if (pointJson.contains("h0")) {
        pointJson.at("h0").get_to(incomingHeadingRadians);
      }
      if (pointJson.contains("h1")) {
        pointJson.at("h1").get_to(outgoingHeadingRadians);
      }
      CanonicalAngle incomingHeading = units::radian_t(incomingHeadingRadians);
      CanonicalAngle outgoingHeading = units::radian_t(outgoingHeadingRadians);
      ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles headings(incomingHeading, outgoingHeading);
      point.setHeadings(headings);

      double incomingWeight = 1.0, outgoingWeight = 1.0;
      if (pointJson.contains("w0")) {
        pointJson.at("w0").get_to(incomingWeight);
      }
      if (pointJson.contains("w1")) {
        pointJson.at("w1").get_to(outgoingWeight);
      }
      ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights headingWeights(incomingWeight, outgoingWeight);
      point.setHeadingWeights(headingWeights);

      if (pointJson.contains("locked")) {
        bool editorLocked = pointJson.at("locked").get<bool>();
        point.setEditorLocked(editorLocked);
      }

      double rotationRadians = 0.0;
      if (pointJson.contains("rotation")) {
        pointJson.at("rotation").get_to(rotationRadians);
      }
      CanonicalAngle rotation = units::radian_t(rotationRadians);

      double segmentRotationPercent = 1.0;

      if (pointIndex != 0 && pointJson.contains("segment_rotation_time_percent")) {
        segmentRotationPercent = pointJson.at("segment_rotation_time_percent").get<double>();
        // This won't be a perfect conversion, but its good enough.
      }

      segmentRotationPercent = std::clamp(segmentRotationPercent, 0.0, 1.0);

      double rotationPosition = static_cast<double>(pointIndex) + segmentRotationPercent - 1.0;

      /**
       * We don't want to add a rotation target if the point is stopped (set the point's stop rotation
       * instead). However, we may still need to set it as the trajectory's start or end rotation, so add it
       * to the list anyway but with `doNotAdd` set to true.
       */
      bool doNotAddRotation = false;
      if (stop) {
        point.setStopRotation(rotation);
        doNotAddRotation = true;
      }
      rotations.push_back(Pre2026TrajectoryRotation{.positionedRotation = {rotationPosition, rotation},
                                                    .waypointIndex = pointIndex,
                                                    .doNotAdd = doNotAddRotation});

      if (pointJson.contains("link_index")) {
        int linkIndexInt = pointJson.at("link_index").get<int>();
        if (linkIndexInt >= 0) {
          size_t linkIndex = static_cast<size_t>(linkIndexInt);
          if (linkIndex < availableWaypointLinks.size()) {
            std::string linkName = availableWaypointLinks[linkIndex];
            point.setLinkName(linkName);
          }
        }
      }

      if (pointJson.contains("actions")) {
        unsigned actionBits = pointJson.at("actions").get<unsigned>();
        for (size_t bit = 0; bit < availableActions.size(); bit++) {
          if (actionBits & (1U << bit)) {
            positionedActions.emplace_back(static_cast<double>(pointIndex), availableActions[bit]);
          }
        }
      }

      appendPoint(point);

      pointIndex++;
    }

    ThunderLibCoreAssert(m_points.size() >= 2);
    /**
     * Setting the start/end points to stopped was not an option in pre-2026 versions.
     * However, they might still might show up as stopped in the save file.
     */
    m_points.front().setStopped(false);
    m_points.back().setStopped(false);

    for (const auto& [positionedRotation, waypointIndex, doNotAdd] : rotations) {
      if (waypointIndex == 0) {
        setStartRotation(positionedRotation.angle);
      } else if (waypointIndex == rotations.size() - 1) {
        setEndRotation(positionedRotation.angle);
      } else if (!doNotAdd) {
        ThunderAutoTrajectoryRotation newRotation{.angle = positionedRotation.angle};
        m_rotations.add(positionedRotation.position, newRotation);
      }
    }

    for (const auto& [position, action] : positionedActions) {
      if (position == 0.0 && !hasStartAction()) {  // Only one start action allowed, so additional ones are
                                                   // added as normal actions.
        setStartAction(action);
      } else if (DoubleEquals(position, static_cast<double>(m_points.size() - 1)) && !hasEndAction()) {
        setEndAction(action);
      } else {
        ThunderAutoTrajectoryAction newAction{.action = action};
        m_actions.add(position, newAction);
      }
    }
  }

  if (json.contains("settings")) {
    ThunderAutoTrajectorySkeletonSettingsPre2026 settingsWrapper;
    json.at("settings").get_to(settingsWrapper);
    setSettings(settingsWrapper.settings);
  }
}

void from_json(const wpi::json& json, ThunderAutoTrajectorySkeleton& trajectory) {
  trajectory = ThunderAutoTrajectorySkeleton(json);
}

}  // namespace thunder::core
