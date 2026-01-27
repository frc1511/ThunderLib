#include <ThunderLibCore/Auto/ThunderAutoOutputTrajectory.hpp>

#include <ThunderLibCore/Error.hpp>
#include <ThunderLibCore/Math.hpp>

#include <wpi/timestamp.h>
#include <gcem.hpp>

#include <algorithm>
#include <fstream>
#include <unordered_map>

namespace thunder::core {

size_t ThunderAutoOutputTrajectory::trajectoryPositionToPointIndex(
    ThunderAutoTrajectoryPosition position) const {
  if (points.empty()) {
    throw OutOfRangeError::Construct("Output trajectory points list is empty");
  }

  if (segments.empty()) {
    throw RuntimeError::Construct("Segments list not set, cannot evaluate position");
  }

  size_t index;

  if (position.segmentIndex() < segments.size()) {
    ThunderAutoOutputTrajectorySegment segment = segments[position.segmentIndex()];
    size_t segmentNumPoints = (segment.endIndex - segment.startIndex) + 1;
    index = segment.startIndex + static_cast<size_t>(position.positionInSegment() * segmentNumPoints);
  } else {
    index = segments.back().endIndex;
  }

  return index;
}

ThunderAutoTrajectoryPosition ThunderAutoOutputTrajectory::pointIndexToTrajectoryPosition(
    size_t pointIndex) const {
  if (segments.empty()) {
    throw RuntimeError::Construct("Segments list not set, cannot evaluate position");
  }

  if (pointIndex > segments.back().endIndex) {
    return ThunderAutoTrajectoryPosition{static_cast<double>(segments.size() - 1)};
  }

  const ThunderAutoOutputTrajectoryPoint& point = points.at(pointIndex);

  // Binary search

  size_t left = 0;
  size_t right = segments.size() - 1;
  while (left <= right) {
    size_t mid = left + (right - left) / 2;
    const ThunderAutoOutputTrajectorySegment& segment = segments.at(mid);

    if (pointIndex >= segment.startIndex && pointIndex <= segment.endIndex) {  // Found!
      const ThunderAutoOutputTrajectoryPoint& segmentStartPoint = points.at(segment.startIndex);
      units::meter_t distanceInSegment = point.distance - segmentStartPoint.distance;

      double position = static_cast<double>(mid) + (distanceInSegment / segment.length);
      return ThunderAutoTrajectoryPosition{position};

    } else if (segment.endIndex < pointIndex) {
      left = mid + 1;
    } else {  // segment.startIndex > pointIndex
      right = mid - 1;
    }
  }

  throw LogicError::Construct("Segments list contains gap");
}

// Same as ThunderAutoOutputTrajectory::trajectoryPositionToPointIndex()
size_t ThunderAutoPartialOutputTrajectory::trajectoryPositionToPointIndex(
    ThunderAutoTrajectoryPosition position) const {
  if (points.empty()) {
    throw OutOfRangeError::Construct("Output trajectory points list is empty");
  }

  if (segments.empty()) {
    throw RuntimeError::Construct("Segments list not set, cannot evaluate position");
  }

  size_t index;

  if (position.segmentIndex() < segments.size()) {
    ThunderAutoOutputTrajectorySegment segment = segments[position.segmentIndex()];
    size_t segmentNumPoints = (segment.endIndex - segment.startIndex) + 1;
    index = segment.startIndex + static_cast<size_t>(position.positionInSegment() * segmentNumPoints);
  } else {
    index = segments.back().endIndex;
  }

  return index;
}

// Same as ThunderAutoOutputTrajectory::pointIndexToTrajectoryPosition()
ThunderAutoTrajectoryPosition ThunderAutoPartialOutputTrajectory::pointIndexToTrajectoryPosition(
    size_t pointIndex) const {
  if (segments.empty()) {
    throw RuntimeError::Construct("Segments list not set, cannot evaluate position");
  }

  if (pointIndex > segments.back().endIndex) {
    return ThunderAutoTrajectoryPosition{static_cast<double>(segments.size() - 1)};
  }

  const ThunderAutoPartialOutputTrajectoryPoint& point = points.at(pointIndex);

  // Binary search

  size_t left = 0;
  size_t right = segments.size() - 1;
  while (left <= right) {
    size_t mid = left + (right - left) / 2;
    const ThunderAutoOutputTrajectorySegment& segment = segments.at(mid);

    if (pointIndex >= segment.startIndex && pointIndex <= segment.endIndex) {  // Found!
      const ThunderAutoPartialOutputTrajectoryPoint& segmentStartPoint = points.at(segment.startIndex);
      units::meter_t distanceInSegment = point.distance - segmentStartPoint.distance;

      double position = static_cast<double>(mid) + (distanceInSegment / segment.length);
      return ThunderAutoTrajectoryPosition{position};

    } else if (segment.endIndex < pointIndex) {
      left = mid + 1;
    } else {  // segment.startIndex > pointIndex
      right = mid - 1;
    }
  }

  throw LogicError::Construct("Segments list contains gap");
}

using EquationFunc = std::function<Point2d(double)>;

/**
 * Builds a parametric function representing a Bezier curve. This function completes in constant O(1) time and
 * returns a function that can be used to evaluate the curve at any point t in the range [0, 1] in constant
 * O(1) time.
 *
 * @param startPoint
 * @param startControlPoint
 * @param endPoint
 * @param endControlPoint
 *
 * @return A Bézier curve function that takes a parameter t in the range [0, 1] and returns a Point2d
 */
static EquationFunc BuildBezierCurveEquation(const Point2d& startPoint,
                                             const Point2d& startControlPoint,
                                             const Point2d& endPoint,
                                             const Point2d& endControlPoint) {
  // https://en.wikipedia.org/wiki/Bézier_curve

  // B(t) = (1-t)^3 * P0 + 3(1-t)^2t * P1 + 3(1-t)t^2 * P2 + t^3 * P3
  auto B = [](double P0, double P1, double P2, double P3, double t) -> double {
    // (1-t)^3 * P0
    double B_0 = gcem::pow(1 - t, 3) * P0;

    // 3(1-t)^2t * P1
    double B_1 = 3 * gcem::pow(1 - t, 2) * t * P1;

    // 3(1-t)t^2 * P2
    double B_2 = 3 * (1 - t) * gcem::pow(t, 2) * P2;

    // t^3 * P3
    double B_3 = gcem::pow(t, 3) * P3;

    return B_0 + B_1 + B_2 + B_3;
  };

  EquationFunc f = [=](double t) -> Point2d {
    double x = B(startPoint.x(), startControlPoint.x(), endControlPoint.x(), endPoint.x(), t);
    double y = B(startPoint.y(), startControlPoint.y(), endControlPoint.y(), endPoint.y(), t);

    return Point2d(units::meter_t(x), units::meter_t(y));
  };

  return f;
}

/**
 * Calculates the Menger curvature of a curve defined by three points.
 *
 * @param a
 * @param b
 * @param c
 *
 * @return Curvature, defined as 1/radius.
 */
static units::curvature_t MengerCurvature(Point2d a, Point2d b, Point2d c) {
  // Side lengths.
  double ab = gcem::hypot(a.x() - b.x(), a.y() - b.y());
  double ac = gcem::hypot(a.x() - c.x(), a.y() - c.y());
  double bc = gcem::hypot(b.x() - c.x(), b.y() - c.y());

  // Semi-perimeter.
  double s = (ab + ac + bc) / 2.0;

  // Heron's formula.
  double radicand = s * (s - ab) * (s - ac) * (s - bc);

  // The radicand can never actually be negative or zero, but floating point math can be weird sometimes.
  if (radicand <= 0.0)
    return units::curvature_t{0.0};

  double A = gcem::sqrt(radicand);

  // Menger's curvature formula.
  double curvature = (4.0 * A) / (ab * ac * bc);

  return units::curvature_t{curvature};
}

/**
 * Samples points along a parametric function f(t) and calculates the length of the segment. This function
 * completes in O(n) time, where n is the number of samples.
 *
 * @param f The parametric function representing the segment.
 * @param samples The number of samples to take along the segment.
 * @param output The output trajectory segment to store the sampled points and length.
 */
static void SampleSegmentPoints(const EquationFunc& f, size_t samples, ThunderAutoOutputTrajectory& output) {
  ThunderLibCoreAssert(samples > 0);
  double delta = 1.0 / static_cast<double>(samples);

  ThunderAutoOutputTrajectorySegment segment{};

  Point2d previousPoint = f(0.0);
  segment.sampledPoints.emplace_back(0.0_m, previousPoint);

  for (size_t i = 1; i <= samples; i++) {
    double t = delta * static_cast<double>(i);

    Point2d point = f(t);

    segment.length += previousPoint.distanceTo(point);
    segment.sampledPoints.emplace_back(segment.length, point);

    previousPoint = point;
  }

  output.segments.push_back(segment);
}

// Same as above SampleSegmentPoints(), but for ThunderAutoPartialOutputTrajectory
static void SampleSegmentPoints(const EquationFunc& f,
                                size_t samples,
                                ThunderAutoPartialOutputTrajectory& output) {
  ThunderLibCoreAssert(samples > 0);
  double delta = 1.0 / static_cast<double>(samples);

  ThunderAutoOutputTrajectorySegment segment{};

  Point2d previousPoint = f(0.0);
  segment.sampledPoints.emplace_back(0.0_m, previousPoint);

  for (size_t i = 1; i <= samples; i++) {
    double t = delta * static_cast<double>(i);

    Point2d point = f(t);

    segment.length += previousPoint.distanceTo(point);
    segment.sampledPoints.emplace_back(segment.length, point);

    previousPoint = point;
  }

  output.segments.push_back(segment);
}

/**
 * Calculates the curvature at each point in a trajectory segment. This function completes in O(n) time, where
 * n is the number of points in the segment.
 *
 * This function assumes that curvatures have been previously initialized to zero.
 *
 * @param segment The trajectory segment to calculate curvatures for.
 */
static void CalculateSegmentCurvatures(ThunderAutoOutputTrajectorySegment& segment) {
  const size_t numPoints = segment.sampledPoints.size();
  if (numPoints < 3)
    return;

  Point2d previousPosition = segment.sampledPoints.front().position;
  for (size_t i = 1; i < numPoints - 1; i++) {
    Point2d position = segment.sampledPoints[i].position;
    Point2d nextPosition = segment.sampledPoints[i + 1].position;

    units::curvature_t curvature = MengerCurvature(previousPosition, position, nextPosition);

    segment.sampledPoints[i].curvature = curvature;

    previousPosition = position;
  }

  segment.sampledPoints.front().curvature = units::curvature_t{0.0};
  segment.sampledPoints.back().curvature = units::curvature_t{0.0};
}

/**
 * Evenly resamples the points of each trajectory segment. This function completes in O(n) time, where n is
 * the number of samples per meter times the total length of the trajectory.
 *
 * @param settings The trajectory skeleton settings, used to provide the max linear velocity to initialize
 *                 each point with.
 * @param samplesPerMeter Number of points to sample per meter.
 * @param output The output trajectory to append points to.
 */
static void ResamplePoints(const ThunderAutoTrajectorySkeletonSettings& settings,
                           size_t samplesPerMeter,
                           ThunderAutoOutputTrajectory& output) {
  ThunderLibCoreAssert(!output.segments.empty());

  for (auto segmentIt = output.segments.begin(); segmentIt != output.segments.end(); segmentIt++) {
    ThunderAutoOutputTrajectorySegment& segment = *segmentIt;

    size_t samples = static_cast<size_t>(segment.length.value() * static_cast<double>(samplesPerMeter));
    ThunderLibCoreAssert(samples > 0);
    ThunderLibCoreAssert(segment.length == segment.sampledPoints.back().distance);

    units::meter_t deltaDistance = segment.length / static_cast<double>(samples);

    const bool isLastSegment = (std::next(segmentIt) == output.segments.end());
    if (isLastSegment) {
      /**
       * Add the last point of the segment. We don't usually add it because it overlaps with the first point
       * of the next segment.
       */
      ++samples;
    }

    output.points.reserve(output.points.size() + samples);

    segment.startIndex = output.points.size();
    segment.endIndex = segment.startIndex + samples - 1;

    size_t lowerPointIndex = 0;

    for (size_t i = 0; i < samples; i++) {
      units::meter_t d = deltaDistance * i;

      size_t upperPointIndex = lowerPointIndex + 1;

      units::curvature_t lowerPointCurvature = segment.sampledPoints[lowerPointIndex].curvature;

      for (; upperPointIndex < segment.sampledPoints.size() - 1 &&
             segment.sampledPoints[upperPointIndex].distance < d;
           upperPointIndex++) {
        lowerPointCurvature = std::max(lowerPointCurvature, segment.sampledPoints[upperPointIndex].curvature);
      }
      lowerPointIndex = upperPointIndex - 1;

      const units::meter_t& lowerPointDistance = segment.sampledPoints[lowerPointIndex].distance;
      const units::meter_t& upperPointDistance = segment.sampledPoints[upperPointIndex].distance;

      double t = (d - lowerPointDistance) / (upperPointDistance - lowerPointDistance);

      ThunderAutoOutputTrajectoryPoint point{};

      const Point2d& lowerPointPosition = segment.sampledPoints[lowerPointIndex].position;
      const Point2d& upperPointPosition = segment.sampledPoints[upperPointIndex].position;

      point.position = lowerPointPosition + (upperPointPosition - lowerPointPosition) * t;  // lerp

      Point2d lastPosition = point.position;
      if (!output.points.empty()) {
        lastPosition = output.points.back().position;
      }

      point.distance = lastPosition.distanceTo(point.position);

      if (!output.points.empty()) {
        point.distance += output.points.back().distance;
      }

      // const units::curvature_t& upperPointCurvature = segment.sampledPoints[upperPointIndex].curvature;
      // point.curvature = lerp(lowerPointCurvature, upperPointCurvature, t);

      if (i != 0 && i != (samples - 1)) {
        point.curvature = lowerPointCurvature;
      }

      point.linearVelocity = settings.maxLinearVelocity;

      output.points.push_back(point);
    }

    ThunderLibCoreAssert(segment.endIndex == (output.points.size() - 1));

    // Length might be slightly different after resampling the trajectory, so make sure it's accurate.
    segment.length = output.points[segment.endIndex].distance - output.points[segment.startIndex].distance;

    output.totalDistance += segment.length;
  }
}

// Same as above ResamplePoints(), but for ThunderAutoPartialOutputTrajectory
static void ResamplePoints(size_t samplesPerMeter, ThunderAutoPartialOutputTrajectory& output) {
  ThunderLibCoreAssert(!output.segments.empty());

  for (auto segmentIt = output.segments.begin(); segmentIt != output.segments.end(); segmentIt++) {
    ThunderAutoOutputTrajectorySegment& segment = *segmentIt;

    size_t samples = static_cast<size_t>(segment.length.value() * static_cast<double>(samplesPerMeter));
    ThunderLibCoreAssert(samples > 0);
    ThunderLibCoreAssert(segment.length == segment.sampledPoints.back().distance);

    units::meter_t deltaDistance = segment.length / static_cast<double>(samples);

    const bool isLastSegment = (std::next(segmentIt) == output.segments.end());
    if (isLastSegment) {
      /**
       * Add the last point of the segment. We don't usually add it because it overlaps with the first point
       * of the next segment.
       */
      ++samples;
    }

    output.points.reserve(output.points.size() + samples);

    segment.startIndex = output.points.size();
    segment.endIndex = segment.startIndex + samples - 1;

    size_t lowerPointIndex = 0;

    for (size_t i = 0; i < samples; i++) {
      units::meter_t d = deltaDistance * i;

      size_t upperPointIndex = lowerPointIndex + 1;

      for (; upperPointIndex < segment.sampledPoints.size() - 1 &&
             segment.sampledPoints[upperPointIndex].distance < d;
           upperPointIndex++) {
        ThunderLibCoreAssert(upperPointIndex != segment.sampledPoints.size());
      }
      lowerPointIndex = upperPointIndex - 1;

      const units::meter_t& lowerPointDistance = segment.sampledPoints[lowerPointIndex].distance;
      const units::meter_t& upperPointDistance = segment.sampledPoints[upperPointIndex].distance;

      double t = (d - lowerPointDistance) / (upperPointDistance - lowerPointDistance);

      ThunderAutoPartialOutputTrajectoryPoint point{};

      const Point2d& lowerPointPosition = segment.sampledPoints[lowerPointIndex].position;
      const Point2d& upperPointPosition = segment.sampledPoints[upperPointIndex].position;

      point.position = lowerPointPosition + (upperPointPosition - lowerPointPosition) * t;  // lerp

      Point2d lastPosition = point.position;
      if (!output.points.empty()) {
        lastPosition = output.points.back().position;
      }

      point.distance = lastPosition.distanceTo(point.position);

      if (!output.points.empty()) {
        point.distance += output.points.back().distance;
      }

      output.points.push_back(point);
    }

    ThunderLibCoreAssert(segment.endIndex == (output.points.size() - 1));

    // Length might be slightly different after resampling the trajectory, so make sure it's accurate.
    segment.length = output.points[segment.endIndex].distance - output.points[segment.startIndex].distance;

    output.totalDistance += segment.length;
  }
}

/**
 * Adds max linear velocity constraints based on the maximum velocities specified in the waypoints. This
 * function completes in O(n) time, where n is the number of waypoints.
 *
 * @param waypoints The list of waypoints in the trajectory skeleton.
 * @param segments The segments of the trajectory, used to map each waypoint to its corresponding output point
 *                 index.
 * @param maxVelocities The list of maximum linear velocities correlating to each point in the output
 *                      trajectory. This will be updated with the maximum velocities from the waypoints.
 */
static void AddWaypointMaxVelocityOverrideConstraints(
    const std::list<ThunderAutoTrajectorySkeletonWaypoint>& waypoints,
    const std::vector<ThunderAutoOutputTrajectorySegment>& segments,
    std::span<units::meters_per_second_t> maxVelocities) {
  auto waypointIt = waypoints.cbegin();
  auto segmentIt = segments.cbegin();

  for (size_t waypointIndex = 0; waypointIndex < waypoints.size();
       ++waypointIndex, ++waypointIt, ++segmentIt) {
    bool isFirstWaypoint = (waypointIndex == 0);
    bool isLastWaypoint = (!isFirstWaypoint && (waypointIndex == waypoints.size() - 1));

    size_t outputPointIndex;
    if (isLastWaypoint) {
      outputPointIndex = maxVelocities.size() - 1;  // Last point in the trajectory.
    } else {
      outputPointIndex = segmentIt->startIndex;
    }

    if (waypointIt->hasMaxVelocityOverride()) {
      maxVelocities[outputPointIndex] =
          std::min(maxVelocities[outputPointIndex], *waypointIt->maxVelocityOverride());

    } else if (isFirstWaypoint || isLastWaypoint) {
      maxVelocities[outputPointIndex] = 0_mps;
    }
  }
}

/**
 * Adds max linear velocity constraints in areas of high curvature where the centripetal acceleration would
 * exceed the maximum allowed. This function completes in O(n) time, where n is the number of points in the
 * trajectory.
 *
 * @param settings The trajectory skeleton settings, containing the maximum centripetal acceleration.
 * @param output The output trajectory containing curvatures.
 * @param maxVelocities The list of maximum linear velocities correlating to each point in the output
 *                      trajectory. This will be updated depending on the curvature.
 */
static void AddCentripetalAccelerationMaxVelocityConstraints(
    const ThunderAutoTrajectorySkeletonSettings& settings,
    const ThunderAutoOutputTrajectory& output,
    std::span<units::meters_per_second_t> maxVelocities) {
  ThunderLibCoreAssert(maxVelocities.size() == output.points.size());

  for (size_t i = 0; i < output.points.size(); i++) {
    const ThunderAutoOutputTrajectoryPoint& point = output.points[i];

    double curvature = point.curvature.value();

    if (curvature > 0.0) {
      units::meter_t radius{1.0 / curvature};

      // Maximum velocity that doesn't exceed the centripetal acceleration.
      // v^2 = a_c * r
      units::meters_per_second_t maxVelocity{
          gcem::sqrt(settings.maxCentripetalAcceleration.value() * radius.value())};

      maxVelocities[i] = std::min(maxVelocities[i], maxVelocity);
    }
  }
}

/**
 * Returns a sorted list of all rotation targets in the given trajectory skeleton. This function completes in
 * O(n) time, where n is the number of waypoints in the skeleton.
 *
 * @param skeleton The trajectory skeleton to extract rotation targets from.
 *
 * @return A list of all the positioned rotation targets in the trajectory.
 */
static std::map<ThunderAutoTrajectoryPosition, CanonicalAngle> GetAllRotationTargets(
    const ThunderAutoTrajectorySkeleton& skeleton) {
  const std::list<ThunderAutoTrajectorySkeletonWaypoint>& waypoints = skeleton.points();

  ThunderAutoTrajectoryPosition maxPosition = static_cast<double>(waypoints.size() - 1);

  std::map<ThunderAutoTrajectoryPosition, CanonicalAngle> rotations;
  for (auto rot : skeleton.rotations()) {
    ThunderAutoTrajectoryPosition position = rot.first;
    if (position >= maxPosition) {
      ThunderLibCoreLogger::Warn(
          "Trajectory build - Ignoring rotation target past the end of the trajectory ({})",
          (double)position);
      continue;
    }
    if (position <= 0.0) {
      ThunderLibCoreLogger::Warn("Trajectory build - Ignoring rotation target below zero ({})",
                                 (double)position);
      continue;
    }

    rotations.emplace(position, rot.second.angle);
  }

  rotations.emplace(ThunderAutoTrajectoryPosition(0.0), skeleton.startRotation());
  rotations.emplace(maxPosition, skeleton.endRotation());

  if (waypoints.size() > 2) {
    size_t waypointIndex = 1;
    for (auto it = std::next(waypoints.begin()); it != std::prev(waypoints.end()); ++it, ++waypointIndex) {
      if (it->isStopped()) {
        rotations.emplace(ThunderAutoTrajectoryPosition(static_cast<double>(waypointIndex)), it->stopRotation());
      }
    }
  }

  return rotations;
}

/**
 * Adds max linear velocity constraints in between rotation targets. This ensures that the robot will not
 * exceed the configured angular velocity and acceleration maximums. This function completes in O(n) time,
 * where n is the number of rotation targets in the trajectory skeleton.
 *
 * @param settings The trajectory skeleton settings, containing the maximum angular velocity and acceleration.
 * @param rotations The sorted list of positioned rotation targets in the trajectory.
 * @param output The output trajectory containing point distances.
 * @param maxVelocities The list of maximum linear velocities correlating to each point in the output
 *                      trajectory. This will be updated.
 */
static void AddRotationMaxVelocityConstraints(
    const ThunderAutoTrajectorySkeletonSettings& settings,
    const std::map<ThunderAutoTrajectoryPosition, CanonicalAngle>& rotations,
    const ThunderAutoOutputTrajectory& output,
    std::span<units::meters_per_second_t> maxVelocities) {
  if (rotations.size() < 2) {
    throw InvalidArgumentError::Construct(
        "At least two rotation targets are required to calculate rotation constraints");
  }

  auto startRotationIt = rotations.cbegin();
  auto endRotationIt = std::next(startRotationIt);

  for (; endRotationIt != rotations.cend(); ++startRotationIt, ++endRotationIt) {
    const auto& [startPosition, startRotation] = *startRotationIt;
    const auto& [endPosition, endRotation] = *endRotationIt;
    ThunderLibCoreAssert(startPosition < endPosition);

    size_t startOutputIndex;
    {
      ThunderLibCoreAssert(startPosition.segmentIndex() < output.segments.size());

      ThunderAutoOutputTrajectorySegment segment = output.segments[startPosition.segmentIndex()];
      size_t segmentNumPoints = (segment.endIndex - segment.startIndex) + 1;
      startOutputIndex =
          segment.startIndex + static_cast<size_t>(startPosition.positionInSegment() * segmentNumPoints);
    }

    size_t endOutputIndex;
    {
      if (endPosition.segmentIndex() < output.segments.size()) {
        ThunderAutoOutputTrajectorySegment segment = output.segments[endPosition.segmentIndex()];
        size_t segmentNumPoints = (segment.endIndex - segment.startIndex) + 1;
        endOutputIndex =
            segment.startIndex + static_cast<size_t>(endPosition.positionInSegment() * segmentNumPoints);
      } else {
        endOutputIndex = output.segments.back().endIndex;
      }
    }

    /**
     * It's possible for the startOutputIndex to be equal to the endOutputIndex if the build sample setting is
     * low enough and/or the rotations are placed very close together. This is obviously not ideal, so the
     * user should avoid constructing trajectories like this.
     */
    ThunderLibCoreAssert(startOutputIndex <= endOutputIndex);

    units::second_t timeTotal;
    {
      /**
       * Calculate rotation velocity profile to determine the minimum time required to complete the rotation
       * while maintaining the given angular velocity and acceleration maximums.
       */

      // Total distance (radians) that the robot is rotating (magnitude).
      units::radian_t angularDistanceTotal = units::math::abs((endRotation - startRotation).radians());
      if (angularDistanceTotal < 1e-6_rad)
        continue;

      // Distance (radians) that the robot is rotating at max angular velocity.
      units::radian_t angularDistanceCruise;

      units::radians_per_second_t peakAngularVelocity =
          units::math::sqrt(settings.maxAngularAcceleration * angularDistanceTotal);
      if (peakAngularVelocity > settings.maxAngularVelocity) {
        peakAngularVelocity = settings.maxAngularVelocity;
        units::radian_t angularDistanceAccel =
            units::math::pow<2>(peakAngularVelocity) / (2.0 * settings.maxAngularAcceleration);
        angularDistanceCruise = angularDistanceTotal - 2.0 * angularDistanceAccel;
      } else {
        angularDistanceCruise = 0.0_rad;
      }

      units::second_t timeAccel = peakAngularVelocity / settings.maxAngularAcceleration;
      units::second_t timeCruise = angularDistanceCruise / peakAngularVelocity;

      timeTotal = timeAccel + timeCruise + timeAccel;
    }

    {
      /**
       * Constrain the linear velocity when rotating so that there is enough time to complete the rotation
       * while maintaining the given angular velocity and acceleration maximums.
       *
       * The current solution in place here is very simple and is guarateed to slow down the robot enough to
       * complete the rotation. However, it makes the assumption that the robot will travel at the constant
       * speed for the entire time between the start and end rotation targets. This may not always be the case
       * in situations where velocity overrides are used at waypoints, which may lead to suboptimal output.
       * This most likely shouldn't be a problem except when making some seriously crazy trajectories.
       *
       * A more robust solution would likely be considerably more involved. Something to look into at some
       * point in the future.
       */

      units::meter_t linearDistanceTotal =
          output.points[endOutputIndex].distance - output.points[startOutputIndex].distance;
      ThunderLibCoreAssert(linearDistanceTotal >= 0.0_m);

      units::meters_per_second_t maxLinearVelocity = linearDistanceTotal / timeTotal;

      // Update the maximum velocity for all points in between the start and end output indices.
      for (size_t i = startOutputIndex; i <= endOutputIndex; ++i) {
        maxVelocities[i] = std::min(maxVelocities[i], maxLinearVelocity);
      }
    }
  }
}

/**
 * Calculates the linear velocities for each point in the trajectory based on the given maximum velocities.
 * This function completes in O(n) time, where n is the number of points in the output trajectory.
 *
 * @param settings The trajectory skeleton settings.
 * @param maxVelocities The list of maximum velocities correlating to each point in the output trajectory.
 * @param output The output trajectory to update with linear velocities.
 */
static void CalculateLinearVelocities(const ThunderAutoTrajectorySkeletonSettings& settings,
                                      std::span<const units::meters_per_second_t> maxVelocities,
                                      ThunderAutoOutputTrajectory& output) {
  ThunderLibCoreAssert(output.points.size() == maxVelocities.size());

  const size_t numPoints = output.points.size();

  // Output points should have all been previously be set to settings.maxLinearVelocity

  for (size_t i = 0; i < numPoints; ++i) {
    const ThunderAutoOutputTrajectoryPoint& point = output.points[i];

    // Deceleration

    size_t offset = 0;

    while (i >= offset) {
      ThunderAutoOutputTrajectoryPoint& offsetPoint = output.points[i - offset];
      units::meter_t offsetDistance = units::math::abs(point.distance - offsetPoint.distance);
      units::meters_per_second_t velocity = units::math::sqrt(
          units::math::pow<2>(maxVelocities[i]) + 2.0 * settings.maxLinearAcceleration * offsetDistance);
      if (offsetPoint.linearVelocity <= velocity)
        break;

      offsetPoint.linearVelocity = velocity;
      offset++;
    }

    // Acceleration

    offset = 1;

    while ((i + offset) < numPoints) {
      ThunderAutoOutputTrajectoryPoint& offsetPoint = output.points[i + offset];
      units::meter_t offsetDistance = units::math::abs(point.distance - offsetPoint.distance);
      units::meters_per_second_t velocity = units::math::sqrt(
          units::math::pow<2>(maxVelocities[i]) + 2.0 * settings.maxLinearAcceleration * offsetDistance);
      if (offsetPoint.linearVelocity <= velocity)
        break;

      offsetPoint.linearVelocity = velocity;
      offset++;
    }
  }
}

/**
 * Calculates the time for each point in the trajectory based on the linear velocities. This function
 * completes in O(n) time, where n is the number of points in the output trajectory.
 *
 * @param settings The trajectory skeleton settings.
 * @param output The output trajectory to update with times.
 */
static void CalculateTimes(const ThunderAutoTrajectorySkeletonSettings& settings,
                           ThunderAutoOutputTrajectory& output) {
  const size_t numPoints = output.points.size();

  units::meter_t lastDistance = 0.0_m;
  units::meters_per_second_t lastLinearVelocity = 0.0_mps;
  units::second_t lastTime = 0.0_s;

  for (size_t i = 0; i < numPoints; ++i) {
    ThunderAutoOutputTrajectoryPoint& point = output.points[i];

    units::meter_t currentDistance = point.distance;
    units::meters_per_second_t currentLinearVelocity = point.linearVelocity;

    if (currentLinearVelocity == 0.0_mps && lastLinearVelocity == 0.0_mps) {
      point.time = lastTime;
    } else {
      units::meter_t distanceChange = currentDistance - lastDistance;
      ThunderLibCoreAssert(distanceChange > 0.0_m);

      units::second_t deltaTime;
      if (currentLinearVelocity != lastLinearVelocity) {
        units::meters_per_second_squared_t a =
            (units::math::pow<2>(currentLinearVelocity) - units::math::pow<2>(lastLinearVelocity)) /
            (2.0 * distanceChange);

        deltaTime = (currentLinearVelocity - lastLinearVelocity) / a;
      } else if (currentLinearVelocity > 0.0_mps) {
        deltaTime = distanceChange / lastLinearVelocity;
      }

      ThunderLibCoreAssert(deltaTime > 0_s,
                           "Delta time is {} s, currentVelocity: {}, lastVelocity: {}, distanceChange: {}",
                           deltaTime.value(), currentLinearVelocity.value(), lastLinearVelocity.value(),
                           distanceChange.value());

      point.time = lastTime + deltaTime;
    }

    lastTime = point.time;
    lastDistance = currentDistance;
    lastLinearVelocity = currentLinearVelocity;
  }

  output.totalTime = lastTime;
}

/**
 * Calculates the angular velocities and rotations for each point in the trajectory based on the rotation
 * targets and the segments. This function completes in O(n) time, where n is the number of points in the
 * output trajectory.
 *
 * @param settings The trajectory skeleton settings.
 * @param rotations The sorted list of positioned rotation targets in the trajectory.
 * @param segments The segments of the trajectory, used to map each rotation target to its corresponding
 *                 output point index.
 * @param output The output trajectory to update with angular velocities.
 */
static void CalculateAngularVelocitiesAndRotations(
    const ThunderAutoTrajectorySkeletonSettings& settings,
    const std::map<ThunderAutoTrajectoryPosition, CanonicalAngle>& rotations,
    ThunderAutoOutputTrajectory& output) {
  ThunderLibCoreAssert(rotations.size() >= 2);

  auto startRotationIt = rotations.cbegin();
  auto endRotationIt = std::next(startRotationIt);

  for (; endRotationIt != rotations.cend(); ++startRotationIt, ++endRotationIt) {
    const auto& [startPosition, startRotation] = *startRotationIt;
    const auto& [endPosition, endRotation] = *endRotationIt;
    ThunderLibCoreAssert(startPosition < endPosition);

    size_t startOutputIndex;
    {
      ThunderLibCoreAssert(startPosition.segmentIndex() < output.segments.size());

      ThunderAutoOutputTrajectorySegment segment = output.segments[startPosition.segmentIndex()];
      size_t segmentNumPoints = (segment.endIndex - segment.startIndex) + 1;
      startOutputIndex =
          segment.startIndex + static_cast<size_t>(startPosition.positionInSegment() * segmentNumPoints);
    }

    size_t endOutputIndex;
    {
      if (endPosition.segmentIndex() < output.segments.size()) {
        ThunderAutoOutputTrajectorySegment segment = output.segments[endPosition.segmentIndex()];
        size_t segmentNumPoints = (segment.endIndex - segment.startIndex) + 1;
        endOutputIndex =
            segment.startIndex + static_cast<size_t>(endPosition.positionInSegment() * segmentNumPoints);
      } else {
        endOutputIndex = output.segments.back().endIndex;
      }
    }

    /**
     * It's possible for the startOutputIndex to be equal to the endOutputIndex if the build sample setting is
     * low enough and/or the rotations are placed very close together. This is obviously not ideal, so the
     * user should avoid constructing trajectories like this.
     */
    ThunderLibCoreAssert(startOutputIndex <= endOutputIndex);

    units::second_t timeTotal = output.points[endOutputIndex].time - output.points[startOutputIndex].time;
    ThunderLibCoreAssert(timeTotal >= 0.0_s, "Time from point {} to point {} is {} s, which is less than 0",
                         startOutputIndex, endOutputIndex, timeTotal.value());

    units::radian_t distanceTotal = (endRotation - startRotation).radians();
    units::radian_t distanceTotalMagnitude = units::math::abs(distanceTotal);

    units::second_t timeAccel, timeCruise;
    units::radian_t distanceAccel, distanceCruise;

    units::radians_per_second_squared_t angularAccelMagnitude = settings.maxAngularAcceleration;
    units::radians_per_second_squared_t angularAccel =
        units::math::copysign(angularAccelMagnitude, distanceTotal);

    units::radians_per_second_t peakAngularVelocity, peakAngularVelocityMagnitude;

    // Check triangular profile.
    units::radians_per_second_t triangularPeakVelocity = (2.0 * distanceTotal) / timeTotal;
    units::radians_per_second_t triangularPeakVelocityMagnitude = units::math::abs(triangularPeakVelocity);

    units::radians_per_second_squared_t triangularAccel =
        (4.0 * distanceTotal) / units::math::pow<2>(timeTotal);

    const bool isTriangularPeakVelocityGood =
        (triangularPeakVelocityMagnitude <= settings.maxAngularVelocity);
    const bool isTriangularAccelGood = (triangularAccel <= settings.maxAngularAcceleration);

    // Check if can use the triangular profile.
    if (isTriangularPeakVelocityGood && isTriangularAccelGood) {
      peakAngularVelocity = triangularPeakVelocity;
      peakAngularVelocityMagnitude = triangularPeakVelocityMagnitude;
      angularAccel = triangularAccel;
      angularAccelMagnitude = units::math::abs(angularAccel);

      timeAccel = timeTotal / 2.0;
      timeCruise = 0_s;

      distanceAccel = distanceTotal / 2.0;
      distanceCruise = 0_rad;

    } else {  // Solve for the trapezoidal profile.

      // Quadratic formula:

      auto discriminant = units::math::pow<2>(timeTotal * angularAccelMagnitude) -
                          4.0 * angularAccelMagnitude * distanceTotalMagnitude;

      /**
       * If v_tri > v_max, then a_tri must be less than a_max to for a trapezoid profile to be possible.
       *
       * We previously capped the linear velocity during the rotation period to make the rotation possible,
       * however there may be rare situations where a_tri slightly exceeds a_max due to limited curve
       * resolution and how times are calculated by CalculateTimes(). To keep the discriminant >= 0, use a_tri
       * and hope that nobody notices.
       */
      if (discriminant.value() < 0.0) {  // !isTriangularAccelGood
        ThunderLibCoreAssert(
            units::math::abs(triangularAccel - angularAccel).value() < 1e-9,
            "Angular velocity profile impossible: triangularAccel: {}, angularAccel: {}, diff: {}",
            triangularAccel.value(), angularAccel.value(), (triangularAccel - angularAccel).value());

        discriminant = decltype(discriminant)(0.0);
        angularAccel = triangularAccel;
        angularAccelMagnitude = units::math::abs(triangularAccel);
      }

      auto ta = timeTotal * angularAccelMagnitude;
      auto sqrtDisc = units::math::sqrt(discriminant);
      if (ta > sqrtDisc) {
        peakAngularVelocityMagnitude = (ta - sqrtDisc) / 2.0;
      } else {
        peakAngularVelocityMagnitude = (ta + sqrtDisc) / 2.0;
      }
      peakAngularVelocity = units::math::copysign(peakAngularVelocityMagnitude, distanceTotal);

      timeAccel = peakAngularVelocityMagnitude / angularAccelMagnitude;
      timeCruise = timeTotal - 2.0 * timeAccel;

      distanceAccel = 0.5 * angularAccel * units::math::pow<2>(timeAccel);
      distanceCruise = distanceTotal - 2.0 * distanceAccel;
    }

    units::second_t startTime = output.points[startOutputIndex].time;
    for (size_t i = startOutputIndex; i <= endOutputIndex; i++) {
      ThunderAutoOutputTrajectoryPoint& point = output.points[i];

      units::second_t time = point.time - startTime;

      if (time < timeAccel) {
        // Accelerating
        point.rotation = startRotation.rotateBy(0.5 * angularAccel * units::math::pow<2>(time));
        point.angularVelocity = angularAccel * time;

      } else if (time < timeAccel + timeCruise) {
        time -= timeAccel;
        point.rotation = startRotation.rotateBy(distanceAccel + peakAngularVelocity * time);
        point.angularVelocity = peakAngularVelocity;

      } else if (time < timeTotal) {
        // Decelerating
        time -= timeAccel + timeCruise;
        point.rotation =
            startRotation.rotateBy(distanceAccel + distanceCruise + (peakAngularVelocity * time) -
                                   (0.5 * angularAccel * units::math::pow<2>(time)));
        point.angularVelocity = peakAngularVelocity - (angularAccel * time);

      } else {
        point.rotation = endRotation;
        point.angularVelocity = 0.0_rad_per_s;
      }
    }
  }
}

/**
 * Calculates the headings and chassis speeds for each point in the trajectory. This function completes in
 * O(n) time, where n is the number of points in the output trajectory.
 *
 * @param output The output trajectory to update with headings and chassis speeds.
 */
static void CalculateHeadingsAndChassisSpeeds(ThunderAutoOutputTrajectory& output) {
  auto currentPointIt = output.points.begin();
  auto nextPointIt = std::next(currentPointIt);

  for (; currentPointIt != output.points.end(); ++currentPointIt, ++nextPointIt) {
    if (nextPointIt == output.points.end()) {
      currentPointIt->heading = std::prev(currentPointIt)->heading;
    } else {
      // Calculate the heading based on the next point's position.
      Displacement2d displacement = nextPointIt->position - currentPointIt->position;
      currentPointIt->heading = displacement.angle();
    }

    frc::ChassisSpeeds chassisSpeeds{
        .vx = currentPointIt->linearVelocity * currentPointIt->heading.cos(),
        .vy = currentPointIt->linearVelocity * currentPointIt->heading.sin(),
        .omega = currentPointIt->angularVelocity,
    };
    currentPointIt->chassisSpeeds = chassisSpeeds;
  }
}

/**
 * Calculates the centripetal acceleration for each point in the trajectory. This function completes in O(n)
 * time, where n is the number of points in the output trajectory.
 *
 * @param output The output trajectory to update with centripetal accelerations.
 */
static void CalculateCentripetalAccelerations(ThunderAutoOutputTrajectory& output) {
  for (auto& point : output.points) {
    // curvature_t has a weird definition... it's supposed to just be 1/radius.
    units::unit_t<units::inverse<units::meters>> curvature{point.curvature.value()};

    point.centripetalAcceleration = units::math::pow<2>(point.linearVelocity) * curvature;
  }
}

/**
 * Returns a sorted list of all actions in the given trajectory skeleton.
 *
 * @param skeleton The trajectory skeleton to extract actions from.
 *
 * @return A list of all the positioned actions in the trajectory.
 */
static std::multimap<ThunderAutoTrajectoryPosition, ThunderAutoTrajectoryAction> GetAllActions(
    const ThunderAutoTrajectorySkeleton& skeleton) {
  std::multimap<ThunderAutoTrajectoryPosition, ThunderAutoTrajectoryAction> actions = skeleton.actions();

  if (skeleton.hasStartAction()) {
    actions.emplace(ThunderAutoTrajectoryPosition(0.0), skeleton.startAction());
  }

  if (skeleton.points().size() > 2) {
    size_t waypointIndex = 1;

    const auto startIt = std::next(skeleton.points().begin());
    const auto endIt = std::prev(skeleton.points().end());

    for (auto waypointIt = startIt; waypointIt != endIt; ++waypointIt) {
      const ThunderAutoTrajectorySkeletonWaypoint& waypoint = *waypointIt;
      if (waypoint.isStopped() && waypoint.hasStopAction()) {
        double position = static_cast<double>(waypointIndex);
        const std::string& stopActionName = waypoint.stopAction();
        actions.emplace(ThunderAutoTrajectoryPosition(position), stopActionName);
      }
      waypointIndex++;
    }
  }

  if (skeleton.hasEndAction()) {
    actions.emplace(ThunderAutoTrajectoryPosition(static_cast<double>(skeleton.points().size() - 1)), skeleton.endAction());
  }

  return actions;
}

/**
 * Fills the actions in the output trajectory based on the actions specified in the skeleton. This function
 * completes in O(n) time, where n is the number of actions in the skeleton.
 *
 * @param actions The sorted list of positioned actions in the trajectory.
 * @param segments The segments of the trajectory, used to map each action to its corresponding output point
 * index.
 * @param output The output trajectory to update with actions.
 */
static void FillActions(
    const ThunderAutoTrajectorySkeleton& skeleton,
    const std::multimap<ThunderAutoTrajectoryPosition, ThunderAutoTrajectoryAction>& actions,
    ThunderAutoOutputTrajectory& output) {
  for (auto actionIt = actions.cbegin(); actionIt != actions.cend(); ++actionIt) {
    const auto& [position, action] = *actionIt;

    size_t outputIndex;
    if (position.segmentIndex() < output.segments.size()) {
      ThunderAutoOutputTrajectorySegment segment = output.segments[position.segmentIndex()];
      size_t segmentNumPoints = (segment.endIndex - segment.startIndex) + 1;
      outputIndex = segment.startIndex + static_cast<size_t>(position.positionInSegment() * segmentNumPoints);
    } else {
      outputIndex = output.segments.back().endIndex;
    }

    ThunderAutoOutputTrajectoryPoint& point = output.points[outputIndex];
    point.actions.push_back(action.action);
  }

  output.startAction = skeleton.startAction();
  output.endAction = skeleton.endAction();
  output.stopActions.clear();

  size_t waypointIndex = 0;
  for (auto waypointIt = skeleton.begin(); waypointIt != skeleton.end(); ++waypointIt, ++waypointIndex) {
    const ThunderAutoTrajectorySkeletonWaypoint& waypoint = *waypointIt;
    if (!waypoint.isStopped())
      continue;

    double position = static_cast<double>(waypointIndex);

    size_t outputPointIndex = output.trajectoryPositionToPointIndex(position);
    output.stopActions.emplace(output.points[outputPointIndex].time, waypoint.stopAction());
  }

  for (const auto& [position, action] : skeleton.actions()) {
    size_t outputPointIndex = output.trajectoryPositionToPointIndex(position);
    output.actions.emplace(output.points[outputPointIndex].time, action.action);
  }
}

std::unique_ptr<ThunderAutoOutputTrajectory> BuildThunderAutoOutputTrajectory(
    const ThunderAutoTrajectorySkeleton& skeleton,
    const ThunderAutoOutputTrajectorySettings& settings) {
  std::unique_ptr<ThunderAutoOutputTrajectory> output = std::make_unique<ThunderAutoOutputTrajectory>();
  output->buildTimestamp = units::second_t(wpi::GetSystemTime());

  const std::list<ThunderAutoTrajectorySkeletonWaypoint>& waypoints = skeleton.points();
  if (waypoints.size() < 2) {
    throw InvalidArgumentError::Construct("Cannot build trajectory with fewer than 2 waypoints");
  }

  /*
  ThunderLibCoreLogger::Info("Building trajectory (length samples: {}, samples per meter: {})",
                         settings.lengthSamples, settings.samplesPerMeter);
  */

  const auto lastIt = std::prev(waypoints.end());

  for (auto it = waypoints.begin(); it != lastIt; ++it) {
    const ThunderAutoTrajectorySkeletonWaypoint& segmentStartPoint = *it;
    const ThunderAutoTrajectorySkeletonWaypoint& segmentEndPoint = *std::next(it);

    EquationFunc f = BuildBezierCurveEquation(
        segmentStartPoint.position(), segmentStartPoint.headingControlPoints().outgoingPoint(),
        segmentEndPoint.position(), segmentEndPoint.headingControlPoints().incomingPoint());

    SampleSegmentPoints(f, settings.lengthSamples, *output);

    CalculateSegmentCurvatures(output->segments.back());
  }

  ResamplePoints(skeleton.settings(), settings.samplesPerMeter, *output);

  std::vector<units::meters_per_second_t> maxVelocities(output->points.size(),
                                                        skeleton.settings().maxLinearVelocity);

  AddWaypointMaxVelocityOverrideConstraints(skeleton.points(), output->segments, maxVelocities);

  AddCentripetalAccelerationMaxVelocityConstraints(skeleton.settings(), *output, maxVelocities);

  std::map<ThunderAutoTrajectoryPosition, CanonicalAngle> rotations = GetAllRotationTargets(skeleton);

  AddRotationMaxVelocityConstraints(skeleton.settings(), rotations, *output, maxVelocities);

  CalculateLinearVelocities(skeleton.settings(), maxVelocities, *output);
  CalculateTimes(skeleton.settings(), *output);
  CalculateAngularVelocitiesAndRotations(skeleton.settings(), rotations, *output);
  CalculateHeadingsAndChassisSpeeds(*output);
  CalculateCentripetalAccelerations(*output);

  std::multimap<ThunderAutoTrajectoryPosition, ThunderAutoTrajectoryAction> actions = GetAllActions(skeleton);
  FillActions(skeleton, actions, *output);

  /*
  ThunderLibCoreLogger::Info("Built trajectory with {} points, total distance: {:.2f} m, total time: {:.2f}
  s", output->points.size(), output->totalDistance.value(), output->totalTime.value());
  */

  return output;
}

// Same as BuildThunderAutoOutputTrajectory() but only position stuff.
std::unique_ptr<ThunderAutoPartialOutputTrajectory> BuildThunderAutoPartialOutputTrajectory(
    const ThunderAutoTrajectorySkeleton& skeleton,
    const ThunderAutoOutputTrajectorySettings& settings) {
  std::unique_ptr<ThunderAutoPartialOutputTrajectory> output =
      std::make_unique<ThunderAutoPartialOutputTrajectory>();
  output->buildTimestamp = units::second_t(wpi::GetSystemTime());

  const std::list<ThunderAutoTrajectorySkeletonWaypoint>& waypoints = skeleton.points();
  if (waypoints.size() < 2) {
    throw InvalidArgumentError::Construct("Cannot build trajectory with fewer than 2 waypoints");
  }

  /*
  ThunderLibCoreLogger::Info("Building partial trajectory (length samples: {}, samples per meter: {})",
                         settings.lengthSamples, settings.samplesPerMeter);
  */

  const auto lastIt = std::prev(waypoints.end());

  for (auto it = waypoints.begin(); it != lastIt; ++it) {
    const ThunderAutoTrajectorySkeletonWaypoint& segmentStartPoint = *it;
    const ThunderAutoTrajectorySkeletonWaypoint& segmentEndPoint = *std::next(it);

    EquationFunc f = BuildBezierCurveEquation(
        segmentStartPoint.position(), segmentStartPoint.headingControlPoints().outgoingPoint(),
        segmentEndPoint.position(), segmentEndPoint.headingControlPoints().incomingPoint());

    SampleSegmentPoints(f, settings.lengthSamples, *output);
  }

  ResamplePoints(settings.samplesPerMeter, *output);

  /*
  ThunderLibCoreLogger::Info("Built partial trajectory with {} points, total distance: {:.2f} m",
                         output->points.size(), output->totalDistance.value());
  */

  return output;
}

void from_json(const wpi::json& json, ThunderAutoCSVExportProperties& properties) {
  json.at("time").get_to(properties.time);
  json.at("position").get_to(properties.position);
  json.at("linear_velocity").get_to(properties.linearVelocity);
  json.at("component_velocities").get_to(properties.componentVelocities);
  json.at("heading").get_to(properties.heading);
  json.at("rotation").get_to(properties.rotation);
  json.at("angular_velocity").get_to(properties.angularVelocity);
  json.at("actions_bit_field").get_to(properties.actionsBitField);
  json.at("distance").get_to(properties.distance);
  json.at("curvature").get_to(properties.curvature);
  json.at("centripetal_acceleration").get_to(properties.centripetalAcceleration);
}

void to_json(wpi::json& json, const ThunderAutoCSVExportProperties& properties) noexcept {
  json = wpi::json{
      {"time", properties.time},
      {"position", properties.position},
      {"linear_velocity", properties.linearVelocity},
      {"component_velocities", properties.componentVelocities},
      {"heading", properties.heading},
      {"rotation", properties.rotation},
      {"angular_velocity", properties.angularVelocity},
      {"actions_bit_field", properties.actionsBitField},
      {"distance", properties.distance},
      {"curvature", properties.curvature},
      {"centripetal_acceleration", properties.centripetalAcceleration},
  };
}

struct CSVExportRowContext {
  bool previousValue = false;
};

// Writes a value to a CSV file.
template <typename T>
static void CSVExportValue(std::ofstream& ofs, const T& value, CSVExportRowContext& context) {
  if (context.previousValue) {
    ofs << ",";
  }
  ofs << value;
  context.previousValue = true;
}

// Writes a header row to a CSV file based on the given export properties.
static void CSVExportHeader(std::ofstream& ofs, const ThunderAutoCSVExportProperties& props) {
  CSVExportRowContext context;

  if (props.time) {
    CSVExportValue(ofs, "time", context);
  }
  if (props.position) {
    CSVExportValue(ofs, "x_pos,y_pos", context);
  }
  if (props.linearVelocity) {
    CSVExportValue(ofs, "velocity", context);
  }
  if (props.componentVelocities) {
    CSVExportValue(ofs, "vx,vy", context);
  }
  if (props.heading) {
    CSVExportValue(ofs, "heading", context);
  }
  if (props.rotation) {
    CSVExportValue(ofs, "rotation", context);
  }
  if (props.angularVelocity) {
    CSVExportValue(ofs, "angular_velocity", context);
  }
  if (props.actionsBitField) {
    CSVExportValue(ofs, "action", context);
  }
  if (props.distance) {
    CSVExportValue(ofs, "distance", context);
  }
  if (props.curvature) {
    CSVExportValue(ofs, "curvature", context);
  }
  if (props.centripetalAcceleration) {
    CSVExportValue(ofs, "centripetal_accel", context);
  }
  ofs << "\n";
}

static std::unordered_map<std::string, uint64_t> BuildActionBitLookupMap(
    std::span<const std::string> orderedActions) {
  std::unordered_map<std::string, uint64_t> actionBitLookup;
  for (size_t i = 0; i < orderedActions.size(); ++i) {
    actionBitLookup[orderedActions[i]] = 1ULL << i;
  }
  return actionBitLookup;
}

static uint64_t ActionsToBitField(const std::vector<std::string>& actions,
                                  const std::unordered_map<std::string, uint64_t>& actionBitLookup) {
  uint64_t bitField = 0;
  for (const std::string& action : actions) {
    auto it = actionBitLookup.find(action);
    ThunderLibCoreAssert(it != actionBitLookup.end(), "Action '{}' not found in action bit lookup map",
                         action);
    if (it != actionBitLookup.end()) {
      bitField |= it->second;
    }
  }
  return bitField;
}

static void CSVExportTrajectoryDataRow(std::ofstream& ofs,
                                       const ThunderAutoOutputTrajectoryPoint& point,
                                       const std::unordered_map<std::string, uint64_t>& actionBitLookup,
                                       const ThunderAutoCSVExportProperties& props) {
  CSVExportRowContext context;

  if (props.time) {
    CSVExportValue(ofs, point.time(), context);
  }
  if (props.position) {
    CSVExportValue(ofs, point.position.x(), context);
    CSVExportValue(ofs, point.position.y(), context);
  }
  if (props.linearVelocity) {
    CSVExportValue(ofs, point.linearVelocity(), context);
  }
  if (props.componentVelocities) {
    CSVExportValue(ofs, point.chassisSpeeds.vx(), context);
    CSVExportValue(ofs, point.chassisSpeeds.vy(), context);
  }
  if (props.heading) {
    CSVExportValue(ofs, point.heading.radians()(), context);
  }
  if (props.rotation) {
    CSVExportValue(ofs, point.rotation.radians()(), context);
  }
  if (props.angularVelocity) {
    CSVExportValue(ofs, point.angularVelocity(), context);
  }
  if (props.actionsBitField) {
    uint64_t actions = ActionsToBitField(point.actions, actionBitLookup);
    CSVExportValue(ofs, actions, context);
  }
  if (props.distance) {
    CSVExportValue(ofs, point.distance(), context);
  }
  if (props.curvature) {
    CSVExportValue(ofs, point.curvature(), context);
  }
  if (props.centripetalAcceleration) {
    CSVExportValue(ofs, point.centripetalAcceleration(), context);
  }
  ofs << "\n";
}

void CSVExportThunderAutoOutputTrajectory(const ThunderAutoOutputTrajectory& output,
                                          std::span<const std::string> orderedActions,
                                          const std::filesystem::path& exportPath,
                                          const ThunderAutoCSVExportProperties& properties) {
  ThunderLibCoreLogger::Info("Exporting trajectory to CSV: {}", exportPath.string());

  std::ofstream ofs;
  ofs.exceptions(std::ofstream::failbit | std::ofstream::badbit);
  ofs.precision(5);
  ofs.setf(std::ios::fixed, std::ios::floatfield);

  try {
    ofs.open(exportPath, std::ios::out | std::ios::trunc);

    if (properties.includeHeader) {
      CSVExportHeader(ofs, properties);
    }
    std::unordered_map<std::string, uint64_t> actionBitLookup = BuildActionBitLookupMap(orderedActions);
    for (const ThunderAutoOutputTrajectoryPoint& point : output.points) {
      CSVExportTrajectoryDataRow(ofs, point, actionBitLookup, properties);
    }

  } catch (const std::ofstream::failure& e) {
    throw RuntimeError::Construct("CSVExportThunderAutoOutputTrajectory: File operation failed: {}",
                                  e.what());
  }

  ThunderLibCoreLogger::Info("Export complete");
}

void BuildAndCSVExportThunderAutoOutputTrajectory(const ThunderAutoTrajectorySkeleton& trajectorySkeleton,
                                                  const ThunderAutoOutputTrajectorySettings& buildSettings,
                                                  std::span<const std::string> orderedActions,
                                                  const std::filesystem::path& exportPath,
                                                  const ThunderAutoCSVExportProperties& exportProperties) {
  std::unique_ptr<ThunderAutoOutputTrajectory> output =
      BuildThunderAutoOutputTrajectory(trajectorySkeleton, buildSettings);
  ThunderLibCoreAssert(
      output);  // BuildThunderAutoOutputTrajectory should have thrown an exception if it failed.
  CSVExportThunderAutoOutputTrajectory(*output, orderedActions, exportPath, exportProperties);
}

}  // namespace thunder::core

// hi chris!!
