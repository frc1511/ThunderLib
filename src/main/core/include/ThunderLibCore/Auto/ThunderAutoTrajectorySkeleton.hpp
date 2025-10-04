#pragma once

#include <ThunderLibCore/Types.hpp>
#include <ThunderLibCore/Error.hpp>

#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <wpi/json.h>
#include <gcem.hpp>

#include <initializer_list>
#include <algorithm>
#include <list>
#include <optional>
#include <string>
#include <string_view>
#include <cmath>
#include <map>
#include <unordered_set>

namespace thunder::core {

/**
 * Represents a waypoint in a ThunderAuto trajectory skeleton.
 */
class ThunderAutoTrajectorySkeletonWaypoint {
 public:
  class HeadingControlPoints;
  class HeadingAngles;
  class HeadingWeights;

  /**
   * Represents the incoming and outgoing points controlling the heading and
   * weight of a trajectory waypoint. Used exclusively by the ThunderAuto
   * editor.
   */
  class HeadingControlPoints {
    Point2d m_incomingPoint;
    Point2d m_outgoingPoint;

   public:
    HeadingControlPoints() = default;

    HeadingControlPoints(Point2d incoming, Point2d outgoing)
        : m_incomingPoint(incoming), m_outgoingPoint(outgoing) {}

    HeadingControlPoints(const Point2d& centerPoint,
                         const HeadingAngles& angles,
                         const HeadingWeights& weights);

    bool operator==(const HeadingControlPoints& other) const = default;

    const Point2d& incomingPoint() const { return m_incomingPoint; }

    void setIncomingPoint(const Point2d& point) { m_incomingPoint = point; }

    void setIncomingPoint(const Point2d& centerPoint,
                          const CanonicalAngle& incomingAngle,
                          const double incomingWeight);

    const Point2d& outgoingPoint() const { return m_outgoingPoint; }

    void setOutgoingPoint(const Point2d& point) { m_outgoingPoint = point; }

    void setOutgoingPoint(const Point2d& centerPoint,
                          const CanonicalAngle& outgoingAngle,
                          const double outgoingWeight);
  };

  /**
   * Represents the incoming and outgoing headings of a trajectory waypoint.
   */
  class HeadingAngles {
    CanonicalAngle m_incomingAngle;
    CanonicalAngle m_outgoingAngle;

   public:
    HeadingAngles() : HeadingAngles(0_deg) {}

    HeadingAngles(CanonicalAngle incoming, CanonicalAngle outgoing)
        : m_incomingAngle(incoming), m_outgoingAngle(outgoing) {}

    explicit HeadingAngles(CanonicalAngle outgoing)
        : m_incomingAngle(outgoing.supplementary()), m_outgoingAngle(outgoing) {}

    HeadingAngles(const Point2d& centerPoint, const HeadingControlPoints& controlPoints);

    bool operator==(const HeadingAngles& other) const = default;

    const CanonicalAngle& incomingAngle() const { return m_incomingAngle; }

    void setIncomingAngle(const CanonicalAngle& angle, bool lockSupplementary = false);

    void setIncomingAngle(const Point2d& centerPoint,
                          const Point2d& incomingControlPoint,
                          bool lockSupplementary = false);

    const CanonicalAngle& outgoingAngle() const { return m_outgoingAngle; }

    void setOutgoingAngle(const CanonicalAngle& angle, bool lockSupplementary = false);

    void setOutgoingAngle(const Point2d& centerPoint,
                          const Point2d& outgoingControlPoint,
                          bool lockSupplementary = false);

    void setAngle(bool isOutgoing, const CanonicalAngle& angle, bool lockSupplementary = false);

    void setAngle(bool isOutgoing,
                  const Point2d& centerPoint,
                  const Point2d& controlPoint,
                  bool lockSupplementary = false);
  };

  /**
   * Represents the weights for the incoming and outgoing headings of a
   * trajectory waypoint. Hard to visualize outside of the ThunderAuto editor.
   */
  class HeadingWeights {
    double m_incomingWeight = 1.0;
    double m_outgoingWeight = 1.0;

   public:
    static constexpr double MIN_WEIGHT = 0.25;

    HeadingWeights() = default;
    HeadingWeights(double incoming, double outgoing);
    HeadingWeights(const Point2d& centerPoint, const HeadingControlPoints& controlPoints);

    bool operator==(const HeadingWeights& other) const = default;

    double incomingWeight() const { return m_incomingWeight; }
    double outgoingWeight() const { return m_outgoingWeight; }

    void setIncomingWeight(double weight);
    void setIncomingWeight(const Point2d& centerPoint, const Point2d& incomingControlPoint);

    void setOutgoingWeight(double weight);
    void setOutgoingWeight(const Point2d& centerPoint, const Point2d& outgoingControlPoint);

    void setWeight(bool isOutgoing, double weight);
    void setWeight(bool isOutgoing, const Point2d& centerPoint, const Point2d& controlPoint);
  };

 private:
  Point2d m_position;

  HeadingAngles m_headings;
  HeadingWeights m_headingWeights;

  std::optional<units::meters_per_second_t> m_maxVelocityOverride;
  CanonicalAngle m_stopRotation;
  std::unordered_set<std::string> m_stopActions;

  std::string m_link = "";

  bool m_editorLocked = false;

 public:
  ThunderAutoTrajectorySkeletonWaypoint() = default;

  ThunderAutoTrajectorySkeletonWaypoint(Point2d position,
                                        HeadingAngles headings,
                                        HeadingWeights headingWeights);

  ThunderAutoTrajectorySkeletonWaypoint(Point2d position,
                                        units::meters_per_second_t velocityOverride,
                                        HeadingAngles headings,
                                        HeadingWeights headingWeights);

  ThunderAutoTrajectorySkeletonWaypoint(Point2d position,
                                        CanonicalAngle outgoingHeading,
                                        HeadingWeights headingWeights);

  ThunderAutoTrajectorySkeletonWaypoint(Point2d position, HeadingControlPoints controlPoints);

  bool operator==(const ThunderAutoTrajectorySkeletonWaypoint& other) const = default;

  // Position

  Point2d position() const { return m_position; }
  void setPosition(const Point2d& position) { m_position = position; }

  void translate(units::meter_t dx, units::meter_t dy) { translate(Point2d{dx, dy}); }
  void translate(const Point2d& translation) { m_position += translation; }

  // Headings

  const HeadingAngles& headings() const { return m_headings; }

  void setHeadings(const HeadingAngles& headings);
  void setIncomingHeading(const CanonicalAngle& angle);
  void setOutgoingHeading(const CanonicalAngle& angle);
  void setHeading(bool isOutgoing, const CanonicalAngle& angle);

  // Heading Weights

  const HeadingWeights& headingWeights() const { return m_headingWeights; }

  void setHeadingWeights(const HeadingWeights& weights);
  void setIncomingHeadingWeight(double weight);
  void setOutgoingHeadingWeight(double weight);
  void setHeadingWeight(bool isOutgoing, double weight);

  // Control Points

  HeadingControlPoints headingControlPoints() const;

  void setHeadingControlPoints(const HeadingControlPoints& controlPoints);
  void setIncomingHeadingControlPoint(const Point2d& point);
  void setOutgoingHeadingControlPoint(const Point2d& point);

  // Velocity

  bool isStopped() const noexcept;

  void setStopped(bool stopped) noexcept;

  CanonicalAngle stopRotation() const;

  void setStopRotation(const CanonicalAngle& rotation);

  const std::unordered_set<std::string>& stopActions() const noexcept { return m_stopActions; }
  bool hasStopActions() const noexcept { return !m_stopActions.empty(); }
  void setAllStopActions(const std::unordered_set<std::string>& actions) noexcept { m_stopActions = actions; }
  void clearStopActions() noexcept { m_stopActions.clear(); }

  bool hasStopAction(const std::string& action) const noexcept;
  bool addStopAction(const std::string& action);
  bool removeStopAction(const std::string& action) noexcept;

  std::optional<units::meters_per_second_t> maxVelocityOverride() const noexcept {
    return m_maxVelocityOverride;
  }

  bool hasMaxVelocityOverride() const noexcept { return m_maxVelocityOverride.has_value(); }

  void setMaxVelocityOverride(units::meters_per_second_t velocity) noexcept;
  void resetMaxVelocityOverride() noexcept;

  // Link

  bool isLinked() const noexcept { return !m_link.empty(); }

  std::string_view linkName() const noexcept { return m_link; }

  void setLinkName(const std::string& link);
  void removeLink() noexcept;

  // Editor Stuff

  bool isEditorLocked() const noexcept { return m_editorLocked; }

  void setEditorLocked(bool locked) noexcept { m_editorLocked = locked; }
};

struct ThunderAutoTrajectorySkeletonSettings {
  units::meters_per_second_t maxLinearVelocity = 2.0_mps;
  units::meters_per_second_squared_t maxLinearAcceleration = 2_mps_sq;
  units::meters_per_second_squared_t maxCentripetalAcceleration = 2 * 9.81_mps_sq;
  units::radians_per_second_t maxAngularVelocity = 3.14_rad_per_s;
  units::radians_per_second_squared_t maxAngularAcceleration = 3.14_rad_per_s_sq;

  bool operator==(const ThunderAutoTrajectorySkeletonSettings& other) const = default;
};

struct ThunderAutoTrajectoryPosition {
  double position = 0.0;

 public:
  ThunderAutoTrajectoryPosition() = default;

  /*implicit*/ ThunderAutoTrajectoryPosition(double pos) : position(pos) {}

  /*implicit*/ operator double() const { return position; }

  size_t segmentIndex() const { return std::floor(position); }

  double positionInSegment() const { return position - segmentIndex(); }
};

template <typename T>
class ThunderAutoPositionedTrajectoryItemList {
  using ListType = std::multimap<ThunderAutoTrajectoryPosition, T>;
  ListType m_list;

 public:
  ThunderAutoPositionedTrajectoryItemList() = default;

  explicit ThunderAutoPositionedTrajectoryItemList(const ListType& list) : m_list(list) {}

  bool operator==(const ThunderAutoPositionedTrajectoryItemList<T>& other) const = default;

  operator const ListType&() const { return m_list; }

  using value_type = T;
  using iterator = ListType::iterator;
  using const_iterator = ListType::const_iterator;
  using reverse_iterator = ListType::reverse_iterator;
  using const_reverse_iterator = ListType::const_reverse_iterator;

  iterator begin() { return m_list.begin(); }
  iterator end() { return m_list.end(); }
  const_iterator begin() const { return m_list.begin(); }
  const_iterator end() const { return m_list.end(); }
  const_iterator cbegin() const { return m_list.cbegin(); }
  const_iterator cend() const { return m_list.cend(); }
  reverse_iterator rbegin() { return m_list.rbegin(); }
  reverse_iterator rend() { return m_list.rend(); }
  const_reverse_iterator rbegin() const { return m_list.rbegin(); }
  const_reverse_iterator rend() const { return m_list.rend(); }
  const_reverse_iterator crbegin() const { return m_list.rbegin(); }
  const_reverse_iterator crend() const { return m_list.rend(); }

  size_t size() const { return m_list.size(); }
  bool empty() const { return m_list.empty(); }

  iterator add(ThunderAutoTrajectoryPosition position, const T& item) {
    return m_list.emplace(position, item);
  }

  iterator move(ThunderAutoTrajectoryPosition previousPosition, ThunderAutoTrajectoryPosition newPosition) {
    typename ListType::node_type node = m_list.extract(previousPosition);
    if (node.empty()) {
      throw LogicError::Construct("No item exists at the specified previous position");
    }

    node.key() = newPosition;
    return m_list.insert(std::move(node));
  }

  iterator move(const_iterator previousPositionIt, ThunderAutoTrajectoryPosition newPosition) {
    if (previousPositionIt == m_list.end()) {
      throw InvalidArgumentError::Construct("Invalid iterator specified");
    }

    typename ListType::node_type node = m_list.extract(previousPositionIt);
    if (node.empty()) {
      throw LogicError::Construct("No item exists at the specified previous position");
    }

    node.key() = newPosition;
    return m_list.insert(std::move(node));
  }

  iterator remove(const_iterator positionIt) {
    if (positionIt == m_list.end()) {
      throw InvalidArgumentError::Construct("Invalid iterator specified");
    }
    return m_list.erase(positionIt);
  }

  void clear() { m_list.clear(); }
};

struct ThunderAutoTrajectoryRotation {
  CanonicalAngle angle;
  bool editorLocked = false;

  bool operator==(const ThunderAutoTrajectoryRotation& other) const = default;
};

struct ThunderAutoTrajectoryAction {
  std::string action;
  bool editorLocked = false;

  // std::optional<ThunderAutoTrajectoryPosition> zoneEndPosition;

  bool operator==(const ThunderAutoTrajectoryAction& other) const = default;
};

/**
 * Represents the outline of a ThunderAuto trajectory. Contains a list of
 * waypoints and properties that describe how the trajectory should be formed.
 */
class ThunderAutoTrajectorySkeleton {
  ThunderAutoTrajectorySkeletonSettings m_settings;
  std::list<ThunderAutoTrajectorySkeletonWaypoint> m_points;

  std::unordered_set<std::string> m_startActions;
  std::unordered_set<std::string> m_endActions;
  ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryAction> m_actions;

  CanonicalAngle m_startRotation;
  CanonicalAngle m_endRotation;

  ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryRotation> m_rotations;

 public:
  explicit ThunderAutoTrajectorySkeleton(ThunderAutoTrajectorySkeletonSettings settings = {})
      : m_settings(settings) {}

  explicit ThunderAutoTrajectorySkeleton(std::initializer_list<ThunderAutoTrajectorySkeletonWaypoint> points,
                                         ThunderAutoTrajectorySkeletonSettings settings = {})
      : m_settings(settings), m_points(points) {}

  explicit ThunderAutoTrajectorySkeleton(const wpi::json& json) : ThunderAutoTrajectorySkeleton() {
    fromJson(json);
  }

  bool operator==(const ThunderAutoTrajectorySkeleton& other) const = default;

  /**
   * Fill the trajectory skeleton from JSON.
   *
   * @param json The JSON object containing the trajectory skeleton.
   */
  void fromJson(const wpi::json& json);

  /**
   * Fill the trajectory skeleton from pre-2026 JSON format.
   *
   * @param json The JSON object containing the trajectory skeleton.
   */
  void fromJsonPre2026(const wpi::json& json,
                       std::span<std::string> actions,
                       std::span<std::string> waypointLinks);

  // Settings

  const ThunderAutoTrajectorySkeletonSettings& settings() const { return m_settings; }
  ThunderAutoTrajectorySkeletonSettings& settings() { return m_settings; }

  void setSettings(const ThunderAutoTrajectorySkeletonSettings& settings) { m_settings = settings; }

  // Points

  std::list<ThunderAutoTrajectorySkeletonWaypoint>& points() { return m_points; }

  const std::list<ThunderAutoTrajectorySkeletonWaypoint>& points() const { return m_points; }

  void setPoints(const std::list<ThunderAutoTrajectorySkeletonWaypoint>& points) { m_points = points; }

  size_t numPoints() const { return m_points.size(); }

  using value_type = ThunderAutoTrajectorySkeletonWaypoint;
  using iterator = decltype(m_points)::iterator;
  using const_iterator = decltype(m_points)::const_iterator;
  using reverse_iterator = decltype(m_points)::reverse_iterator;
  using const_reverse_iterator = decltype(m_points)::const_reverse_iterator;

  iterator begin() { return m_points.begin(); }
  iterator end() { return m_points.end(); }
  const_iterator begin() const { return m_points.begin(); }
  const_iterator end() const { return m_points.end(); }
  const_iterator cbegin() const { return m_points.cbegin(); }
  const_iterator cend() const { return m_points.cend(); }
  reverse_iterator rbegin() { return m_points.rbegin(); }
  reverse_iterator rend() { return m_points.rend(); }
  const_reverse_iterator rbegin() const { return m_points.rbegin(); }
  const_reverse_iterator rend() const { return m_points.rend(); }
  const_reverse_iterator crbegin() const { return m_points.rbegin(); }
  const_reverse_iterator crend() const { return m_points.rend(); }

  const ThunderAutoTrajectorySkeletonWaypoint& front() const { return m_points.front(); }
  ThunderAutoTrajectorySkeletonWaypoint& front() { return m_points.front(); }
  const ThunderAutoTrajectorySkeletonWaypoint& back() const { return m_points.back(); }
  ThunderAutoTrajectorySkeletonWaypoint& back() { return m_points.back(); }

  const_iterator insertPoint(const_iterator it, const ThunderAutoTrajectorySkeletonWaypoint& point);
  const_iterator insertPoint(size_t index, const ThunderAutoTrajectorySkeletonWaypoint& point);

  void prependPoint(const ThunderAutoTrajectorySkeletonWaypoint& point);
  void appendPoint(const ThunderAutoTrajectorySkeletonWaypoint& point);

  const_iterator removePoint(const_iterator it);
  const_iterator removePoint(size_t index);

  void clearPoints();

  ThunderAutoTrajectorySkeletonWaypoint& getPoint(size_t index);
  const ThunderAutoTrajectorySkeletonWaypoint& getPoint(size_t index) const;

  // Actions

  const std::unordered_set<std::string>& startActions() const noexcept { return m_startActions; }
  bool hasStartActions() const noexcept { return !m_startActions.empty(); }
  void setAllStartActions(const std::unordered_set<std::string>& actions) { m_startActions = actions; }
  void clearStartActions() { m_startActions.clear(); }

  bool hasStartAction(const std::string& action) const noexcept;
  bool addStartAction(const std::string& action);
  bool removeStartAction(const std::string& action);

  const std::unordered_set<std::string>& endActions() const noexcept { return m_endActions; }
  bool hasEndActions() const noexcept { return !m_endActions.empty(); }
  void setAllEndActions(const std::unordered_set<std::string>& actions) { m_endActions = actions; }
  void clearEndActions() { m_endActions.clear(); }

  bool hasEndAction(const std::string& action) const noexcept;
  bool addEndAction(const std::string& action);
  bool removeEndAction(const std::string& action);

  ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryAction>& actions() { return m_actions; }
  const ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryAction>& actions() const {
    return m_actions;
  }

  void setActions(const ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryAction>& actions) {
    m_actions = actions;
  }

  ThunderAutoTrajectoryAction& getAction(size_t index);
  const ThunderAutoTrajectoryAction& getAction(size_t index) const;

  // Rotations

  CanonicalAngle startRotation() const { return m_startRotation; }
  CanonicalAngle endRotation() const { return m_endRotation; }

  void setStartRotation(const CanonicalAngle& rotation) { m_startRotation = rotation; }
  void setEndRotation(const CanonicalAngle& rotation) { m_endRotation = rotation; }

  ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryRotation>& rotations() { return m_rotations; }
  const ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryRotation>& rotations() const {
    return m_rotations;
  }

  void setRotations(const ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryRotation>& rotations) {
    m_rotations = rotations;
  }

  ThunderAutoTrajectoryRotation& getRotation(size_t index);
  const ThunderAutoTrajectoryRotation& getRotation(size_t index) const;

  // Other stuff

  void reverseDirection();

  /**
   * Separate rotation targets in the trajectory. This function will attempt to make all rotation targets be
   * spaced at least minSeparationDistance. If this is not possible (in rare situations when there are too
   * many rotation targets and the trajectory is short), a smaller separation distance may be used instead.
   *
   * @param minSeparationDistance The minimum allowable distance in between rotation targets.
   * @param trajectoryPositionData
   */
  void separateRotations(units::meter_t minSeparationDistance,
                         const struct ThunderAutoPartialOutputTrajectory* trajectoryPositionData);

 private:
  /**
   * Remove any rotation targets and actions that are positioned past the last waypoint in the trajectory.
   */
  void purgeOutOfBoundsRotationsAndActions();

  void shiftRotationAndActionPositions(ThunderAutoTrajectoryPosition shiftStartPosition, double shiftAmount);
};

void to_json(wpi::json& json, const ThunderAutoTrajectorySkeleton& trajectory);
void from_json(const wpi::json& json, ThunderAutoTrajectorySkeleton& trajectory);

}  // namespace thunder::core
