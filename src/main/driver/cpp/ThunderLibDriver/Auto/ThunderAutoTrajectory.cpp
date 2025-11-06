#include <ThunderLibDriver/Auto/ThunderAutoTrajectory.hpp>
#include <ThunderLibCore/Math.hpp>

namespace thunder::driver {

ThunderAutoTrajectory::ThunderAutoTrajectory(
    std::shared_ptr<core::ThunderAutoOutputTrajectory> trajectory) noexcept
    : m_trajectory(std::move(trajectory)) {}

ThunderAutoTrajectory::~ThunderAutoTrajectory() = default;

bool ThunderAutoTrajectory::isValid() const noexcept {
  return m_trajectory != nullptr;
}

ThunderAutoTrajectoryState ThunderAutoTrajectory::sample(units::second_t time) const noexcept {
  if (!isValid() || m_trajectory->points.empty() || time < 0_s) {
    return ThunderAutoTrajectoryState{};
  }

  const auto& points = m_trajectory->points;

  // First element with time >= requested time
  const auto lowerBound = std::lower_bound(
      points.begin(), points.end(), time,
      [](const core::ThunderAutoOutputTrajectoryPoint& point, units::second_t t) { return point.time < t; });

  if (lowerBound == points.begin()) {
    return getInitialState();
  }

  const core::ThunderAutoOutputTrajectoryPoint& prevPoint = *(lowerBound - 1);
  const core::ThunderAutoOutputTrajectoryPoint& nextPoint = *lowerBound;

  const ThunderAutoTrajectoryState prev = toState(prevPoint);
  const ThunderAutoTrajectoryState next = toState(nextPoint);

  // Close enough.
  if (next.time - prev.time < 10_ms) {
    return next;
  }

  // Linear interpolation.
  const double t = (time - prev.time).value() / (next.time - prev.time).value();

  frc::Pose2d pose = prev.pose.Exp(prev.pose.Log(next.pose) * t);

  frc::ChassisSpeeds chassisSpeeds;
  chassisSpeeds.vx = core::Lerp(prev.chassisSpeeds.vx, next.chassisSpeeds.vx, t);
  chassisSpeeds.vy = core::Lerp(prev.chassisSpeeds.vy, next.chassisSpeeds.vy, t);
  chassisSpeeds.omega = core::Lerp(prev.chassisSpeeds.omega, next.chassisSpeeds.omega, t);

  units::meters_per_second_t linearVelocity = core::Lerp(prev.linearVelocity, next.linearVelocity, t);

  const core::CanonicalAngle prevHeading(prev.heading);
  const core::CanonicalAngle nextHeading(next.heading);
  core::CanonicalAngle heading = core::Lerp(prevHeading, nextHeading, t);

  ThunderAutoTrajectoryState state = {.time = time,
                                      .pose = pose,
                                      .chassisSpeeds = chassisSpeeds,
                                      .linearVelocity = linearVelocity,
                                      .heading = heading};

  return state;
}

units::second_t ThunderAutoTrajectory::getDuration() const noexcept {
  if (!isValid()) {
    return 0_s;
  }

  return m_trajectory->totalTime;
}

ThunderAutoTrajectoryState ThunderAutoTrajectory::getInitialState() const noexcept {
  if (!isValid() || m_trajectory->points.empty()) {
    return ThunderAutoTrajectoryState{};
  }

  ThunderAutoTrajectoryState state = getState(0);
  return state;
}

ThunderAutoTrajectoryState ThunderAutoTrajectory::getFinalState() const noexcept {
  if (!isValid() || m_trajectory->points.empty()) {
    return ThunderAutoTrajectoryState{};
  }

  size_t index = m_trajectory->points.size() - 1;
  ThunderAutoTrajectoryState state = getState(index);
  return state;
}

const std::string& ThunderAutoTrajectory::getStartAction() const noexcept {
  if (!isValid()) {
    static std::string emptyString;
    return emptyString;
  }
  return m_trajectory->startAction;
}

const std::string& ThunderAutoTrajectory::getEndAction() const noexcept {
  if (!isValid()) {
    static std::string emptyString;
    return emptyString;
  }
  return m_trajectory->endAction;
}

const std::map<units::second_t, std::string>& ThunderAutoTrajectory::getStopActions()
    const noexcept {
  if (!isValid()) {
    static const std::map<units::second_t, std::string> emptyMap;
    return emptyMap;
  }
  return m_trajectory->stopActions;
}

const std::multimap<units::second_t, std::string>& ThunderAutoTrajectory::getActions() const noexcept {
  if (!isValid()) {
    static const std::multimap<units::second_t, std::string> emptyMultimap;
    return emptyMultimap;
  }
  return m_trajectory->actions;
}

ThunderAutoTrajectoryState ThunderAutoTrajectory::getState(size_t index) const noexcept {
  if (!isValid()) {
    return ThunderAutoTrajectoryState{};
  }

  core::ThunderAutoOutputTrajectoryPoint point = m_trajectory->points.at(index);
  ThunderAutoTrajectoryState state = toState(point);
  return state;
}

ThunderAutoTrajectoryState ThunderAutoTrajectory::toState(
    const core::ThunderAutoOutputTrajectoryPoint& point) const noexcept {
  ThunderAutoTrajectoryState state = {.time = point.time,
                                      .pose = frc::Pose2d(point.position.x, point.position.y, point.rotation),
                                      .chassisSpeeds = point.chassisSpeeds,
                                      .linearVelocity = point.linearVelocity,
                                      .heading = point.heading};
  return state;
}

}  // namespace thunder::driver
