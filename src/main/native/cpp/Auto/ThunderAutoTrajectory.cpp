#include <ThunderLib/Auto/ThunderAutoTrajectory.hpp>
#include <ThunderLibDriver/Auto/ThunderAutoTrajectory.hpp>

namespace thunder {

ThunderAutoTrajectory::ThunderAutoTrajectory(driver::ThunderAutoTrajectory* trajectory) noexcept
    : Trajectory(), m_handle(trajectory) {}

ThunderAutoTrajectory::~ThunderAutoTrajectory() {
  if (m_handle) {
    delete m_handle;
  }
}

bool ThunderAutoTrajectory::isValid() const noexcept {
  return m_handle != nullptr;
}

TrajectoryState ThunderAutoTrajectory::sample(units::second_t time) const noexcept {
  if (!m_handle) {
    return TrajectoryState{};
  }

  driver::ThunderAutoTrajectoryState driverState = m_handle->sample(time);
  TrajectoryState state = convertState(driverState);
  return state;
}

units::second_t ThunderAutoTrajectory::getDuration() const noexcept {
  if (!m_handle) {
    return 0_s;
  }

  return m_handle->getDuration();
}

TrajectoryState ThunderAutoTrajectory::getInitialState() const noexcept {
  if (!m_handle) {
    return TrajectoryState{};
  }

  driver::ThunderAutoTrajectoryState driverState = m_handle->getInitialState();
  TrajectoryState state = convertState(driverState);
  return state;
}

TrajectoryState ThunderAutoTrajectory::getFinalState() const noexcept {
  if (!m_handle) {
    return TrajectoryState{};
  }

  driver::ThunderAutoTrajectoryState driverState = m_handle->getFinalState();
  TrajectoryState state = convertState(driverState);
  return state;
}

const std::string& ThunderAutoTrajectory::getStartAction() const noexcept {
  if (!m_handle) {
    static std::string emptyString;
    return emptyString;
  }

  return m_handle->getStartAction();
}

const std::string& ThunderAutoTrajectory::getEndAction() const noexcept {
  if (!m_handle) {
    static std::string emptyString;
    return emptyString;
  }

  return m_handle->getEndAction();
}

const std::map<units::second_t, std::string>& ThunderAutoTrajectory::getStopActions() const noexcept {
  if (!m_handle) {
    static const std::map<units::second_t, std::string> emptyMap;
    return emptyMap;
  }

  return m_handle->getStopActions();
}

const std::multimap<units::second_t, std::string>& ThunderAutoTrajectory::getActions() const noexcept {
  if (!m_handle) {
    static const std::multimap<units::second_t, std::string> emptyMap;
    return emptyMap;
  }

  return m_handle->getActions();
}

driver::ThunderAutoTrajectory* ThunderAutoTrajectory::getHandle() noexcept {
  return m_handle;
}

const driver::ThunderAutoTrajectory* ThunderAutoTrajectory::getHandle() const noexcept {
  return m_handle;
}

TrajectoryState ThunderAutoTrajectory::convertState(
    const driver::ThunderAutoTrajectoryState& driverState) noexcept {
  TrajectoryState state;
  state.time = driverState.time;
  state.pose = driverState.pose;
  state.chassisSpeeds = driverState.chassisSpeeds;
  state.linearVelocity = driverState.linearVelocity;
  state.heading = driverState.heading;
  return state;
}

}  // namespace thunder
