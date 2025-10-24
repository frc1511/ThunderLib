#pragma once

#include <ThunderLib/Trajectory/Trajectory.hpp>
#include <string>
#include <unordered_set>
#include <map>

namespace thunder {

class ThunderAutoProject;

namespace driver {

struct ThunderAutoTrajectoryState;
class ThunderAutoTrajectory;

}  // namespace driver

class ThunderAutoTrajectory final : public thunder::Trajectory {
 public:
  ~ThunderAutoTrajectory();

  ThunderAutoTrajectory(const ThunderAutoTrajectory&) = delete;
  ThunderAutoTrajectory& operator=(const ThunderAutoTrajectory&) = delete;
  ThunderAutoTrajectory(ThunderAutoTrajectory&&) noexcept = delete;
  ThunderAutoTrajectory& operator=(ThunderAutoTrajectory&&) noexcept = delete;

  bool isValid() const noexcept override;

  /**
   * Samples the trajectory at the given time.
   *
   * @param time Time to sample at.
   *
   * @return Trajectory state at the given time.
   */
  TrajectoryState sample(units::second_t time) const noexcept override;

  /**
   * Gets the duration of the trajectory.
   *
   * @return Duration of the trajectory.
   */
  virtual units::second_t getDuration() const noexcept override;

  /**
   * Gets the initial state of the trajectory.
   *
   * @return Initial trajectory state.
   */
  virtual TrajectoryState getInitialState() const noexcept override;

  /**
   * Gets the final state of the trajectory.
   *
   * @return Final trajectory state.
   */
  virtual TrajectoryState getFinalState() const noexcept override;

  /**
   * Gets the set of actions to perform concurrently before starting the trajectory.
   *
   * @return Set of actions.
   */
  const std::unordered_set<std::string>& getStartActions() const noexcept;

  /**
   * Gets the set of actions to perform concurrently after finishing the trajectory.
   *
   * @return Set of actions.
   */
  const std::unordered_set<std::string>& getEndActions() const noexcept;

  /**
   * Gets the map of actions to perform concurrently at times at which the robot is stopped during the
   * trajectory.
   *
   * The robot should pause driving at these times to perform the actions and resume driving after the actions
   * are complete.
   *
   * @return Map of stop times to sets of actions.
   */
  const std::map<units::second_t, std::unordered_set<std::string>>& getStopActions() const noexcept;

  /**
   * Gets the map of actions to perform at specific times during the trajectory.
   *
   * These actions may continue to run while the robot is driving.
   *
   * @return Map of times to actions.
   */
  const std::multimap<units::second_t, std::string>& getActions() const noexcept;

  driver::ThunderAutoTrajectory* getHandle() noexcept;

 private:
  friend class ThunderAutoProject;  // TODO: ThunderAutoTrajectoryBuilder

  // Ownership is transferred to the constructed object.
  explicit ThunderAutoTrajectory(driver::ThunderAutoTrajectory* trajectory) noexcept;

 private:
  driver::ThunderAutoTrajectory* m_handle = nullptr;

 private:
  static TrajectoryState convertState(const driver::ThunderAutoTrajectoryState& driverState) noexcept;
};

}  // namespace thunder
