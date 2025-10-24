#pragma once

#include <ThunderLibCore/Auto/ThunderAutoOutputTrajectory.hpp>
#include <string>
#include <unordered_set>
#include <map>
#include <memory>

namespace thunder::driver {

struct ThunderAutoTrajectoryState {
  units::second_t time = 0_s;
  frc::Pose2d pose;
  frc::ChassisSpeeds chassisSpeeds;
  units::meters_per_second_t linearVelocity = 0.0_mps;
  frc::Rotation2d heading;
};

class ThunderAutoTrajectory final {
 public:
  explicit ThunderAutoTrajectory(std::shared_ptr<core::ThunderAutoOutputTrajectory> trajectory) noexcept;
  ~ThunderAutoTrajectory();

  bool isValid() const noexcept;

  ThunderAutoTrajectoryState sample(units::second_t time) const noexcept;

  virtual units::second_t getDuration() const noexcept;

  virtual ThunderAutoTrajectoryState getInitialState() const noexcept;
  virtual ThunderAutoTrajectoryState getFinalState() const noexcept;

  const std::unordered_set<std::string>& getStartActions() const noexcept;
  const std::unordered_set<std::string>& getEndActions() const noexcept;

  const std::map<units::second_t, std::unordered_set<std::string>>& getStopActions() const noexcept;
  const std::multimap<units::second_t, std::string>& getActions() const noexcept;

 private:
   ThunderAutoTrajectoryState getState(size_t index) const noexcept;
   ThunderAutoTrajectoryState toState(const core::ThunderAutoOutputTrajectoryPoint& point) const noexcept;

 private:
  std::shared_ptr<core::ThunderAutoOutputTrajectory> m_trajectory;
};

}  // namespace thunder::driver
