#pragma once

#include <ThunderLibDriver/Auto/ThunderAutoTrajectory.hpp>
#include <ThunderLibDriver/Auto/ThunderAutoMode.hpp>
#include <ThunderLibCore/Auto/ThunderAutoProject.hpp>
#include <ThunderLibCore/Auto/ThunderAutoOutputTrajectory.hpp>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <ntcore_c.h>
#include <filesystem>
#include <functional>
#include <unordered_set>
#include <unordered_map>
#include <string>
#include <memory>
#include <mutex>
#include <cstddef>
#include <optional>

namespace thunder::driver {

enum class FieldSymmetry {
  NONE,
  ROTATIONAL,
  REFLECTIONAL,
};

struct FieldDimensions {
  units::meter_t width;
  units::meter_t length;
};

class ThunderAutoProject {
 public:
  ThunderAutoProject() noexcept;
  explicit ThunderAutoProject(const std::filesystem::path& projectPath) noexcept;

  ~ThunderAutoProject() noexcept;

  bool load(const std::filesystem::path& projectPath) noexcept;
  bool discoverAndLoadFromDeployDirectory() noexcept;

  bool isLoaded() const noexcept;

  std::string getName() const noexcept;

  bool hasAction(const std::string& actionName) const noexcept;

  std::optional<core::ThunderAutoAction> getAction(const std::string& actionName) const noexcept;

  // Ownership of returned pointer is transferred to caller.
  [[nodiscard]]
  ThunderAutoTrajectory* getTrajectory(const std::string& trajectoryName) const noexcept;

  bool hasTrajectory(const std::string& trajectoryName) const noexcept;

  std::unordered_set<std::string> getTrajectoryNames() const noexcept;

  // Ownership of returned pointer is transferred to caller.
  [[nodiscard]]
  ThunderAutoMode* getAutoMode(const std::string& autoModeName) const noexcept;

  bool hasAutoMode(const std::string& autoModeName) const noexcept;

  std::unordered_set<std::string> getAutoModeNames() const noexcept;

  FieldSymmetry getFieldSymmetry() const noexcept;
  FieldDimensions getFieldDimensions() const noexcept;

  void setRemoteUpdatesEnabled(bool enabled) noexcept;
  void enableRemoteUpdates() noexcept;
  void disableRemoteUpdates() noexcept;
  bool areRemoteUpdatesEnabled() const noexcept;

  using BasicRemoteUpdateCallbackFunc = std::function<void()>;
  using RemoteUpdateCallbackFunc =
      std::function<void(const std::unordered_set<std::string>& updatedTrajectories,
                         const std::unordered_set<std::string>& removedTrajectories,
                         const std::unordered_set<std::string>& updatedAutoModes,
                         const std::unordered_set<std::string>& removedAutoModes)>;
  using RemoteUpdateSubscriberID = size_t;

  RemoteUpdateSubscriberID registerRemoteUpdateSubscriber(BasicRemoteUpdateCallbackFunc callback) noexcept;
  RemoteUpdateSubscriberID registerRemoteUpdateSubscriber(RemoteUpdateCallbackFunc callback) noexcept;
  bool unregisterRemoteUpdateSubscriber(RemoteUpdateSubscriberID subscriberId) noexcept;

 private:
  void remoteUpdateReceived(const nt::Event& event) noexcept;

  void updateTrajectories(const std::unordered_set<std::string>& updatedTrajectoryNames,
                          const std::unordered_set<std::string>& removedTrajectoryNames) noexcept;
  void updateAutoModes(const std::unordered_set<std::string>& updatedAutoModes,
                       const std::unordered_set<std::string>& removedAutoModes) noexcept;

  void callUpdateSubscribers(const std::unordered_set<std::string>& updatedTrajectories,
                             const std::unordered_set<std::string>& removedTrajectories,
                             const std::unordered_set<std::string>& updatedAutoModes,
                             const std::unordered_set<std::string>& removedAutoModes) noexcept;

 private:
 static std::filesystem::path getDeployDirectoryPath() noexcept;

 private:
  nt::NetworkTableInstance m_networkTableInstance;
  std::shared_ptr<nt::NetworkTable> m_thunderAutoNetworkTable;
  NT_Listener m_ntRemoteUpdateListenerId = 0;

  bool m_remoteUpdatesEnabled = true;
  std::unordered_map<size_t, RemoteUpdateCallbackFunc> m_remoteUpdateSubscribers;
  size_t m_nextRemoteUpdateSubscriberId = 1;

  std::unique_ptr<core::ThunderAutoProject> m_project;
  core::ThunderAutoProjectStateDataHashes m_stateHashes;
  mutable std::mutex m_projectMutex;

  std::unordered_map<std::string, std::shared_ptr<core::ThunderAutoOutputTrajectory>> m_trajectories;
  // // TODO: auto modes
  mutable std::mutex m_trajectoriesMutex;
};

}  // namespace thunder::driver
