#include <ThunderLibDriver/Auto/ThunderAutoProject.hpp>
#include <ThunderLibDriver/Logger.hpp>
#include <ThunderLibDriver/Error.hpp>

struct HAL_ControlWord {
  uint32_t enabled : 1;
  uint32_t autonomous : 1;
  uint32_t test : 1;
  uint32_t eStop : 1;
  uint32_t fmsAttached : 1;
  uint32_t dsAttached : 1;
  uint32_t control_reserved : 26;
};

namespace thunder::driver {

ThunderAutoProject::ThunderAutoProject() noexcept
    : m_networkTableInstance(nt::NetworkTableInstance::GetDefault()),
      m_thunderAutoNetworkTable(m_networkTableInstance.GetTable("ThunderAuto")),
      m_fmsInfoNetworkTable(m_networkTableInstance.GetTable("FMSInfo")) {
  m_initializeTimestamp = std::chrono::steady_clock::now();
}

ThunderAutoProject::ThunderAutoProject(const std::filesystem::path& projectPath) noexcept
    : ThunderAutoProject() {
  load(projectPath);
}

ThunderAutoProject::~ThunderAutoProject() noexcept {
  // Deregister Network Tables listener.
  if (m_thunderAutoNetworkTable && m_ntRemoteUpdateListenerId != 0) {
    m_thunderAutoNetworkTable->RemoveListener(m_ntRemoteUpdateListenerId);
    m_ntRemoteUpdateListenerId = 0;
  }
}

bool ThunderAutoProject::load(const std::filesystem::path& projectPath) noexcept {
  if (m_project) {
    ThunderLibLogger::Warn("Cannot load project: another project is already loaded");
    return false;
  }

  std::filesystem::path absoluteProjectPath = projectPath;
  if (!projectPath.has_root_path()) {
    absoluteProjectPath = getDeployDirectoryPath() / projectPath;
  }

  if (!projectPath.has_extension()) {
    absoluteProjectPath.replace_extension(".thunderauto");
  }

  // Open Project.

  bool unknownError = false;
  try {
    m_project = core::LoadThunderAutoProject(absoluteProjectPath);

  } catch (const core::ThunderError& e) {
    ThunderLibLogger::Error("{}", e.message());
    return false;
  } catch (const std::exception& e) {
    ThunderLibLogger::Error("{}", e.what());
    return false;
  } catch (...) {
    unknownError = true;
  }

  if (unknownError || !m_project) {
    ThunderLibLogger::Error("Unknown error occurred while loading ThunderAuto project from '{}'",
                            absoluteProjectPath.string());
    return false;
  }

  // Build trajectories and auto modes.

  updateTrajectories(getTrajectoryNames(), {});
  updateAutoModes(getAutoModeNames(), {});

  // Setup Network Tables listener.

  m_ntRemoteUpdateListenerId = m_thunderAutoNetworkTable->AddListener(
      getName(), nt::EventFlags::kValueRemote | nt::EventFlags::kTimeSync,
      [this](nt::NetworkTable* table, std::string_view key, const nt::Event& event) {
        remoteUpdateReceived(event);
      });

  return true;
}

bool ThunderAutoProject::discoverAndLoadFromDeployDirectory() noexcept {
  std::filesystem::path deployDirectory = getDeployDirectoryPath();

  std::vector<std::filesystem::path> discoveredProjects = core::DiscoverThunderAutoProjects(deployDirectory);
  if (discoveredProjects.empty()) {
    ThunderLibLogger::Warn("No ThunderAuto projects found in deploy directory: '{}'",
                           deployDirectory.string());
    return false;
  }

  return load(discoveredProjects.front());
}

bool ThunderAutoProject::isLoaded() const noexcept {
  return m_project != nullptr;
}

std::string ThunderAutoProject::getName() const noexcept {
  if (!isLoaded())
    return "";

  std::lock_guard<std::mutex> lk(m_projectMutex);
  return m_project->settings().name;
}

bool ThunderAutoProject::hasAction(const std::string& actionName) const noexcept {
  std::lock_guard<std::mutex> lk(m_projectMutex);
  return m_project->state().actions.contains(actionName);
}

std::optional<core::ThunderAutoAction> ThunderAutoProject::getAction(
    const std::string& actionName) const noexcept {
  std::lock_guard<std::mutex> lk(m_projectMutex);

  auto it = m_project->state().actions.find(actionName);
  if (it == m_project->state().actions.end())
    return std::nullopt;

  return it->second;
}

ThunderAutoTrajectory* ThunderAutoProject::getTrajectory(const std::string& trajectoryName) const noexcept {
  std::lock_guard<std::mutex> lk(m_trajectoriesAndAutoModesMutex);

  auto it = m_trajectories.find(trajectoryName);
  if (it == m_trajectories.end())
    return nullptr;

  ThunderAutoTrajectory* thunderAutoTrajectory = new ThunderAutoTrajectory(it->second);
  return thunderAutoTrajectory;
}

bool ThunderAutoProject::hasTrajectory(const std::string& trajectoryName) const noexcept {
  if (!isLoaded())
    return false;

  std::lock_guard<std::mutex> lk(m_projectMutex);
  return m_project->state().trajectories.contains(trajectoryName);
}

std::unordered_set<std::string> ThunderAutoProject::getTrajectoryNames() const noexcept {
  if (!isLoaded())
    return {};

  std::lock_guard<std::mutex> lk(m_projectMutex);

  std::unordered_set<std::string> trajectoryNames;

  for (const auto& [name, _] : m_project->state().trajectories) {
    trajectoryNames.insert(name);
  }

  return trajectoryNames;
}

std::map<std::string, core::ThunderAutoTrajectorySkeleton> ThunderAutoProject::getTrajectorySkeletons()
    const noexcept {
  if (!isLoaded())
    return {};

  std::lock_guard<std::mutex> lk(m_trajectoriesAndAutoModesMutex);
  return m_trajectorySkeletons;
}

ThunderAutoMode* ThunderAutoProject::getAutoMode(const std::string& autoModeName) const noexcept {
  std::lock_guard<std::mutex> lk(m_trajectoriesAndAutoModesMutex);

  auto it = m_autoModes.find(autoModeName);
  if (it == m_autoModes.end())
    return nullptr;

  ThunderAutoMode* thunderAutoMode = new ThunderAutoMode(it->second);
  return thunderAutoMode;
}

bool ThunderAutoProject::hasAutoMode(const std::string& autoModeName) const noexcept {
  if (!isLoaded())
    return false;

  std::lock_guard<std::mutex> lk(m_projectMutex);
  return m_project->state().autoModes.contains(autoModeName);
}

std::unordered_set<std::string> ThunderAutoProject::getAutoModeNames() const noexcept {
  if (!isLoaded())
    return {};

  std::lock_guard<std::mutex> lk(m_projectMutex);

  std::unordered_set<std::string> autoModeNames;

  for (const auto& [name, _] : m_project->state().autoModes) {
    autoModeNames.insert(name);
  }

  return autoModeNames;
}

FieldSymmetry ThunderAutoProject::getFieldSymmetry() const noexcept {
  std::lock_guard<std::mutex> lk(m_projectMutex);

  const core::ThunderAutoFieldImage& field = m_project->settings().fieldImage;
  if (field.type() == core::ThunderAutoFieldImageType::CUSTOM) {
    return FieldSymmetry::NONE;
  }

  const core::ThunderAutoBuiltinFieldImage& fieldYear = field.builtinImage();

  switch (fieldYear) {
    using enum core::ThunderAutoBuiltinFieldImage;
    case FIELD_2026:
    case FIELD_2025:
    case FIELD_2022:
      return FieldSymmetry::ROTATIONAL;
    case FIELD_2024:
    case FIELD_2023:
      return FieldSymmetry::REFLECTIONAL;
    default:
      ThunderLibLogger::Warn("ThunderAuto project '{}' has unknown field image '{}', assuming no symmetry",
                             getName(), core::ThunderAutoBuiltinFieldImageToString(fieldYear));
      return FieldSymmetry::NONE;
  }

  return FieldSymmetry::NONE;
}

FieldDimensions ThunderAutoProject::getFieldDimensions() const noexcept {
  std::lock_guard<std::mutex> lk(m_projectMutex);

  const core::ThunderAutoFieldImage& field = m_project->settings().fieldImage;

  core::Measurement2d size = field.fieldSize();
  FieldDimensions dimensions{.width = size.width, .length = size.length};
  return dimensions;
}

void ThunderAutoProject::setRemoteUpdatesEnabled(bool enabled) noexcept {
  m_remoteUpdatesEnabled = enabled;
}

void ThunderAutoProject::enableRemoteUpdates() noexcept {
  m_remoteUpdatesEnabled = true;
}

void ThunderAutoProject::disableRemoteUpdates() noexcept {
  m_remoteUpdatesEnabled = false;
}

bool ThunderAutoProject::areRemoteUpdatesEnabled() const noexcept {
  return m_remoteUpdatesEnabled;
}

ThunderAutoProject::RemoteUpdateSubscriberID ThunderAutoProject::registerRemoteUpdateSubscriber(
    BasicRemoteUpdateCallbackFunc basicCallback) noexcept {
  if (!basicCallback)
    return 0;

  RemoteUpdateCallbackFunc callback = [basicCallback](auto, auto, auto, auto) { basicCallback(); };

  m_remoteUpdateSubscribers.emplace(m_nextRemoteUpdateSubscriberId, callback);
  return m_nextRemoteUpdateSubscriberId++;
}

ThunderAutoProject::RemoteUpdateSubscriberID ThunderAutoProject::registerRemoteUpdateSubscriber(
    RemoteUpdateCallbackFunc callback) noexcept {
  if (!callback)
    return 0;

  m_remoteUpdateSubscribers.emplace(m_nextRemoteUpdateSubscriberId, callback);
  return m_nextRemoteUpdateSubscriberId++;
}

bool ThunderAutoProject::unregisterRemoteUpdateSubscriber(RemoteUpdateSubscriberID subscriberId) noexcept {
  return m_remoteUpdateSubscribers.erase(subscriberId) > 0;
}

void ThunderAutoProject::remoteUpdateReceived(const nt::Event& event) noexcept {
  if (!m_remoteUpdatesEnabled || !isLoaded())
    return;

  const std::string remoteIP = event.GetConnectionInfo() ? event.GetConnectionInfo()->remote_ip : "UNKNOWN";

  auto now = std::chrono::steady_clock::now();
  auto timeSinceInit = now - m_initializeTimestamp;
  if (timeSinceInit < std::chrono::seconds(10)) {
    // An update may have already been published to NetworkTables before the robot program started, but we
    // still get the notification. To avoid this issue, we ignore any remote updates received within 10 seconds
    // of initialization.
    ThunderLibLogger::Warn(
        "Remote update for project '{}' from '{}' skipped: received too soon after initialization", getName(),
        remoteIP);
    return;
  }

  std::unique_lock<std::mutex> lock(m_remoteUpdateMutex, std::try_to_lock);
  if (!lock.owns_lock()) {
    ThunderLibLogger::Warn("Remote update for project '{}' from '{}' skipped: another update is in progress",
                           getName(), remoteIP);
    return;
  }

  {
    nt::Value controlWordValue = m_fmsInfoNetworkTable->GetValue("FMSControlData");
    if (!controlWordValue.IsValid()) {
      ThunderLibLogger::Warn(
          "Remote update for project '{}' from '{}' skipped: Could not get FMS control data", getName(),
          remoteIP);
      return;
    }

    int64_t controlWordInt = controlWordValue.GetInteger();

    HAL_ControlWord* controlWord = reinterpret_cast<HAL_ControlWord*>(&controlWordInt);
    if (controlWord->fmsAttached) {
      ThunderLibLogger::Warn("Remote update for project '{}' from '{}' skipped: FMS is connected", getName(),
                             remoteIP);
      return;
    }
    if (controlWord->enabled) {
      ThunderLibLogger::Warn("Remote update for project '{}' from '{}' skipped: robot is enabled", getName(),
                             remoteIP);
      return;
    }
  }

  // Deserialize new state.

  std::span<const uint8_t> eventData = event.GetValueEventData()->value.GetRaw();

  core::ThunderAutoProjectState newState;
  core::ThunderAutoProjectStateDataHashes newHashes;

  try {
    newHashes = core::DeserializeThunderAutoProjectStateFromTransmission(eventData, newState);

  } catch (const core::ThunderError& e) {
    ThunderLibLogger::Error("Remote update for project '{}' from '{}' failed: {}", getName(), remoteIP,
                            e.message());
    return;
  } catch (const std::exception& e) {
    ThunderLibLogger::Error("Remote update for project '{}' from '{}' failed: {}", getName(), remoteIP,
                            e.what());
    return;
  } catch (...) {
    ThunderLibLogger::Error("Remote update for project '{}' from '{}' failed: unknown error", getName(),
                            remoteIP);
    return;
  }

  // Determine differences between new state and current state.

  const core::ThunderAutoProjectStateDataHashes::Diff stateDiff = newHashes.diff(m_stateHashes);

  if (!stateDiff.stateWasChanged) {
    ThunderLibLogger::Info("Remote update for project '{}' received from '{}' but no changes detected",
                           getName(), remoteIP);
    return;
  }

  ThunderLibLogger::Info("Remote update for project '{}' received from '{}'", getName(), remoteIP);
  // TODO: Notify ThunderAuto about successful remote update.

  // Apply new state.

  {
    std::lock_guard<std::mutex> lk(m_projectMutex);

    m_project->state() = std::move(newState);
    m_stateHashes = std::move(newHashes);
  }

  updateTrajectories(stateDiff.updatedTrajectories, stateDiff.removedTrajectories);
  updateAutoModes(stateDiff.updatedAutoModes, stateDiff.removedAutoModes);

  // Notify subscribers.

  callUpdateSubscribers(stateDiff.updatedTrajectories, stateDiff.removedTrajectories,
                        stateDiff.updatedAutoModes, stateDiff.removedAutoModes);
}

void ThunderAutoProject::updateTrajectories(
    const std::unordered_set<std::string>& updatedTrajectoryNames,
    const std::unordered_set<std::string>& removedTrajectoryNames) noexcept {
  std::lock_guard<std::mutex> lk1(m_projectMutex);
  std::lock_guard<std::mutex> lk2(m_trajectoriesAndAutoModesMutex);

  const core::ThunderAutoProjectState& state = m_project->state();

  for (const std::string& name : updatedTrajectoryNames) {
    auto skeletonIt = state.trajectories.find(name);
    ThunderLibAssert(skeletonIt != state.trajectories.end(),
                     "Updated trajectory '{}' not found in project state", name);

    const core::ThunderAutoTrajectorySkeleton& skeleton = skeletonIt->second;

    m_trajectorySkeletons[name] = skeleton;

    m_trajectories.erase(name);

    std::unique_ptr<core::ThunderAutoOutputTrajectory> outputTrajectory;

    bool unknownError = false;
    try {
      outputTrajectory =
          core::BuildThunderAutoOutputTrajectory(skeleton, core::kHighResOutputTrajectorySettings);

    } catch (const core::ThunderError& e) {
      ThunderLibLogger::Error("Failed to build trajectory '{}': {}", name, e.message());
      continue;
    } catch (const std::exception& e) {
      ThunderLibLogger::Error("Failed to build trajectory '{}': {}", name, e.what());
      continue;
    } catch (...) {
      unknownError = true;
    }

    if (unknownError || !outputTrajectory) {
      ThunderLibLogger::Error("Failed to build trajectory '{}' due to unknown error", name);
      continue;
    }

    std::shared_ptr<core::ThunderAutoOutputTrajectory> sharedOutputTrajectory = std::move(outputTrajectory);
    m_trajectories.emplace(name, sharedOutputTrajectory);
  }

  for (const std::string& name : removedTrajectoryNames) {
    m_trajectories.erase(name);
    m_trajectorySkeletons.erase(name);
  }
}

void ThunderAutoProject::updateAutoModes(const std::unordered_set<std::string>& updatedAutoModes,
                                         const std::unordered_set<std::string>& removedAutoModes) noexcept {
  std::lock_guard<std::mutex> lk1(m_projectMutex);
  std::lock_guard<std::mutex> lk2(m_trajectoriesAndAutoModesMutex);

  const core::ThunderAutoProjectState& state = m_project->state();

  for (const std::string& name : updatedAutoModes) {
    auto it = state.autoModes.find(name);
    ThunderLibAssert(skeletonIt != state.trajectories.end(),
                     "Updated auto mode '{}' not found in project state", name);

    // Just copy the auto modes.
    m_autoModes[name] = std::make_shared<core::ThunderAutoMode>(it->second);
  }

  for (const std::string& name : removedAutoModes) {
    m_autoModes.erase(name);
  }
}

void ThunderAutoProject::callUpdateSubscribers(
    const std::unordered_set<std::string>& updatedTrajectories,
    const std::unordered_set<std::string>& removedTrajectories,
    const std::unordered_set<std::string>& updatedAutoModes,
    const std::unordered_set<std::string>& removedAutoModes) noexcept {
  for (auto& [id, callback] : m_remoteUpdateSubscribers) {
    callback(updatedTrajectories, removedTrajectories, updatedAutoModes, removedAutoModes);
  }
}

std::filesystem::path ThunderAutoProject::getDeployDirectoryPath() noexcept {
  std::filesystem::path deployDirectory;
#if defined(__FRC_ROBORIO__)
  deployDirectory = "/home/lvuser/deploy";
#elif defined(__FRC_SYSTEMCORE__)
  deployDirectory = "/home/systemcore/deploy";
#else
  deployDirectory = std::filesystem::current_path() / "src" / "main" / "deploy";
#endif
  return deployDirectory;
}

}  // namespace thunder::driver
