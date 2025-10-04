#include <ThunderLibCore/Auto/ThunderAutoProject.hpp>

#include <ThunderLibCore/Logger.hpp>
#include <ThunderLibCore/Error.hpp>

#include <fmt/ranges.h>
#include <fstream>
#include <queue>

namespace thunder::core {

static constexpr ThunderAutoProjectVersion kThunderAutoProjectVersionCurrent = {
    THUNDERAUTO_PROJECT_VERSION_MAJOR, THUNDERAUTO_PROJECT_VERSION_MINOR};

static void from_json(const wpi::json& json, ThunderAutoProjectVersion& version) {
  json.at("major").get_to(version.major);
  json.at("minor").get_to(version.minor);
}

static void to_json(wpi::json& json, const ThunderAutoProjectVersion& version) noexcept {
  json = wpi::json{
      {"major", version.major},
      {"minor", version.minor},
  };
}

static void from_json(const wpi::json& json, ThunderAutoActionType& type) {
  std::string str = json.get<std::string>();

  using enum ThunderAutoActionType;
  if (str == "command") {
    type = COMMAND;
    return;
  } else if (str == "sequential_action_group") {
    type = SEQUENTIAL_ACTION_GROUP;
    return;
  } else if (str == "concurrent_action_group") {
    type = CONCURRENT_ACTION_GROUP;
    return;
  }

  throw RuntimeError::Construct("Invalid action type: '{}'", str);
}

static void to_json(wpi::json& json, const ThunderAutoActionType& type) noexcept {
  switch (type) {
    using enum ThunderAutoActionType;
    case COMMAND:
      json = "command";
      break;
    case SEQUENTIAL_ACTION_GROUP:
      json = "sequential_action_group";
      break;
    case CONCURRENT_ACTION_GROUP:
      json = "concurrent_action_group";
      break;
    default:
      ThunderLibUnreachable("Invalid action type");
  }
}

static void from_json(const wpi::json& json, ThunderAutoAction& action) {
  ThunderAutoActionType type;
  json.at("type").get_to(type);
  action.setType(type);

  switch (action.type()) {
    using enum ThunderAutoActionType;
    case COMMAND:
      break;
    case SEQUENTIAL_ACTION_GROUP:
    case CONCURRENT_ACTION_GROUP: {
      std::vector<std::string> m_actionGroupVec;
      json.at("action_group").get_to(m_actionGroupVec);
      action.setActionGroup(m_actionGroupVec);
    } break;
    default:
      ThunderLibUnreachable("Invalid action type");
  }
}

static void to_json(wpi::json& json, const ThunderAutoAction& action) noexcept {
  json["type"] = action.type();

  switch (action.type()) {
    using enum ThunderAutoActionType;
    case COMMAND:
      break;
    case SEQUENTIAL_ACTION_GROUP:
    case CONCURRENT_ACTION_GROUP:
      json["action_group"] = action.actionGroup();
      break;
    default:
      ThunderLibUnreachable("Invalid action type");
  }
}

const char* ThunderAutoActionTypeToString(ThunderAutoActionType type) noexcept {
  switch (type) {
    using enum ThunderAutoActionType;
    case COMMAND:
      return "Command";
      break;
    case SEQUENTIAL_ACTION_GROUP:
      return "Sequential Action Group";
      break;
    case CONCURRENT_ACTION_GROUP:
      return "Concurrent Action Group";
    default:
      ThunderLibUnreachable("Invalid action type");
  }
}

ThunderAutoAction::ThunderAutoAction(ThunderAutoActionType type,
                                     const std::vector<std::string>& actionGroupVec /*= {}*/)
    : m_type(type) {
  setActionGroup(actionGroupVec);
}

ThunderAutoAction::ThunderAutoAction(const ThunderAutoAction& other)
    : m_actionGroupVec(other.m_actionGroupVec),
      m_actionGroupMap(other.m_actionGroupMap),
      m_type(other.m_type) {}

ThunderAutoAction& ThunderAutoAction::operator=(const ThunderAutoAction& other) noexcept {
  if (this == &other) {
    return *this;
  }

  m_actionGroupVec = other.m_actionGroupVec;
  m_actionGroupMap = other.m_actionGroupMap;
  m_type = other.m_type;

  return *this;
}

void ThunderAutoAction::setType(ThunderAutoActionType type) noexcept {
  if (m_type != type &&
      (m_type == ThunderAutoActionType::COMMAND || type == ThunderAutoActionType::COMMAND)) {
    m_actionGroupMap.clear();
    m_actionGroupVec.clear();
  }

  m_type = type;
}

bool ThunderAutoAction::hasGroupAction(const std::string& actionName) noexcept {
  return m_actionGroupMap.contains(actionName);
}

bool ThunderAutoAction::addGroupAction(const std::string& actionName) noexcept {
  if (hasGroupAction(actionName)) {
    ThunderLibLogger::Warn("Cannot add group action '{}' because action is already in group", actionName);
    return false;
  }

  m_actionGroupVec.push_back(actionName);
  m_actionGroupMap.emplace(actionName, m_actionGroupVec.size() - 1);
  return true;
}

void ThunderAutoAction::renameGroupAction(const std::string& originalActionName,
                                          const std::string& newActionName) {
  if (hasGroupAction(newActionName)) {
    throw LogicError::Construct(
        "Cannot rename group action to '{}' because action with that name is already in group",
        newActionName);
  }

  auto mapIt = m_actionGroupMap.find(originalActionName);
  if (mapIt == m_actionGroupMap.end())
    return;

  size_t vecIndex = mapIt->second;

  m_actionGroupVec[vecIndex] = newActionName;

  auto mapNodeHandle = m_actionGroupMap.extract(mapIt);
  mapNodeHandle.key() = newActionName;
  m_actionGroupMap.insert(std::move(mapNodeHandle));
}

void ThunderAutoAction::setActionGroup(const std::vector<std::string>& actionGroupVec) {
  m_actionGroupMap.clear();
  m_actionGroupVec = actionGroupVec;

  for (size_t i = 0; i < m_actionGroupVec.size(); ++i) {
    m_actionGroupMap.emplace(m_actionGroupVec[i], i);
  }
}

bool ThunderAutoAction::removeGroupAction(const std::string& actionToRemoveName) noexcept {
  auto mapIt = m_actionGroupMap.find(actionToRemoveName);

  if (mapIt == m_actionGroupMap.end())
    return false;

  size_t vecIndex = mapIt->second;
  auto vecIt = m_actionGroupVec.begin() + vecIndex;

  m_actionGroupVec.erase(vecIt);
  m_actionGroupMap.erase(mapIt);

  for (auto& [actionName, index] : m_actionGroupMap) {
    if (index > vecIndex) {
      index--;
    }
  }

  return true;
}

bool ThunderAutoAction::swapGroupActionWithNext(const std::string actionName) {
  auto mapIt = m_actionGroupMap.find(actionName);
  if (mapIt == m_actionGroupMap.end()) {
    throw LogicError::Construct("Action to swap does not exist: {}", actionName);
  }

  size_t vecIndex = mapIt->second;
  if (vecIndex == m_actionGroupVec.size() - 1) {
    ThunderLibLogger::Warn("Cannot swap last group action with next");
    return false;
  }

  const std::string otherActionName = m_actionGroupVec[vecIndex + 1];

  std::swap(m_actionGroupVec[vecIndex], m_actionGroupVec[vecIndex + 1]);
  std::swap(m_actionGroupMap[actionName], m_actionGroupMap[otherActionName]);

  return true;
}

bool ThunderAutoAction::swapGroupActionWithPrevious(const std::string actionName) {
  auto mapIt = m_actionGroupMap.find(actionName);
  if (mapIt == m_actionGroupMap.end()) {
    throw LogicError::Construct("Action to swap does not exist: {}", actionName);
  }

  size_t vecIndex = mapIt->second;
  if (vecIndex == 0) {
    ThunderLibLogger::Warn("Cannot swap first group action '{}' with previous", actionName);
    return false;
  }

  const std::string otherActionName = m_actionGroupVec[vecIndex - 1];

  std::swap(m_actionGroupVec[vecIndex], m_actionGroupVec[vecIndex - 1]);
  std::swap(m_actionGroupMap[actionName], m_actionGroupMap[otherActionName]);

  return true;
}

ThunderAutoProjectSettings::ThunderAutoProjectSettings() noexcept
    : fieldImage(ThunderAutoBuiltinFieldImage::LATEST),
      driveController(DriveControllerType::HOLONOMIC),
      robotSize(Measurement2d(0.8_m, 0.8_m)),
      autoSave(false),
      autoCSVExport(false) {}

ThunderAutoProjectSettings::ThunderAutoProjectSettings(const std::filesystem::path& path,
                                                       const wpi::json& json,
                                                       const ThunderAutoProjectVersion& version)
    : ThunderAutoProjectSettings() {
  fromJson(path, json, version);
}

void ThunderAutoProjectSettings::fromJson(const std::filesystem::path& path,
                                          const wpi::json& json,
                                          const ThunderAutoProjectVersion& version) {
  if (path.empty()) {
    throw InvalidArgumentError::Construct("Project path cannot be empty");
  }

  setProjectPath(path);

  if (version.major < 2026) {
    // Maintain compatibility with older versions (update state to new format).
    fromJsonPre2026Version(json);
    return;
  }

  ThunderLibAssert(version.major <= THUNDERAUTO_PROJECT_VERSION_MAJOR, "Version too new");

  fromJsonCurrentVersion(json);
}

void ThunderAutoProjectSettings::setProjectPath(const std::filesystem::path& path) noexcept {
  if (path.empty()) {
    projectPath.clear();
    directory.clear();
    name.clear();
    return;
  }
  projectPath = path;
  directory = path.parent_path();
  name = path.stem().string();
}

void ThunderAutoProjectSettings::fromJsonPre2026Version(const wpi::json& json) {
  fieldImage.fromJson(json.at("field"), directory);

  json.at("drive_ctrl").get_to(driveController);

  double robotLength, robotWidth;
  json.at("robot_length").get_to(robotLength);
  json.at("robot_width").get_to(robotWidth);
  robotSize.length = units::meter_t(robotLength);
  robotSize.width = units::meter_t(robotWidth);

  autoSave = json.value("auto_save", false);
  autoCSVExport = json.value("auto_export", false);

  csvExportProps = ThunderAutoCSVExportProperties::LegacyDefault();
}

void ThunderAutoProjectSettings::fromJsonCurrentVersion(const wpi::json& json) {
  fieldImage.fromJson(json.at("field"), directory);

  json.at("drive_controller").get_to(driveController);

  double robotLength, robotWidth;
  json.at("robot_length").get_to(robotLength);
  json.at("robot_width").get_to(robotWidth);
  robotSize.length = units::meter_t(robotLength);
  robotSize.width = units::meter_t(robotWidth);
  if (json.contains("robot_corner_radius")) {
    double cornerRadius;
    json.at("robot_corner_radius").get_to(cornerRadius);
    robotCornerRadius = units::meter_t(cornerRadius);
  }

  autoSave = json.value("auto_save", false);
  autoCSVExport = json.value("auto_csv_export", false);

  json.at("csv_export_props").get_to(csvExportProps);
}

static void to_json(wpi::json& json, const ThunderAutoProjectSettings& settings) noexcept {
  json = wpi::json{
      {"drive_controller", static_cast<size_t>(settings.driveController)},
      {"robot_length", settings.robotSize.length.value()},
      {"robot_width", settings.robotSize.width.value()},
      {"robot_corner_radius", settings.robotCornerRadius.value()},
      {"auto_save", settings.autoSave},
      {"auto_export", settings.autoCSVExport},
      {"csv_export_props", settings.csvExportProps},

      /*
       * TODO: Remove this at some point in the future. 2025.4.0 looks at this and will produce an error when
       * it sees that the version is >2025 instead of crashing. Once everybody is upgraded to 2026 this can be
       * removed.
       */
      {"version_major", THUNDERAUTO_PROJECT_VERSION_MAJOR},
  };

  settings.fieldImage.toJson(settings.directory, json["field"]);
}

ThunderAutoProjectState::ThunderAutoProjectState(const wpi::json& json,
                                                 const ThunderAutoProjectVersion& version)
    : ThunderAutoProjectState() {
  fromJson(json, version);
}

ThunderAutoProjectState::ThunderAutoProjectState(const ThunderAutoProjectState& other) noexcept
    : m_actions(other.m_actions),
      m_actionsOrder(other.m_actionsOrder),
      trajectories(other.trajectories),
      autoModes(other.autoModes),
      waypointLinks(other.waypointLinks),
      editorState(other.editorState) {}

ThunderAutoProjectState& ThunderAutoProjectState::operator=(const ThunderAutoProjectState& other) noexcept {
  if (this == &other) {
    return *this;
  }

  m_actions = other.m_actions;
  m_actionsOrder = other.m_actionsOrder;
  trajectories = other.trajectories;
  autoModes = other.autoModes;
  waypointLinks = other.waypointLinks;
  editorState = other.editorState;
  return *this;
}

void ThunderAutoProjectState::fromJson(const wpi::json& json, const ThunderAutoProjectVersion& version) {
  if (version.major < 2026) {
    // Maintain compatibility with older versions (update state to new format).
    fromJsonPre2026Version(json);
    return;
  }

  ThunderLibAssert(version.major <= THUNDERAUTO_PROJECT_VERSION_MAJOR, "Version too new");

  fromJsonCurrentVersion(json);
}

void ThunderAutoProjectState::fromJsonPre2026Version(const wpi::json& json) {
  json.at("actions").get_to(m_actionsOrder);

  m_actions.clear();
  for (const std::string& actionName : m_actionsOrder) {
    m_actions.emplace(actionName, ThunderAutoAction{});  // TODO: Default action
  }

  std::vector<std::string> waypointLinksVec;
  if (json.contains("waypoint_links")) {
    json.at("waypoint_links").get_to(waypointLinksVec);
  }

  waypointLinks.clear();
  waypointLinks.insert(waypointLinksVec.begin(), waypointLinksVec.end());

  // Convert the trajectories to the new format.
  {
    const wpi::json& trajectoriesJson = json.at("paths");

    for (const std::pair<std::string, wpi::json> namedTrajectoryJson : trajectoriesJson) {
      const auto& [name, trajectoryJson] = namedTrajectoryJson;
      ThunderAutoTrajectorySkeleton trajectory;
      trajectory.fromJsonPre2026(trajectoryJson, m_actionsOrder, waypointLinksVec);
      trajectories[name] = trajectory;
    }
  }

  // These are new.
  autoModes.clear();
  editorState = ThunderAutoEditorState{};
}

void ThunderAutoProjectState::fromJsonCurrentVersion(const wpi::json& json) {
  json.at("trajectories").get_to(trajectories);
  json.at("auto_modes").get_to(autoModes);

  json.at("actions_order").get_to(m_actionsOrder);
  json.at("actions").get_to(m_actions);

  json.at("waypoint_links").get_to(waypointLinks);
  json.at("editor_state").get_to(editorState);

  validateActions();
  validateWaypointLinks();
}

void ThunderAutoProjectState::validateActions() {
  // Validate references in action groups.

  for (auto& [actionName, actionInfo] : m_actions) {
    using enum ThunderAutoActionType;
    ThunderAutoActionType type = actionInfo.type();

    if (type == COMMAND)
      continue;

    for (const std::string& subActionName : actionInfo.actionGroup()) {
      if (!m_actions.contains(subActionName)) {
        throw RuntimeError::Construct("Group action '{}' contains reference to non-existent action '{}'",
                                      actionName, subActionName);
      }
    }
  }

  // Check for recursive action definitions.

  for (auto& [actionName, actionInfo] : m_actions) {
    std::list<std::string> path = findActionRecursionPath(actionName);
    std::string pathStr = fmt::format("{}", fmt::join(path, " -> "));
    if (!path.empty()) {
      throw RuntimeError::Construct("Action '{}' is recursively defined: {}", actionName, pathStr);
    }
  }

  // Validate references in trajectories.

  for (auto& [name, skeleton] : trajectories) {
    for (auto actionIt = skeleton.actions().begin(); actionIt != skeleton.actions().end(); ++actionIt) {
      if (!m_actions.contains(actionIt->second.action)) {
        throw RuntimeError::Construct("Trajectory '{}' contains reference to non-existent action '{}'", name,
                                      actionIt->second.action);
      }
    }

    for (auto pointIt = skeleton.begin(); pointIt != skeleton.end(); pointIt++) {
      if (pointIt->isStopped()) {
        for (const std::string& stopAction : pointIt->stopActions()) {
          if (!m_actions.contains(stopAction)) {
            Point2d pos = pointIt->position();
            throw RuntimeError::Construct(
                "Trajectory '{}' contains reference to non-existent stop action '{}' at waypoint ({}, {})",
                name, stopAction, pos.x(), pos.y());
          }
        }
      }
    }

    for (const std::string& startActionName : skeleton.startActions()) {
      if (!m_actions.contains(startActionName)) {
        throw RuntimeError::Construct("Trajectory '{}' contains reference to non-existent start action '{}'",
                                      name, startActionName);
      }
    }
    for (const std::string& endActionName : skeleton.endActions()) {
      if (!m_actions.contains(endActionName)) {
        throw RuntimeError::Construct("Trajectory '{}' contains reference to non-existent end action '{}'",
                                      name, endActionName);
      }
    }
  }

  // TODO: Validate references in auto modes
}

void ThunderAutoProjectState::validateWaypointLinks() {
  for (const auto& [name, skeleton] : trajectories) {
    for (const auto& point : skeleton) {
      const std::string linkName(point.linkName());
      if (!linkName.empty() && !waypointLinks.contains(linkName)) {
        Point2d pos = point.position();
        throw RuntimeError::Construct(
            "Trajectory '{}' contains reference to non-existent waypoint link '{}' at waypoint ({}, {})",
            name, linkName, pos.x(), pos.y());
      }
    }
  }
}

std::optional<frc::Pose2d> ThunderAutoProjectState::getAutoModeInitialPose(
    std::string_view autoModeName) const noexcept {
  // TODO: Once auto modes are added

  return std::nullopt;
}

void ThunderAutoProjectState::addAction(const std::string& actionName,
                                        const ThunderAutoAction& actionInfo) noexcept {
  if (m_actions.contains(actionName))
    return;

  m_actions.emplace(actionName, actionInfo);
  m_actionsOrder.push_back(actionName);
}

ThunderAutoAction& ThunderAutoProjectState::getAction(const std::string& actionName) {
  if (!m_actions.contains(actionName)) {
    throw LogicError::Construct("Action to get does not exist: {}", actionName);
  }
  return m_actions.at(actionName);
}

const ThunderAutoAction& ThunderAutoProjectState::getAction(const std::string& actionName) const {
  if (!m_actions.contains(actionName)) {
    throw LogicError::Construct("Action to get does not exist: {}", actionName);
  }
  return m_actions.at(actionName);
}

void ThunderAutoProjectState::removeAction(const std::string& actionToRemoveName) noexcept {
  if (!m_actions.contains(actionToRemoveName))
    return;
  {
    m_actions.erase(actionToRemoveName);
    auto it = std::remove(m_actionsOrder.begin(), m_actionsOrder.end(), actionToRemoveName);
    ThunderLibAssert(it != m_actionsOrder.end(), "Action to remove not found in actions order");
    m_actionsOrder.erase(it, m_actionsOrder.end());
  }

  // Rename actions in action groups

  for (auto& [actionName, actionInfo] : m_actions) {
    using enum ThunderAutoActionType;
    ThunderAutoActionType type = actionInfo.type();
    if (type != COMMAND) {
      actionInfo.removeGroupAction(actionToRemoveName);
    }
  }

  // Remove all references to action in trajectories.

  for (auto& [name, skeleton] : trajectories) {
    std::vector<std::multimap<ThunderAutoTrajectoryPosition, ThunderAutoTrajectoryAction>::iterator> toRemove;

    for (auto actionIt = skeleton.actions().begin(); actionIt != skeleton.actions().end(); ++actionIt) {
      if (actionIt->second.action == actionToRemoveName) {
        toRemove.push_back(actionIt);
      }
    }

    for (auto it : toRemove) {
      skeleton.actions().remove(it);
    }

    for (auto pointIt = skeleton.begin(); pointIt != skeleton.end(); pointIt++) {
      if (pointIt->isStopped()) {
        if (pointIt->hasStopAction(actionToRemoveName)) {
          pointIt->removeStopAction(actionToRemoveName);
        }
      }
    }

    if (skeleton.hasStartAction(actionToRemoveName)) {
      skeleton.removeStartAction(actionToRemoveName);
    }
    if (skeleton.hasEndAction(actionToRemoveName)) {
      skeleton.removeEndAction(actionToRemoveName);
    }
  }

  // TODO: Remove references in auto modes
}

void ThunderAutoProjectState::renameAction(const std::string& oldName, const std::string& newName) {
  if (!m_actions.contains(oldName))
    return;

  if (m_actions.contains(newName)) {
    throw LogicError::Construct("Action cannot be renamed to an existing action name: {}", newName);
  }

  {
    ThunderAutoAction actionInfo = m_actions[oldName];

    m_actions.erase(oldName);
    m_actions.emplace(newName, actionInfo);
    auto it = std::find(m_actionsOrder.begin(), m_actionsOrder.end(), oldName);
    ThunderLibAssert(it != m_actionsOrder.end(), "Old action name not found in actions order");
    *it = newName;
  }

  // Rename actions in action groups

  for (auto& [actionName, actionInfo] : m_actions) {
    using enum ThunderAutoActionType;
    ThunderAutoActionType type = actionInfo.type();
    if (type != COMMAND) {
      actionInfo.renameGroupAction(oldName, newName);
    }
  }

  // Rename all references to action in trajectories.

  for (auto& [name, skeleton] : trajectories) {
    for (auto actionIt = skeleton.actions().begin(); actionIt != skeleton.actions().end(); ++actionIt) {
      if (actionIt->second.action == oldName) {
        actionIt->second.action = newName;
      }
    }

    for (auto pointIt = skeleton.begin(); pointIt != skeleton.end(); pointIt++) {
      if (pointIt->isStopped()) {
        if (pointIt->hasStopAction(oldName)) {
          pointIt->removeStopAction(oldName);
          pointIt->addStopAction(newName);
        }
      }
    }

    if (skeleton.hasStartAction(oldName)) {
      skeleton.removeStartAction(oldName);
      skeleton.addStartAction(newName);
    }
    if (skeleton.hasEndAction(oldName)) {
      skeleton.removeEndAction(oldName);
      skeleton.addEndAction(newName);
    }
  }

  // TODO: Rename references in auto modes
}

void ThunderAutoProjectState::moveActionBeforeOther(const std::string& actionName,
                                                    const std::string& otherActionName) {
  if (!m_actions.contains(actionName)) {
    throw LogicError::Construct("Action to move does not exist: {}", actionName);
  }
  if (!m_actions.contains(otherActionName)) {
    throw LogicError::Construct("Other action does not exist: {}", otherActionName);
  }

  if (actionName == otherActionName)
    return;

  auto it = std::find(m_actionsOrder.begin(), m_actionsOrder.end(), actionName);
  ThunderLibAssert(it != m_actionsOrder.end(), "Action to move not found in actions order");
  m_actionsOrder.erase(it);

  it = std::find(m_actionsOrder.begin(), m_actionsOrder.end(), otherActionName);
  ThunderLibAssert(it != m_actionsOrder.end(), "Other action not found in actions order");
  m_actionsOrder.insert(it, actionName);
}

void ThunderAutoProjectState::moveActionAfterOther(const std::string& actionName,
                                                   const std::string& otherActionName) {
  if (!m_actions.contains(actionName)) {
    throw LogicError::Construct("Action to move does not exist: {}", actionName);
  }
  if (!m_actions.contains(otherActionName)) {
    throw LogicError::Construct("Other action does not exist: {}", otherActionName);
  }

  if (actionName == otherActionName)
    return;

  auto it = std::find(m_actionsOrder.begin(), m_actionsOrder.end(), actionName);
  ThunderLibAssert(it != m_actionsOrder.end(), "Action to move not found in actions order");
  m_actionsOrder.erase(it);

  it = std::find(m_actionsOrder.begin(), m_actionsOrder.end(), otherActionName);
  ThunderLibAssert(it != m_actionsOrder.end(), "Other action not found in actions order");
  m_actionsOrder.insert(std::next(it), actionName);
}

std::list<std::string> ThunderAutoProjectState::findActionRecursionPath(
    const std::string& targetActionName) const {
  if (!m_actions.contains(targetActionName)) {
    throw LogicError::Construct("Action to find recursive path to does not exist: {}", targetActionName);
  }

  using Path = std::list<std::string>;

  // BFS

  std::unordered_set<std::string> seen;
  std::queue<std::pair<std::string, Path>> nextActions;
  nextActions.emplace(targetActionName, Path{targetActionName});

  while (!nextActions.empty()) {
    auto [actionName, actionPath] = nextActions.front();
    nextActions.pop();

    const ThunderAutoAction& actionInfo = m_actions.at(actionName);
    if (actionInfo.type() == ThunderAutoActionType::COMMAND)
      continue;

    for (const std::string& subActionName : actionInfo.actionGroup()) {
      if (seen.contains(subActionName))
        continue;
      seen.insert(subActionName);

      Path subActionPath = actionPath;
      subActionPath.push_back(subActionName);

      if (subActionName == targetActionName) {
        return subActionPath;
      }

      nextActions.emplace(subActionName, subActionPath);
    }
  }

  return Path{};
}

void ThunderAutoProjectState::trajectorySelect(const std::string& trajectoryName) {
  if (!trajectoryName.empty() && !trajectories.contains(trajectoryName)) {
    throw LogicError::Construct("Cannot select trajectory: trajectory does not exist");
  }

  editorState.view = ThunderAutoEditorState::View::TRAJECTORY;

  ThunderAutoTrajectoryEditorState& trajectoryEditorState = editorState.trajectoryEditorState;
  trajectoryEditorState.currentTrajectoryName = trajectoryName;
  trajectoryEditorState.trajectorySelection = ThunderAutoTrajectoryEditorState::TrajectorySelection::NONE;
  trajectoryEditorState.selectionIndex = 0;
}

ThunderAutoTrajectorySkeleton& ThunderAutoProjectState::currentTrajectory() {
  if (editorState.view != ThunderAutoEditorState::View::TRAJECTORY) {
    throw LogicError::Construct("Cannot get current trajectory: not in trajectory editor view");
  }

  const std::string& trajectoryName = editorState.trajectoryEditorState.currentTrajectoryName;
  if (trajectoryName.empty()) {
    throw LogicError::Construct("Cannot get current trajectory: no trajectory selected");
  }

  auto skeletonIt = trajectories.find(trajectoryName);
  if (skeletonIt == trajectories.end()) {
    throw LogicError::Construct("Cannot get current trajectory: selected trajectory does not exist");
  }

  ThunderAutoTrajectorySkeleton& skeleton = skeletonIt->second;
  return skeleton;
}

const ThunderAutoTrajectorySkeleton& ThunderAutoProjectState::currentTrajectory() const {
  if (editorState.view != ThunderAutoEditorState::View::TRAJECTORY) {
    throw LogicError::Construct("Cannot get current trajectory: not in trajectory editor view");
  }

  const std::string& trajectoryName = editorState.trajectoryEditorState.currentTrajectoryName;
  if (trajectoryName.empty()) {
    throw LogicError::Construct("Cannot get current trajectory: no trajectory selected");
  }

  auto skeletonIt = trajectories.find(trajectoryName);
  if (skeletonIt == trajectories.end()) {
    throw LogicError::Construct("Cannot get current trajectory: selected trajectory does not exist");
  }

  const ThunderAutoTrajectorySkeleton& skeleton = skeletonIt->second;
  return skeleton;
}

ThunderAutoTrajectorySkeletonWaypoint& ThunderAutoProjectState::currentTrajectorySelectedWaypoint() {
  ThunderAutoTrajectorySkeleton& skeleton = currentTrajectory();

  if (editorState.trajectoryEditorState.trajectorySelection !=
      ThunderAutoTrajectoryEditorState::TrajectorySelection::WAYPOINT) {
    throw LogicError::Construct("Cannot get selected waypoint: no waypoint selected");
  }

  ThunderAutoTrajectorySkeletonWaypoint& point =
      skeleton.getPoint(editorState.trajectoryEditorState.selectionIndex);

  return point;
}

const ThunderAutoTrajectorySkeletonWaypoint& ThunderAutoProjectState::currentTrajectorySelectedWaypoint()
    const {
  const ThunderAutoTrajectorySkeleton& skeleton = currentTrajectory();

  if (editorState.trajectoryEditorState.trajectorySelection !=
      ThunderAutoTrajectoryEditorState::TrajectorySelection::WAYPOINT) {
    throw LogicError::Construct("Cannot get selected waypoint: no waypoint selected");
  }

  const ThunderAutoTrajectorySkeletonWaypoint& point =
      skeleton.getPoint(editorState.trajectoryEditorState.selectionIndex);

  return point;
}

void ThunderAutoProjectState::currentTrajectoryInsertWaypoint(size_t newWaypointIndex,
                                                              Point2d position,
                                                              CanonicalAngle outgoingHeading) {
  ThunderAutoTrajectorySkeleton& skeleton = currentTrajectory();

  ThunderAutoTrajectorySkeletonWaypoint newWaypoint(position, outgoingHeading, {});

  auto newWaypointIt = skeleton.insertPoint(newWaypointIndex, newWaypoint);
  if (newWaypointIt == skeleton.cend()) {
    throw LogicError::Construct("Failed to insert waypoint");
  }

  /**
   * It's a little weird when there are actions/rotations in the segment being split. Might be tricky to
   * fix in a good way... something to look into.
   */

  editorState.trajectoryEditorState.trajectorySelection =
      ThunderAutoTrajectoryEditorState::TrajectorySelection::WAYPOINT;
  editorState.trajectoryEditorState.selectionIndex = std::distance(skeleton.cbegin(), newWaypointIt);
}

void ThunderAutoProjectState::currentTrajectoryPrependWaypoint(Point2d position,
                                                               CanonicalAngle outgoingHeading) {
  ThunderAutoTrajectorySkeleton& skeleton = currentTrajectory();

  ThunderAutoTrajectorySkeletonWaypoint newWaypoint(position, outgoingHeading, {});

  skeleton.prependPoint(newWaypoint);

  editorState.trajectoryEditorState.trajectorySelection =
      ThunderAutoTrajectoryEditorState::TrajectorySelection::WAYPOINT;
  editorState.trajectoryEditorState.selectionIndex = 0;
}

void ThunderAutoProjectState::currentTrajectoryAppendWaypoint(Point2d position,
                                                              CanonicalAngle outgoingHeading) {
  ThunderAutoTrajectorySkeleton& skeleton = currentTrajectory();

  ThunderAutoTrajectorySkeletonWaypoint newWaypoint(position, outgoingHeading, {});

  skeleton.appendPoint(newWaypoint);

  editorState.trajectoryEditorState.trajectorySelection =
      ThunderAutoTrajectoryEditorState::TrajectorySelection::WAYPOINT;
  editorState.trajectoryEditorState.selectionIndex = skeleton.numPoints() - 1;
}

void ThunderAutoProjectState::currentTrajectoryInsertRotation(ThunderAutoTrajectoryPosition position,
                                                              CanonicalAngle angle) {
  ThunderAutoTrajectorySkeleton& skeleton = currentTrajectory();

  ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryRotation>& rotations = skeleton.rotations();

  ThunderAutoTrajectoryRotation newRotation{.angle = angle};
  (void)rotations.add(position, newRotation);

  std::unique_ptr<ThunderAutoPartialOutputTrajectory> trajectoryPositionData =
      BuildThunderAutoPartialOutputTrajectory(skeleton, kPreviewOutputTrajectorySettings);

  skeleton.separateRotations(0.1_m, trajectoryPositionData.get());
}

void ThunderAutoProjectState::currentTrajectoryInsertAction(ThunderAutoTrajectoryPosition position,
                                                            const std::string& action) {
  ThunderAutoTrajectorySkeleton& skeleton = currentTrajectory();

  ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryAction>& actions = skeleton.actions();

  ThunderAutoTrajectoryAction newAction{.action = action};
  (void)actions.add(position, newAction);
}

bool ThunderAutoProjectState::currentTrajectoryDeleteSelectedItem() {
  ThunderAutoTrajectorySkeleton& skeleton = currentTrajectory();

  size_t numItems = 0;
  const size_t selectionIndex = editorState.trajectoryEditorState.selectionIndex;

  switch (editorState.trajectoryEditorState.trajectorySelection) {
    using enum ThunderAutoTrajectoryEditorState::TrajectorySelection;
    case NONE:
      return false;  // nothing selected, delete nothing
    case WAYPOINT:
      skeleton.removePoint(selectionIndex);  // Bounds checking is handled by skeleton.
      numItems = skeleton.numPoints();
      break;
    case ACTION: {
      ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryAction>& actions = skeleton.actions();
      auto actionIt = std::next(actions.begin(), selectionIndex);
      actions.remove(actionIt);
      numItems = actions.size();
      break;
    }
    case ROTATION: {
      ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryRotation>& rotations =
          skeleton.rotations();
      auto rotationIt = std::next(rotations.begin(), selectionIndex);
      rotations.remove(rotationIt);
      numItems = rotations.size();
      break;
    }
    default:
      ThunderLibUnreachable("Invalid trajectory selection type");
  }

  if (numItems == 0) {
    editorState.trajectoryEditorState.trajectorySelection =
        ThunderAutoTrajectoryEditorState::TrajectorySelection::NONE;
  } else if (selectionIndex >= numItems) {
    editorState.trajectoryEditorState.selectionIndex = numItems - 1;
  } else if (selectionIndex != 0) {
    editorState.trajectoryEditorState.selectionIndex--;
  }

  return true;
}

void ThunderAutoProjectState::currentTrajectoryToggleEditorLockedForSelectedItem() {
  ThunderAutoTrajectorySkeleton& skeleton = currentTrajectory();

  const size_t selectionIndex = editorState.trajectoryEditorState.selectionIndex;

  switch (editorState.trajectoryEditorState.trajectorySelection) {
    using enum ThunderAutoTrajectoryEditorState::TrajectorySelection;
    case NONE:
      return;
    case WAYPOINT: {
      ThunderAutoTrajectorySkeletonWaypoint& waypoint = skeleton.getPoint(selectionIndex);
      waypoint.setEditorLocked(!waypoint.isEditorLocked());
      break;
    }
    case ACTION: {
      ThunderAutoTrajectoryAction& action = skeleton.getAction(selectionIndex);
      action.editorLocked = !action.editorLocked;
      break;
    }
    case ROTATION: {
      ThunderAutoTrajectoryRotation& rotation = skeleton.getRotation(selectionIndex);
      rotation.editorLocked = !rotation.editorLocked;
      break;
    }
    default:
      ThunderLibUnreachable("Invalid trajectory selection type");
  }
}

bool ThunderAutoProjectState::currentTrajectoryIncrementSelectedItemIndex(bool forwards /*= true*/) {
  const ThunderAutoTrajectorySkeleton& skeleton = currentTrajectory();

  const size_t selectionIndex = editorState.trajectoryEditorState.selectionIndex;

  bool canIncrement = false;
  bool canDecrement = false;

  switch (editorState.trajectoryEditorState.trajectorySelection) {
    using enum ThunderAutoTrajectoryEditorState::TrajectorySelection;
    case NONE:
      return false;
    case WAYPOINT:
      canIncrement = (selectionIndex < skeleton.numPoints() - 1);
      canDecrement = selectionIndex > 0;
      break;
    case ACTION: {
      const ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryAction>& actions =
          skeleton.actions();
      canIncrement = (selectionIndex < actions.size() - 1);
      canDecrement = selectionIndex > 0;
      break;
    }
    case ROTATION: {
      const ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryRotation>& rotations =
          skeleton.rotations();
      canIncrement = (selectionIndex < rotations.size() - 1);
      canDecrement = selectionIndex > 0;
      break;
    }
    default:
      ThunderLibUnreachable("Invalid trajectory selection type");
  }

  if (forwards && canIncrement) {
    editorState.trajectoryEditorState.selectionIndex++;
    return true;
  } else if (!forwards && canDecrement) {
    editorState.trajectoryEditorState.selectionIndex--;
    return true;
  }

  return false;
}

void ThunderAutoProjectState::trajectoryUpdateLinkedWaypointsFromSelected() {
  // This can be done more efficiently if we store more information about links. Not a big deal right now
  // because trajectories aren't very long.

  const ThunderAutoTrajectorySkeletonWaypoint& selectedPoint = currentTrajectorySelectedWaypoint();
  if (!selectedPoint.isLinked())
    return;

  const std::string_view selectedPointLinkName = selectedPoint.linkName();

  for (auto& [name, skeleton] : trajectories) {
    const bool isCurrentTrajectory = (name == editorState.trajectoryEditorState.currentTrajectoryName);

    size_t pointIndex = 0;
    for (auto pointIt = skeleton.begin(); pointIt != skeleton.cend(); pointIt++, pointIndex++) {
      if (isCurrentTrajectory && pointIndex == editorState.trajectoryEditorState.selectionIndex)
        continue;

      ThunderAutoTrajectorySkeletonWaypoint& point = *pointIt;

      const bool arePointsLinked = point.linkName() == selectedPointLinkName;
      if (arePointsLinked) {
        point.setPosition(selectedPoint.position());
      }
    }
  }
}

void ThunderAutoProjectState::trajectoryUpdateSelectedWaypointFromLink() {
  ThunderAutoTrajectorySkeletonWaypoint& selectedPoint = currentTrajectorySelectedWaypoint();
  if (!selectedPoint.isLinked())
    return;

  const std::string_view selectedPointLinkName = selectedPoint.linkName();

  for (auto& [name, skeleton] : trajectories) {
    const bool isCurrentTrajectory = (name == editorState.trajectoryEditorState.currentTrajectoryName);

    size_t pointIndex = 0;
    for (auto pointIt = skeleton.begin(); pointIt != skeleton.cend(); pointIt++, pointIndex++) {
      if (isCurrentTrajectory && pointIndex == editorState.trajectoryEditorState.selectionIndex)
        continue;

      ThunderAutoTrajectorySkeletonWaypoint& point = *pointIt;

      const bool arePointsLinked = point.linkName() == selectedPointLinkName;
      if (arePointsLinked) {
        selectedPoint.setPosition(point.position());
        return;
      }
    }
  }
}

void ThunderAutoProjectState::trajectoryDelete(const std::string& trajectoryName) {
  auto trajectoryIt = trajectories.find(trajectoryName);
  if (trajectoryIt == trajectories.end()) {
    throw LogicError::Construct("Cannot delete trajectory: trajectory does not exist");
  }

  trajectories.erase(trajectoryIt);

  // Deselect trajectory if selected.

  const bool isInTrajectoryMode = editorState.view == ThunderAutoEditorState::View::TRAJECTORY;
  ThunderAutoTrajectoryEditorState& trajectoryEditorState = editorState.trajectoryEditorState;

  if (isInTrajectoryMode && trajectoryName == trajectoryEditorState.currentTrajectoryName) {
    trajectoryEditorState.currentTrajectoryName = "";
    trajectoryEditorState.trajectorySelection = ThunderAutoTrajectoryEditorState::TrajectorySelection::NONE;
    trajectoryEditorState.selectionIndex = 0;
  }

  // TODO: Remove references in auto modes
}

void ThunderAutoProjectState::trajectoryRename(const std::string& oldTrajectoryName,
                                               const std::string& newTrajectoryName) {
  auto oldTrajectoryIt = trajectories.find(oldTrajectoryName);
  if (oldTrajectoryIt == trajectories.end()) {
    throw LogicError::Construct("Cannot rename trajectory: original trajectory does not exist");
  }

  auto nodeHandle = trajectories.extract(oldTrajectoryIt);
  nodeHandle.key() = newTrajectoryName;
  trajectories.insert(std::move(nodeHandle));

  // Show the new trajectory in the editor.

  if (editorState.view == ThunderAutoEditorState::View::TRAJECTORY &&
      editorState.trajectoryEditorState.currentTrajectoryName == oldTrajectoryName) {
    editorState.trajectoryEditorState.currentTrajectoryName = newTrajectoryName;
  } else {
    editorState.trajectoryEditorState.currentTrajectoryName = "";
  }

  // TODO: Update references in auto modes
}

void ThunderAutoProjectState::trajectoryDuplicate(const std::string& oldTrajectoryName,
                                                  const std::string& newTrajectoryName) {
  auto oldTrajectoryIt = trajectories.find(oldTrajectoryName);
  if (oldTrajectoryIt == trajectories.end()) {
    throw LogicError::Construct("Cannot duplicate trajectory: original trajectory does not exist");
  }

  trajectories.emplace(newTrajectoryName, oldTrajectoryIt->second);

  trajectorySelect(newTrajectoryName);
}

void ThunderAutoProjectState::trajectoryReverseDirection(const std::string& trajectoryName) {
  auto trajectoryIt = trajectories.find(trajectoryName);
  if (trajectoryIt == trajectories.end()) {
    throw LogicError::Construct("Cannot reverse trajectory: trajectory does not exist");
  }

  ThunderAutoTrajectorySkeleton& skeleton = trajectoryIt->second;
  skeleton.reverseDirection();
}

static void to_json(wpi::json& json, const ThunderAutoProjectState& state) noexcept {
  json = wpi::json{
      {"trajectories", state.trajectories},    {"auto_modes", state.autoModes},
      {"actions_order", state.actionsOrder},   {"actions", state.actions},
      {"waypoint_links", state.waypointLinks}, {"editor_state", state.editorState},
  };
}

ThunderAutoProject::ThunderAutoProject(const ThunderAutoProjectSettings& settings,
                                       const ThunderAutoProjectState& state) noexcept
    : m_settings(settings), m_state(state) {}

ThunderAutoProject::ThunderAutoProject(const std::filesystem::path& path,
                                       const ThunderAutoProjectVersion& version,
                                       const wpi::json& json)
    : m_settings(path, json.at("settings"), version), m_state(json.at("state"), version) {}

void ThunderAutoProject::save() const {
  SaveThunderAutoProject(m_settings, m_state);
}

std::unique_ptr<ThunderAutoProject> LoadThunderAutoProject(const std::filesystem::path& path,
                                                           ThunderAutoProjectVersion* outVersion) {
  if (path.empty() || !std::filesystem::exists(path))
    throw RuntimeError::Construct("LoadThunderAutoProject: File not found: {}", path.string());

  std::ifstream file(path);
  if (!file.is_open())
    throw RuntimeError::Construct("LoadThunderAutoProject: Failed to open file: {}", path.string());

  std::unique_ptr<ThunderAutoProject> project = nullptr;
  bool invalidContents = false;

  try {
    wpi::json json;
    file >> json;

    ThunderAutoProjectVersion version;
    if (json.contains("version")) {
      json.at("version").get_to(version);
    } else {
      /*
       * Version information was added in 2026, legacy versions are lumped
       * together as 2025.
       */
      version = ThunderAutoProjectVersion{2025, 0};
    }
    if (outVersion) {
      *outVersion = version;
    }

    if (version.major > THUNDERAUTO_PROJECT_VERSION_MAJOR) {
      throw RuntimeError::Construct(
          "ThunderAutoProject: Project version {}.{} is too new (current version is {}.{})", version.major,
          version.minor, THUNDERAUTO_PROJECT_VERSION_MAJOR, THUNDERAUTO_PROJECT_VERSION_MINOR);
    }

    project = std::make_unique<ThunderAutoProject>(path, version, json);

  } catch (const wpi::json::exception& e) {
    ThunderLibLogger::Error("LoadThunderAutoProject: JSON parsing error: {}", e.what());
    invalidContents = true;

  } catch (const std::exception& e) {
    ThunderLibLogger::Error("LoadThunderAutoProject: Exception occurred while loading project: {}", e.what());
    invalidContents = true;

  } catch (...) {
    ThunderLibLogger::Error("LoadThunderAutoProject: Unknown exception occurred while loading project.");
    invalidContents = true;
  }

  if (invalidContents || !project) {
    throw RuntimeError::Construct("ThunderAutoProject: Invalid contents in file: {}", path.string());
  }

  return project;
}

void SaveThunderAutoProject(const ThunderAutoProjectSettings& settings,
                            const ThunderAutoProjectState& state) {
  std::ofstream ofs;
  ofs.exceptions(std::ofstream::failbit | std::ofstream::badbit);

  try {
    ofs.open(settings.projectPath, std::ios::out | std::ios::trunc);

    wpi::json json = wpi::json{
        {"version", kThunderAutoProjectVersionCurrent},
        {"settings", settings},
        {"state", state},
    };

    ofs << json.dump(2);

  } catch (const std::ofstream::failure& e) {
    throw RuntimeError::Construct("ThunderAutoProject: File operation failed: {}", e.what());
  }
  // json construction exceptions are handled by the caller
}

std::vector<std::filesystem::path> DiscoverThunderAutoProjects(const std::filesystem::path& path) noexcept {
  std::vector<std::filesystem::path> projects;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(path)) {
    if (entry.is_regular_file() && entry.path().extension() == ".thunderauto") {
      projects.push_back(entry.path());
    }
  }
  return projects;
}

}  // namespace thunder::core
