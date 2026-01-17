#include <ThunderLibCore/Auto/ThunderAutoProject.hpp>

#include <ThunderLibCore/Logger.hpp>
#include <ThunderLibCore/Error.hpp>

#include <fmt/ranges.h>
#include <fstream>
#include <queue>
#include <numeric>

namespace thunder::core {

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
      ThunderLibCoreUnreachable("Invalid action type");
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
      ThunderLibCoreUnreachable("Invalid action type");
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
      ThunderLibCoreUnreachable("Invalid action type");
  }
}

ThunderAutoProjectVersion CurrentThunderAutoProjectVersion() noexcept {
  return {THUNDERAUTO_PROJECT_VERSION_MAJOR, THUNDERAUTO_PROJECT_VERSION_MINOR};
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
      ThunderLibCoreUnreachable("Invalid action type");
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
    ThunderLibCoreLogger::Warn("Cannot add group action '{}' because action is already in group", actionName);
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
    ThunderLibCoreLogger::Warn("Cannot swap last group action with next");
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
    ThunderLibCoreLogger::Warn("Cannot swap first group action '{}' with previous", actionName);
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

  ThunderLibCoreAssert(version.major <= THUNDERAUTO_PROJECT_VERSION_MAJOR, "Version too new");

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
      trajectoryEndBehaviorLinks(other.trajectoryEndBehaviorLinks),
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
  trajectoryEndBehaviorLinks = other.trajectoryEndBehaviorLinks;
  editorState = other.editorState;
  return *this;
}

void ThunderAutoProjectState::fromJson(const wpi::json& json, const ThunderAutoProjectVersion& version) {
  if (version.major < 2026) {
    // Maintain compatibility with older versions (update state to new format).
    fromJsonPre2026Version(json);
    return;
  }

  ThunderLibCoreAssert(version.major <= THUNDERAUTO_PROJECT_VERSION_MAJOR, "Version too new");

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
  if (json.contains("trajectory_end_behavior_links")) {
    json.at("trajectory_end_behavior_links").get_to(trajectoryEndBehaviorLinks);
  }

  json.at("editor_state").get_to(editorState);

  validateActionsAndTrajectories();
  validateWaypointLinks();
  validateTrajectoryEndBehaviorLinks();
}

void ThunderAutoProjectState::validateActionsAndTrajectories() {
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
      if (pointIt->isStopped() && pointIt->hasStopAction()) {
        const std::string& stopAction = pointIt->stopAction();
        if (!m_actions.contains(stopAction)) {
          Point2d pos = pointIt->position();
          throw RuntimeError::Construct(
              "Trajectory '{}' contains reference to non-existent stop action '{}' at waypoint ({}, {})",
              name, stopAction, pos.x(), pos.y());
        }
      }
    }

    if (skeleton.hasStartAction()) {
      const std::string& startActionName = skeleton.startAction();
      if (!m_actions.contains(startActionName)) {
        throw RuntimeError::Construct("Trajectory '{}' contains reference to non-existent start action '{}'",
                                      name, startActionName);
      }
    }

    if (skeleton.hasEndAction()) {
      const std::string& endActionName = skeleton.endAction();
      if (!m_actions.contains(endActionName)) {
        throw RuntimeError::Construct("Trajectory '{}' contains reference to non-existent end action '{}'",
                                      name, endActionName);
      }
    }
  }

  // Validate references in auto modes

  for (auto& [name, autoMode] : autoModes) {
    using namespace std::placeholders;

    const auto& steps = autoMode.steps;
    std::for_each(
        steps.begin(), steps.end(),
        std::bind(&ThunderAutoProjectState::validateActionsAndTrajectoriesInAutoModeStep, this, _1));
  }
}

void ThunderAutoProjectState::validateActionsAndTrajectoriesInAutoModeStep(
    const std::unique_ptr<ThunderAutoModeStep>& step) {
  switch (step->type()) {
    using enum ThunderAutoModeStepType;
    case ACTION: {
      const auto actionStep = reinterpret_cast<const ThunderAutoModeActionStep*>(step.get());
      const std::string& actionName = actionStep->actionName;
      // Action steps with no action name just do nothing, so ignore them.
      if (!actionName.empty() && !m_actions.contains(actionName)) {
        throw RuntimeError::Construct("Auto mode action step contains reference to non-existent action '{}'",
                                      actionStep->actionName);
      }
      break;
    }
    case TRAJECTORY: {
      const auto trajectoryStep = reinterpret_cast<const ThunderAutoModeTrajectoryStep*>(step.get());
      const std::string& trajectoryName = trajectoryStep->trajectoryName;
      // Trajectory steps with no trajectory name just do nothing, so ignore them.
      if (!trajectoryName.empty() && !trajectories.contains(trajectoryName)) {
        throw RuntimeError::Construct(
            "Auto mode trajectory step contains reference to non-existent trajectory '{}'",
            trajectoryStep->trajectoryName);
      }
      break;
    }
    case BRANCH_BOOL: {
      const auto branchStep = reinterpret_cast<const ThunderAutoModeBoolBranchStep*>(step.get());

      using namespace std::placeholders;

      const auto& trueSteps = branchStep->trueBranch;
      const auto& elseSteps = branchStep->elseBranch;
      std::for_each(
          trueSteps.begin(), trueSteps.end(),
          std::bind(&ThunderAutoProjectState::validateActionsAndTrajectoriesInAutoModeStep, this, _1));
      std::for_each(
          elseSteps.begin(), elseSteps.end(),
          std::bind(&ThunderAutoProjectState::validateActionsAndTrajectoriesInAutoModeStep, this, _1));
      break;
    }
    case BRANCH_SWITCH: {
      const auto branchStep = reinterpret_cast<const ThunderAutoModeSwitchBranchStep*>(step.get());

      using namespace std::placeholders;

      for (const auto& [caseID, caseSteps] : branchStep->caseBranches) {
        std::for_each(
            caseSteps.begin(), caseSteps.end(),
            std::bind(&ThunderAutoProjectState::validateActionsAndTrajectoriesInAutoModeStep, this, _1));
      }
      const auto& defaultSteps = branchStep->defaultBranch;
      std::for_each(
          defaultSteps.begin(), defaultSteps.end(),
          std::bind(&ThunderAutoProjectState::validateActionsAndTrajectoriesInAutoModeStep, this, _1));
      break;
    }
    default:
      ThunderLibCoreUnreachable("Invalid auto mode step type");
  }
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

void ThunderAutoProjectState::validateTrajectoryEndBehaviorLinks() {
  for (const auto& [name, skeleton] : trajectories) {
    const std::string& startBehaviorLinkName = skeleton.startBehaviorLinkName();
    if (!startBehaviorLinkName.empty() && !trajectoryEndBehaviorLinks.contains(startBehaviorLinkName)) {
      throw RuntimeError::Construct(
          "Trajectory '{}' contains reference to non-existent trajectory start behavior link '{}'", name,
          startBehaviorLinkName);
    }

    const std::string& endBehaviorLinkName = skeleton.endBehaviorLinkName();
    if (!endBehaviorLinkName.empty() && !trajectoryEndBehaviorLinks.contains(endBehaviorLinkName)) {
      throw RuntimeError::Construct(
          "Trajectory '{}' contains reference to non-existent trajectory end behavior link '{}'", name,
          endBehaviorLinkName);
    }
  }
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
    ThunderLibCoreAssert(it != m_actionsOrder.end(), "Action to remove not found in actions order");
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
      if (pointIt->isStopped() && (pointIt->stopAction() == actionToRemoveName)) {
        pointIt->clearStopAction();
      }
    }

    if (skeleton.startAction() == actionToRemoveName) {
      skeleton.clearStartAction();
    }
    if (skeleton.endAction() == actionToRemoveName) {
      skeleton.clearEndAction();
    }
  }

  // Remove references in auto modes

  for (auto& [name, autoMode] : autoModes) {
    using namespace std::placeholders;

    auto& steps = autoMode.steps;
    std::for_each(
        steps.begin(), steps.end(),
        std::bind(&ThunderAutoProjectState::renameActionsInAutoModeStep, this, _1, actionToRemoveName, ""));
  }
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
    ThunderLibCoreAssert(it != m_actionsOrder.end(), "Old action name not found in actions order");
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
      if (pointIt->isStopped() && (pointIt->stopAction() == oldName)) {
        pointIt->setStopAction(newName);
      }
    }

    if (skeleton.startAction() == oldName) {
      skeleton.setStartAction(newName);
    }
    if (skeleton.endAction() == oldName) {
      skeleton.setEndAction(newName);
    }
  }

  // Rename references in auto modes

  for (auto& [name, autoMode] : autoModes) {
    using namespace std::placeholders;

    auto& steps = autoMode.steps;
    std::for_each(
        steps.begin(), steps.end(),
        std::bind(&ThunderAutoProjectState::renameActionsInAutoModeStep, this, _1, oldName, newName));
  }
}

void ThunderAutoProjectState::renameActionsInAutoModeStep(std::unique_ptr<ThunderAutoModeStep>& step,
                                                          const std::string& oldName,
                                                          const std::string& newName) {
  switch (step->type()) {
    using enum ThunderAutoModeStepType;
    case UNKNOWN:
      break;
    case ACTION: {
      auto actionStep = reinterpret_cast<ThunderAutoModeActionStep*>(step.get());
      if (actionStep->actionName == oldName) {
        actionStep->actionName = newName;
      }
      break;
    }
    case TRAJECTORY:
      break;
    case BRANCH_BOOL: {
      auto branchStep = reinterpret_cast<ThunderAutoModeBoolBranchStep*>(step.get());

      using namespace std::placeholders;

      auto& trueSteps = branchStep->trueBranch;
      auto& elseSteps = branchStep->elseBranch;
      std::for_each(
          trueSteps.begin(), trueSteps.end(),
          std::bind(&ThunderAutoProjectState::renameActionsInAutoModeStep, this, _1, oldName, newName));
      std::for_each(
          elseSteps.begin(), elseSteps.end(),
          std::bind(&ThunderAutoProjectState::renameActionsInAutoModeStep, this, _1, oldName, newName));

      break;
    }
    case BRANCH_SWITCH: {
      auto branchStep = reinterpret_cast<ThunderAutoModeSwitchBranchStep*>(step.get());

      using namespace std::placeholders;

      for (auto& [caseID, caseSteps] : branchStep->caseBranches) {
        std::for_each(
            caseSteps.begin(), caseSteps.end(),
            std::bind(&ThunderAutoProjectState::renameActionsInAutoModeStep, this, _1, oldName, newName));
      }
      auto& defaultSteps = branchStep->defaultBranch;
      std::for_each(
          defaultSteps.begin(), defaultSteps.end(),
          std::bind(&ThunderAutoProjectState::renameActionsInAutoModeStep, this, _1, oldName, newName));

      break;
    }
    default:
      ThunderLibCoreUnreachable("Invalid auto mode step type");
  }
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
  ThunderLibCoreAssert(it != m_actionsOrder.end(), "Action to move not found in actions order");
  m_actionsOrder.erase(it);

  it = std::find(m_actionsOrder.begin(), m_actionsOrder.end(), otherActionName);
  ThunderLibCoreAssert(it != m_actionsOrder.end(), "Other action not found in actions order");
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
  ThunderLibCoreAssert(it != m_actionsOrder.end(), "Action to move not found in actions order");
  m_actionsOrder.erase(it);

  it = std::find(m_actionsOrder.begin(), m_actionsOrder.end(), otherActionName);
  ThunderLibCoreAssert(it != m_actionsOrder.end(), "Other action not found in actions order");
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
    throw LogicError::Construct("Cannot select trajectory: trajectory '{}' does not exist", trajectoryName);
  }

  editorState.view = ThunderAutoEditorState::View::TRAJECTORY;

  ThunderAutoTrajectoryEditorState& trajectoryEditorState = editorState.trajectoryEditorState;
  trajectoryEditorState = ThunderAutoTrajectoryEditorState{};
  trajectoryEditorState.currentTrajectoryName = trajectoryName;
}

const std::string& ThunderAutoProjectState::currentTrajectoryName() const {
  if (editorState.view != ThunderAutoEditorState::View::TRAJECTORY) {
    throw LogicError::Construct("Cannot get current trajectory: not in trajectory editor view");
  }

  const std::string& trajectoryName = editorState.trajectoryEditorState.currentTrajectoryName;
  if (trajectoryName.empty()) {
    throw LogicError::Construct("Cannot get current trajectory: no trajectory selected");
  }
  return trajectoryName;
}

ThunderAutoTrajectorySkeleton& ThunderAutoProjectState::currentTrajectory() {
  const std::string& trajectoryName = currentTrajectoryName();

  auto skeletonIt = trajectories.find(trajectoryName);
  if (skeletonIt == trajectories.end()) {
    throw LogicError::Construct("Cannot get current trajectory: selected trajectory does not exist");
  }

  ThunderAutoTrajectorySkeleton& skeleton = skeletonIt->second;
  return skeleton;
}

const ThunderAutoTrajectorySkeleton& ThunderAutoProjectState::currentTrajectory() const {
  const std::string& trajectoryName = currentTrajectoryName();

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
      currentTrajectoryUpdateEndBehaviorFromLinks();
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
      ThunderLibCoreUnreachable("Invalid trajectory selection type");
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
      ThunderLibCoreUnreachable("Invalid trajectory selection type");
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
      ThunderLibCoreUnreachable("Invalid trajectory selection type");
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

void ThunderAutoProjectState::trajectoryUpdateAllLinkedWaypointPositionsFromSelectedWaypoint() {
  // This can be done more efficiently if we store more information about links. Not a big deal right now
  // because trajectories aren't very long.

  ThunderAutoTrajectorySkeleton& selectedSkeleton = currentTrajectory();

  if (editorState.trajectoryEditorState.trajectorySelection !=
      ThunderAutoTrajectoryEditorState::TrajectorySelection::WAYPOINT) {
    throw LogicError::Construct("Cannot get selected waypoint: no waypoint selected");
  }

  ThunderAutoTrajectorySkeletonWaypoint& selectedPoint =
      selectedSkeleton.getPoint(editorState.trajectoryEditorState.selectionIndex);
  const size_t selectedPointIndex = editorState.trajectoryEditorState.selectionIndex;
  const Point2d selectedPointPosition = selectedPoint.position();

  // Waypoint links

  if (selectedPoint.isLinked()) {
    const std::string_view selectedPointLinkName = selectedPoint.linkName();

    for (auto& [name, skeleton] : trajectories) {
      const bool isCurrentTrajectory = (name == editorState.trajectoryEditorState.currentTrajectoryName);

      size_t pointIndex = 0;
      for (auto pointIt = skeleton.begin(); pointIt != skeleton.cend(); pointIt++, pointIndex++) {
        if (isCurrentTrajectory && pointIndex == selectedPointIndex)
          continue;

        ThunderAutoTrajectorySkeletonWaypoint& point = *pointIt;

        const bool arePointsLinked = point.linkName() == selectedPointLinkName;
        if (arePointsLinked) {
          point.setPosition(selectedPointPosition);
        }
      }
    }
  }

  ThunderLibCoreAssert(selectedSkeleton.numPoints() >= 2);

  // Trajectory start/end behavior links

  if (selectedPointIndex == 0 && selectedSkeleton.hasStartBehaviorLink()) {
    const std::string_view linkName = selectedSkeleton.startBehaviorLinkName();

    for (auto& [name, skeleton] : trajectories) {
      const bool isCurrentTrajectory = (name == editorState.trajectoryEditorState.currentTrajectoryName);

      if (!isCurrentTrajectory && skeleton.startBehaviorLinkName() == linkName) {
        skeleton.front().setPosition(selectedPointPosition);
      }
      if (skeleton.endBehaviorLinkName() == linkName) {
        skeleton.back().setPosition(selectedPointPosition);
      }
    }
  } else if (selectedPointIndex == selectedSkeleton.numPoints() - 1 &&
             selectedSkeleton.hasEndBehaviorLink()) {
    const std::string_view linkName = selectedSkeleton.endBehaviorLinkName();

    for (auto& [name, skeleton] : trajectories) {
      const bool isCurrentTrajectory = (name == editorState.trajectoryEditorState.currentTrajectoryName);

      if (skeleton.startBehaviorLinkName() == linkName) {
        skeleton.front().setPosition(selectedPointPosition);
      }
      if (!isCurrentTrajectory && skeleton.endBehaviorLinkName() == linkName) {
        skeleton.back().setPosition(selectedPointPosition);
      }
    }
  }
}

void ThunderAutoProjectState::trajectoryUpdateAllLinkedTrajectoryEndBehaviorsFromCurrentTrajectoryEndBehavior(
    bool updateFromStartBehavior,
    bool updateFromEndBehavior) {
  const ThunderAutoTrajectorySkeleton& selectedSkeleton = currentTrajectory();

  if (updateFromStartBehavior && selectedSkeleton.hasStartBehaviorLink()) {
    const std::string_view linkName = selectedSkeleton.startBehaviorLinkName();
    const Point2d linkPosition = selectedSkeleton.front().position();
    const CanonicalAngle linkRotation = selectedSkeleton.startRotation();

    for (auto& [name, skeleton] : trajectories) {
      const bool isCurrentTrajectory = (name == editorState.trajectoryEditorState.currentTrajectoryName);

      if (!isCurrentTrajectory && skeleton.startBehaviorLinkName() == linkName) {
        skeleton.front().setPosition(linkPosition);
        skeleton.setStartRotation(linkRotation);
      }
      if (skeleton.endBehaviorLinkName() == linkName) {
        skeleton.back().setPosition(linkPosition);
        skeleton.setEndRotation(linkRotation);
      }
    }
  }

  if (updateFromEndBehavior && selectedSkeleton.hasEndBehaviorLink()) {
    const std::string_view linkName = selectedSkeleton.endBehaviorLinkName();
    const Point2d linkPosition = selectedSkeleton.back().position();
    const CanonicalAngle linkRotation = selectedSkeleton.endRotation();

    for (auto& [name, skeleton] : trajectories) {
      const bool isCurrentTrajectory = (name == editorState.trajectoryEditorState.currentTrajectoryName);

      if (skeleton.startBehaviorLinkName() == linkName) {
        skeleton.front().setPosition(linkPosition);
        skeleton.setStartRotation(linkRotation);
      }
      if (!isCurrentTrajectory && skeleton.endBehaviorLinkName() == linkName) {
        skeleton.back().setPosition(linkPosition);
        skeleton.setEndRotation(linkRotation);
      }
    }
  }
}

bool ThunderAutoProjectState::currentTrajectoryUpdateSelectedWaypointFromLink() {
  ThunderAutoTrajectorySkeletonWaypoint& selectedPoint = currentTrajectorySelectedWaypoint();
  if (!selectedPoint.isLinked())
    return false;

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
        return true;
      }
    }
  }
  return false;
}

bool ThunderAutoProjectState::currentTrajectoryUpdateEndBehaviorFromLinks() {
  ThunderAutoTrajectorySkeleton& selectedSkeleton = currentTrajectory();

  bool updated = false;

  // Update front
  if (selectedSkeleton.hasStartBehaviorLink()) {
    const std::string_view linkName = selectedSkeleton.startBehaviorLinkName();

    for (const auto& [name, skeleton] : trajectories) {
      const bool isCurrentTrajectory = (name == editorState.trajectoryEditorState.currentTrajectoryName);

      if (!isCurrentTrajectory && skeleton.startBehaviorLinkName() == linkName) {
        selectedSkeleton.front().setPosition(skeleton.front().position());
        selectedSkeleton.setStartRotation(skeleton.startRotation());
        updated = true;
        break;
      } else if (skeleton.endBehaviorLinkName() == linkName) {
        selectedSkeleton.front().setPosition(skeleton.back().position());
        selectedSkeleton.setStartRotation(skeleton.endRotation());
        updated = true;
        break;
      }
    }
  }

  // Update back
  if (selectedSkeleton.hasEndBehaviorLink()) {
    const std::string_view linkName = selectedSkeleton.endBehaviorLinkName();

    for (const auto& [name, skeleton] : trajectories) {
      const bool isCurrentTrajectory = (name == editorState.trajectoryEditorState.currentTrajectoryName);

      if (skeleton.startBehaviorLinkName() == linkName) {
        selectedSkeleton.back().setPosition(skeleton.front().position());
        selectedSkeleton.setEndRotation(skeleton.startRotation());
        updated = true;
        break;
      } else if (!isCurrentTrajectory && skeleton.endBehaviorLinkName() == linkName) {
        selectedSkeleton.back().setPosition(skeleton.back().position());
        selectedSkeleton.setEndRotation(skeleton.endRotation());
        updated = true;
        break;
      }
    }
  }

  return updated;
}

void ThunderAutoProjectState::trajectoryDelete(const std::string& trajectoryName) {
  auto trajectoryIt = trajectories.find(trajectoryName);
  if (trajectoryIt == trajectories.end()) {
    throw LogicError::Construct("Cannot delete trajectory: trajectory '{}' does not exist", trajectoryName);
  }

  trajectories.erase(trajectoryIt);

  // Deselect trajectory if selected.

  const bool isInTrajectoryState = editorState.view == ThunderAutoEditorState::View::TRAJECTORY;
  ThunderAutoTrajectoryEditorState& trajectoryEditorState = editorState.trajectoryEditorState;

  if (isInTrajectoryState && trajectoryName == trajectoryEditorState.currentTrajectoryName) {
    trajectoryEditorState = ThunderAutoTrajectoryEditorState{};
  }

  // Remove references in auto modes

  for (auto& [name, autoMode] : autoModes) {
    using namespace std::placeholders;

    auto& steps = autoMode.steps;
    std::for_each(
        steps.begin(), steps.end(),
        std::bind(&ThunderAutoProjectState::renameTrajectoryInAutoModeStep, this, _1, trajectoryName, ""));
  }
}

void ThunderAutoProjectState::trajectoryRename(const std::string& oldName, const std::string& newName) {
  auto oldTrajectoryIt = trajectories.find(oldName);
  if (oldTrajectoryIt == trajectories.end()) {
    throw LogicError::Construct("Cannot rename trajectory: trajectory '{}' does not exist", oldName);
  }

  auto nodeHandle = trajectories.extract(oldTrajectoryIt);
  nodeHandle.key() = newName;
  trajectories.insert(std::move(nodeHandle));

  // Show the new trajectory in the editor.

  if (editorState.view == ThunderAutoEditorState::View::TRAJECTORY &&
      editorState.trajectoryEditorState.currentTrajectoryName == oldName) {
    editorState.trajectoryEditorState.currentTrajectoryName = newName;
  }

  // Rename references in auto modes

  for (auto& [name, autoMode] : autoModes) {
    using namespace std::placeholders;

    auto& steps = autoMode.steps;
    std::for_each(
        steps.begin(), steps.end(),
        std::bind(&ThunderAutoProjectState::renameTrajectoryInAutoModeStep, this, _1, oldName, newName));
  }
}

void ThunderAutoProjectState::renameTrajectoryInAutoModeStep(std::unique_ptr<ThunderAutoModeStep>& step,
                                                             const std::string& oldName,
                                                             const std::string& newName) {
  switch (step->type()) {
    using enum ThunderAutoModeStepType;
    case UNKNOWN:
      break;
    case ACTION:
      break;
    case TRAJECTORY: {
      auto trajectoryStep = reinterpret_cast<ThunderAutoModeTrajectoryStep*>(step.get());
      if (trajectoryStep->trajectoryName == oldName) {
        trajectoryStep->trajectoryName = newName;
      }
      break;
    }
    case BRANCH_BOOL: {
      auto branchStep = reinterpret_cast<ThunderAutoModeBoolBranchStep*>(step.get());

      using namespace std::placeholders;

      auto& trueSteps = branchStep->trueBranch;
      auto& elseSteps = branchStep->elseBranch;
      std::for_each(
          trueSteps.begin(), trueSteps.end(),
          std::bind(&ThunderAutoProjectState::renameTrajectoryInAutoModeStep, this, _1, oldName, newName));
      std::for_each(
          elseSteps.begin(), elseSteps.end(),
          std::bind(&ThunderAutoProjectState::renameTrajectoryInAutoModeStep, this, _1, oldName, newName));

      break;
    }
    case BRANCH_SWITCH: {
      auto branchStep = reinterpret_cast<ThunderAutoModeSwitchBranchStep*>(step.get());

      using namespace std::placeholders;

      for (auto& [caseID, caseSteps] : branchStep->caseBranches) {
        std::for_each(
            caseSteps.begin(), caseSteps.end(),
            std::bind(&ThunderAutoProjectState::renameTrajectoryInAutoModeStep, this, _1, oldName, newName));
      }
      auto& defaultSteps = branchStep->defaultBranch;
      std::for_each(
          defaultSteps.begin(), defaultSteps.end(),
          std::bind(&ThunderAutoProjectState::renameTrajectoryInAutoModeStep, this, _1, oldName, newName));

      break;
    }
    default:
      ThunderLibCoreUnreachable("Invalid auto mode step type");
  }
}

void ThunderAutoProjectState::trajectoryDuplicate(const std::string& oldTrajectoryName,
                                                  const std::string& newTrajectoryName) {
  auto oldTrajectoryIt = trajectories.find(oldTrajectoryName);
  if (oldTrajectoryIt == trajectories.end()) {
    throw LogicError::Construct("Cannot duplicate trajectory: trajectory '{}' does not exist",
                                oldTrajectoryName);
  }

  trajectories.emplace(newTrajectoryName, oldTrajectoryIt->second);

  trajectorySelect(newTrajectoryName);
}

void ThunderAutoProjectState::trajectoryReverseDirection(const std::string& trajectoryName) {
  auto trajectoryIt = trajectories.find(trajectoryName);
  if (trajectoryIt == trajectories.end()) {
    throw LogicError::Construct("Cannot reverse trajectory: trajectory '{}' does not exist", trajectoryName);
  }

  ThunderAutoTrajectorySkeleton& skeleton = trajectoryIt->second;
  skeleton.reverseDirection();
}

void ThunderAutoProjectState::autoModeSelect(const std::string& autoModeName) {
  if (!autoModeName.empty() && !autoModes.contains(autoModeName)) {
    throw LogicError::Construct("Cannot select auto mode: auto mode '{}' does not exist", autoModeName);
  }

  editorState.view = ThunderAutoEditorState::View::AUTO_MODE;

  ThunderAutoModeEditorState& autoModeEditorState = editorState.autoModeEditorState;
  autoModeEditorState = ThunderAutoModeEditorState{};
  autoModeEditorState.currentAutoModeName = autoModeName;
}

ThunderAutoMode& ThunderAutoProjectState::currentAutoMode() {
  if (editorState.view != ThunderAutoEditorState::View::AUTO_MODE) {
    throw LogicError::Construct("Cannot get current auto mode: not in auto mode editor view");
  }

  const std::string& autoModeName = editorState.autoModeEditorState.currentAutoModeName;
  if (autoModeName.empty()) {
    throw LogicError::Construct("Cannot get current auto mode: no auto mode selected");
  }

  auto autoModeIt = autoModes.find(autoModeName);
  if (autoModeIt == autoModes.end()) {
    throw LogicError::Construct("Cannot get current auto mode: selected auto mode does not exist");
  }

  ThunderAutoMode& autoMode = autoModeIt->second;
  return autoMode;
}

const ThunderAutoMode& ThunderAutoProjectState::currentAutoMode() const {
  if (editorState.view != ThunderAutoEditorState::View::AUTO_MODE) {
    throw LogicError::Construct("Cannot get current auto mode: not in auto mode editor view");
  }

  const std::string& autoModeName = editorState.autoModeEditorState.currentAutoModeName;
  if (autoModeName.empty()) {
    throw LogicError::Construct("Cannot get current auto mode: no auto mode selected");
  }

  auto autoModeIt = autoModes.find(autoModeName);
  if (autoModeIt == autoModes.end()) {
    throw LogicError::Construct("Cannot get current auto mode: selected auto mode does not exist");
  }

  const ThunderAutoMode& autoMode = autoModeIt->second;
  return autoMode;
}

bool ThunderAutoProjectState::currentAutoModeMoveStepBeforeOther(
    const ThunderAutoModeStepPath& stepPath,
    const ThunderAutoModeStepPath& otherStepPath) {
  if (stepPath == otherStepPath)
    return false;

  if (otherStepPath.hasParentPath(stepPath)) {
    ThunderLibCoreLogger::Warn("Cannot move a step under one of its own children");
    return false;
  }
  if (otherStepPath.stepIndex() != 0) {
    bool isInSameDirectory = (stepPath.directoryPath() == otherStepPath.directoryPath());
    if (isInSameDirectory && stepPath.stepIndex() == otherStepPath.stepIndex() - 1) {
      ThunderLibCoreLogger::Warn("Won't move step because it's already in the target position");
      return false;
    }
  }

  ThunderAutoMode& autoMode = currentAutoMode();

  // Move the step.
  auto [stepDirectory, stepIt] = autoMode.findStepAtPath(stepPath);
  auto [otherStepDirectory, otherStepIt] = autoMode.findStepAtPath(otherStepPath);

  std::unique_ptr<ThunderAutoModeStep> step = (*stepIt)->clone();
  stepDirectory->erase(stepIt);
  otherStepDirectory->insert(otherStepIt, std::move(step));

  ThunderAutoModeStepPath newStepPath = otherStepPath;
  newStepPath.updateWithRemovalOfStep(stepPath);

  // Select the moved step.
  auto& selectedStepPath = editorState.autoModeEditorState.selectedStepPath;
  selectedStepPath = newStepPath;

  return true;
}

bool ThunderAutoProjectState::currentAutoModeMoveStepAfterOther(
    const ThunderAutoModeStepPath& stepPath,
    const ThunderAutoModeStepPath& otherStepPath) {
  if (stepPath == otherStepPath)
    return false;

  if (otherStepPath.hasParentPath(stepPath)) {
    ThunderLibCoreLogger::Warn("Cannot move a step under one of its own children");
    return false;
  }
  bool isInSameDirectory = (stepPath.directoryPath() == otherStepPath.directoryPath());
  if (isInSameDirectory && stepPath.stepIndex() == otherStepPath.stepIndex() + 1) {
    ThunderLibCoreLogger::Warn("Won't move step because it's already in the target position");
    return false;
  }

  ThunderAutoMode& autoMode = currentAutoMode();

  // Move the step.
  auto [stepDirectory, stepIt] = autoMode.findStepAtPath(stepPath);
  auto [otherStepDirectory, otherStepIt] = autoMode.findStepAtPath(otherStepPath);

  std::unique_ptr<ThunderAutoModeStep> step = (*stepIt)->clone();
  stepDirectory->erase(stepIt);
  otherStepDirectory->insert(std::next(otherStepIt), std::move(step));

  ThunderAutoModeStepPath newStepPath = otherStepPath;
  newStepPath.updateWithRemovalOfStep(stepPath);
  newStepPath.setStepIndex(newStepPath.stepIndex() + 1);

  // Select the moved step.
  auto& selectedStepPath = editorState.autoModeEditorState.selectedStepPath;
  selectedStepPath = newStepPath;

  return true;
}

bool ThunderAutoProjectState::currentAutoModeMoveStepIntoDirectory(
    const ThunderAutoModeStepPath& stepPath,
    const ThunderAutoModeStepDirectoryPath& directoryPath) {
  ThunderAutoMode& autoMode = currentAutoMode();

  if (directoryPath.hasParentPath(stepPath)) {
    ThunderLibCoreLogger::Warn("Cannot move a step under one of its own children");
    return false;
  }
  if (stepPath.directoryPath() == directoryPath) {
    ThunderLibCoreLogger::Warn("Won't move step because it's already in the target directory");
    return false;
  }

  // Move the step.
  auto [stepDirectory, stepIt] = autoMode.findStepAtPath(stepPath);
  auto& newStepDirectory = autoMode.findStepDirectoryAtPath(directoryPath);

  std::unique_ptr<ThunderAutoModeStep> step = (*stepIt)->clone();
  stepDirectory->erase(stepIt);
  newStepDirectory.push_back(std::move(step));

  ThunderAutoModeStepPath newStepPath = directoryPath.step(newStepDirectory.size() - 1);
  newStepPath.updateWithRemovalOfStep(stepPath);

  // Select the moved step.
  auto& selectedStepPath = editorState.autoModeEditorState.selectedStepPath;
  selectedStepPath = newStepPath;

  return true;
}

void ThunderAutoProjectState::currentAutoModeInsertStepBeforeOther(
    const ThunderAutoModeStepPath& stepPath,
    std::unique_ptr<ThunderAutoModeStep> step) {
  ThunderAutoMode& autoMode = currentAutoMode();

  // Insert the step.
  auto [stepDirectory, stepIt] = autoMode.findStepAtPath(stepPath);
  stepDirectory->insert(stepIt, std::move(step));

  // Select the inserted step.
  auto& selectedStepPath = editorState.autoModeEditorState.selectedStepPath;
  selectedStepPath = stepPath;
}

void ThunderAutoProjectState::currentAutoModeInsertStepAfterOther(const ThunderAutoModeStepPath& stepPath,
                                                                  std::unique_ptr<ThunderAutoModeStep> step) {
  ThunderAutoMode& autoMode = currentAutoMode();

  // Insert the step.
  auto [stepDirectory, stepIt] = autoMode.findStepAtPath(stepPath);
  stepDirectory->insert(std::next(stepIt), std::move(step));

  // Select the inserted step.
  auto& selectedStepPath = editorState.autoModeEditorState.selectedStepPath;
  selectedStepPath = stepPath;
  selectedStepPath->setStepIndex(selectedStepPath->stepIndex() + 1);
}

void ThunderAutoProjectState::currentAutoModeInsertStepInDirectory(
    const ThunderAutoModeStepDirectoryPath& directoryPath,
    std::unique_ptr<ThunderAutoModeStep> step) {
  ThunderAutoMode& autoMode = currentAutoMode();

  // Insert the step.
  auto& stepDirectory = autoMode.findStepDirectoryAtPath(directoryPath);
  stepDirectory.push_back(std::move(step));

  // Select the inserted step.
  auto& selectedStepPath = editorState.autoModeEditorState.selectedStepPath;
  selectedStepPath = directoryPath.step(stepDirectory.size() - 1);
}

void ThunderAutoProjectState::currentAutoModeDeleteStep(const ThunderAutoModeStepPath& stepPath) {
  ThunderAutoMode& autoMode = currentAutoMode();

  auto& selectedStepPath = editorState.autoModeEditorState.selectedStepPath;
  if (selectedStepPath.has_value()) {
    if (selectedStepPath == stepPath || selectedStepPath->hasParentPath(stepPath)) {
      selectedStepPath = std::nullopt;
    } else {
      selectedStepPath->updateWithRemovalOfStep(stepPath);
    }
  }

  auto [stepsList, stepIt] = autoMode.findStepAtPath(stepPath);
  stepsList->erase(stepIt);
}

void ThunderAutoProjectState::autoModeDelete(const std::string& autoModeName) {
  auto autoModeIt = autoModes.find(autoModeName);
  if (autoModeIt == autoModes.end()) {
    throw LogicError::Construct("Cannot delete auto mode: auto mode '{}' does not exist", autoModeName);
  }

  autoModes.erase(autoModeIt);

  // Deselect auto mode if selected.

  const bool isInAutoModeState = editorState.view == ThunderAutoEditorState::View::AUTO_MODE;
  ThunderAutoModeEditorState& autoModeEditorState = editorState.autoModeEditorState;

  if (isInAutoModeState && autoModeName == autoModeEditorState.currentAutoModeName) {
    autoModeEditorState = ThunderAutoModeEditorState{};
  }
}

void ThunderAutoProjectState::autoModeRename(const std::string& oldName, const std::string& newName) {
  auto oldAutoModeIt = autoModes.find(oldName);
  if (oldAutoModeIt == autoModes.end()) {
    throw LogicError::Construct("Cannot rename auto mode: auto mode '{}' does not exist", oldName);
  }

  auto nodeHandle = autoModes.extract(oldAutoModeIt);
  nodeHandle.key() = newName;
  autoModes.insert(std::move(nodeHandle));

  // Show the new auto mode in the editor.

  if (editorState.view == ThunderAutoEditorState::View::AUTO_MODE &&
      editorState.autoModeEditorState.currentAutoModeName == oldName) {
    editorState.autoModeEditorState.currentAutoModeName = newName;
  }
}

void ThunderAutoProjectState::autoModeDuplicate(const std::string& oldAutoModeName,
                                                const std::string& newAutoModeName) {
  auto oldAutoModeIt = autoModes.find(oldAutoModeName);
  if (oldAutoModeIt == autoModes.end()) {
    throw LogicError::Construct("Cannot duplicate auto mode: auto mode '{}' does not exist", oldAutoModeName);
  }

  autoModes.emplace(newAutoModeName, oldAutoModeIt->second);

  autoModeSelect(newAutoModeName);
}

static void to_json(wpi::json& json, const ThunderAutoProjectState& state) noexcept {
  json = wpi::json{
      {"trajectories", state.trajectories},
      {"auto_modes", state.autoModes},
      {"actions_order", state.actionsOrder},
      {"actions", state.actions},
      {"waypoint_links", state.waypointLinks},
      {"trajectory_end_behavior_links", state.trajectoryEndBehaviorLinks},
      {"editor_state", state.editorState},
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
    ThunderLibCoreLogger::Error("LoadThunderAutoProject: JSON parsing error: {}", e.what());
    invalidContents = true;

  } catch (const std::exception& e) {
    ThunderLibCoreLogger::Error("LoadThunderAutoProject: Exception occurred while loading project: {}",
                                e.what());
    invalidContents = true;

  } catch (...) {
    ThunderLibCoreLogger::Error("LoadThunderAutoProject: Unknown exception occurred while loading project.");
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
        {"version", CurrentThunderAutoProjectVersion()},
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

template <typename T>
concept Serializable = std::integral<T> || std::floating_point<T>;

/**
 * A simple data output stream that writes to a byte buffer, used for serializing the project state.
 */
class DataOutputStream {
  std::vector<uint8_t>& m_buffer;

 public:
  explicit DataOutputStream(std::vector<uint8_t>& buffer) : m_buffer(buffer) {}

  /**
   * Writes a value of type T to the buffer.
   *
   * @param value The value to write.
   * @return Sum of bytes written.
   */
  template <Serializable T>
  size_t write(const T& value) {
    const uint8_t* rawData = reinterpret_cast<const uint8_t*>(&value);
    size_t sum = 0;
    for (size_t i = 0; i < sizeof(T); i++) {
      uint8_t byte = rawData[i];
      m_buffer.push_back(byte);
      sum += static_cast<size_t>(byte);
    }
    return sum;
  }

  size_t write(const std::string& str) {
    // Length (1 byte)
    uint8_t length = static_cast<uint8_t>(std::min(str.size(), size_t(UINT8_MAX)));
    *this << length;
    // String data (0-255 bytes)
    size_t sum = length;
    for (size_t i = 0; i < length; i++) {
      uint8_t byte = static_cast<uint8_t>(str[i]);
      m_buffer.push_back(byte);
      sum += static_cast<size_t>(byte);
    }
    return sum;
  }

  template <typename T>
  size_t operator<<(const T& value) {
    return write(value);
  }
};

/**
 * A simple data input stream that reads from a byte buffer, used for deserializing the project state.
 */
class DataInputStream {
  std::span<const uint8_t> m_buffer;
  size_t m_offset = 0;

 public:
  explicit DataInputStream(std::span<const uint8_t> buffer) : m_buffer(buffer) {}

  template <Serializable T>
  size_t read(T& value) {
    if (m_offset + sizeof(T) > m_buffer.size()) {
      throw RuntimeError::Construct(
          "DeserializeThunderAutoProjectStateFromTransmission: Buffer too small to read value of size {}",
          sizeof(T));
    }
    unsigned char* rawData = reinterpret_cast<unsigned char*>(&value);
    size_t sum = 0;
    for (size_t i = 0; i < sizeof(T); i++) {
      uint8_t byte = m_buffer[m_offset++];
      rawData[i] = byte;
      sum += static_cast<size_t>(byte);
    }
    return sum;
  }

  size_t read(std::string& str) {
    // Length (1 byte)
    uint8_t length = 0;
    *this >> length;
    if (m_offset + length > m_buffer.size()) {
      throw RuntimeError::Construct(
          "DeserializeThunderAutoProjectStateFromTransmission: Buffer too small to read string of length {}",
          length);
    }
    // String data (0-255 bytes)
    str = std::string(reinterpret_cast<const char*>(m_buffer.data() + m_offset), length);
    m_offset += length;

    size_t sum = std::accumulate(str.begin(), str.end(), size_t(0));
    return length + sum;
  }

  template <typename T>
  size_t operator>>(T& value) {
    return read(value);
  }
};

enum : uint8_t {
  kSectionHeader = 0x01,
  kSectionTrajectories = 0x02,
  kSectionAutoModes = 0x03,
  kSectionActions = 0x04,
};

static size_t SerializeHeader(const ThunderAutoProjectState& state, DataOutputStream& stream) {
  size_t sum = 0;

  sum += stream << uint8_t(kSectionHeader);

  sum += stream << uint16_t(THUNDERAUTO_PROJECT_VERSION_MAJOR);
  sum += stream << uint16_t(THUNDERAUTO_PROJECT_VERSION_MINOR);

  sum += stream << uint8_t(sum & 0xFF);

  return sum;
}

static size_t DeserializeHeader(DataInputStream& stream, ThunderAutoProjectState& state) {
  size_t sum = 0;

  uint8_t sectionByte;
  sum += stream >> sectionByte;
  if (sectionByte != kSectionHeader) {
    throw RuntimeError::Construct(
        "DeserializeThunderAutoProjectStateFromTransmission: Expected header section (0x01), got 0x{:02X}",
        sectionByte);
  }

  uint16_t major, minor;
  sum += stream >> major;
  sum += stream >> minor;

  if (major > THUNDERAUTO_PROJECT_VERSION_MAJOR) {
    throw RuntimeError::Construct(
        "DeserializeThunderAutoProjectStateFromTransmission: Project version {}.{} is too new (current "
        "version is {}.{})",
        major, minor, THUNDERAUTO_PROJECT_VERSION_MAJOR, THUNDERAUTO_PROJECT_VERSION_MINOR);
  }

  uint8_t readSum;
  stream >> readSum;

  if (readSum != (sum & 0xFF)) {
    throw RuntimeError::Construct(
        "DeserializeThunderAutoProjectStateFromTransmission: Header checksum mismatch: expected 0x{:02X}, "
        "got 0x{:02X}",
        sum & 0xFF, readSum);
  }

  return sum + readSum;
}

static size_t SerializeTrajectoryPoint(const ThunderAutoTrajectorySkeletonWaypoint& point,
                                       DataOutputStream& stream) {
  size_t sum = 0;

  const auto& [x, y] = point.position().data;
  sum += stream << x.value();
  sum += stream << y.value();

  const ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles& headings = point.headings();
  const double incomingHeading = headings.incomingAngle().radians().value();
  sum += stream << incomingHeading;
  const double outgoingHeading = headings.outgoingAngle().radians().value();
  sum += stream << outgoingHeading;

  const ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights& headingWeights = point.headingWeights();
  const double incomingHeadingWeight = headingWeights.incomingWeight();
  sum += stream << incomingHeadingWeight;
  const double outgoingHeadingWeight = headingWeights.outgoingWeight();
  sum += stream << outgoingHeadingWeight;

  const uint8_t hasMaxVelocityOverride = point.hasMaxVelocityOverride() ? 1 : 0;
  sum += stream << hasMaxVelocityOverride;

  if (hasMaxVelocityOverride) {
    const double maxVelocityOverride = point.maxVelocityOverride().value().value();
    sum += stream << maxVelocityOverride;

    if (maxVelocityOverride == 0.0) {
      const double stopRotation = point.stopRotation().radians().value();
      sum += stream << stopRotation;

      sum += stream << point.stopAction();
    }
  }

  return sum;
}

static size_t DeserializeTrajectoryPoint(DataInputStream& stream,
                                         ThunderAutoTrajectorySkeletonWaypoint& point) {
  size_t sum = 0;

  {
    double xValue, yValue;
    sum += stream >> xValue;
    sum += stream >> yValue;

    units::meter_t x{xValue};
    units::meter_t y{yValue};
    Point2d position{x, y};

    point.setPosition(position);
  }

  {
    double incomingHeadingValue, outgoingHeadingValue;
    sum += stream >> incomingHeadingValue;
    sum += stream >> outgoingHeadingValue;

    CanonicalAngle incomingAngle(units::radian_t{incomingHeadingValue});
    CanonicalAngle outgoingAngle(units::radian_t{outgoingHeadingValue});
    ThunderAutoTrajectorySkeletonWaypoint::HeadingAngles headings(incomingAngle, outgoingAngle);

    point.setHeadings(headings);
  }

  {
    double incomingHeadingWeight, outgoingHeadingWeight;
    sum += stream >> incomingHeadingWeight;
    sum += stream >> outgoingHeadingWeight;

    ThunderAutoTrajectorySkeletonWaypoint::HeadingWeights headingWeights(incomingHeadingWeight,
                                                                         outgoingHeadingWeight);

    point.setHeadingWeights(headingWeights);
  }

  uint8_t hasMaxVelocityOverride;
  sum += stream >> hasMaxVelocityOverride;

  if (hasMaxVelocityOverride) {
    double maxVelocityOverrideValue;
    sum += stream >> maxVelocityOverrideValue;

    units::meters_per_second_t maxVelocityOverride{maxVelocityOverrideValue};
    point.setMaxVelocityOverride(maxVelocityOverride);

    if (maxVelocityOverrideValue == 0.0) {
      double stopRotationValue;
      sum += stream >> stopRotationValue;

      units::radian_t stopRotation{stopRotationValue};
      point.setStopRotation(CanonicalAngle(stopRotation));

      std::string stopActionName;
      sum += stream >> stopActionName;
      point.setStopAction(stopActionName);
    }
  }

  return sum;
}

static size_t SerializeTrajectoryAction(const ThunderAutoTrajectoryPosition& position,
                                        const ThunderAutoTrajectoryAction& action,
                                        DataOutputStream& stream) {
  size_t sum = 0;

  sum += stream << position.position;
  sum += stream << action.action;

  return sum;
}

static size_t DeserializeTrajectoryAction(DataInputStream& stream,
                                          ThunderAutoTrajectoryPosition& position,
                                          ThunderAutoTrajectoryAction& action) {
  size_t sum = 0;

  sum += stream >> position.position;
  sum += stream >> action.action;

  return sum;
}

static size_t SerializeTrajectoryRotation(const ThunderAutoTrajectoryPosition& position,
                                          const ThunderAutoTrajectoryRotation& rotation,
                                          DataOutputStream& stream) {
  size_t sum = 0;

  sum += stream << position.position;
  sum += stream << rotation.angle.radians().value();

  return sum;
}

static size_t DeserializeTrajectoryRotation(DataInputStream& stream,
                                            ThunderAutoTrajectoryPosition& position,
                                            ThunderAutoTrajectoryRotation& rotation) {
  size_t sum = 0;

  sum += stream >> position.position;

  double angleValue;
  sum += stream >> angleValue;
  rotation.angle = CanonicalAngle(units::radian_t{angleValue});

  return sum;
}

static size_t SerializeTrajectory(const std::string& name,
                                  const ThunderAutoTrajectorySkeleton& skeleton,
                                  DataOutputStream& stream) {
  size_t sum = 0;

  sum += stream << name;

  {
    const std::list<ThunderAutoTrajectorySkeletonWaypoint>& points = skeleton.points();

    const uint8_t numPoints = static_cast<uint8_t>(std::min(points.size(), size_t(UINT8_MAX)));
    sum += stream << numPoints;

    size_t pointIndex = 0;
    for (auto pointIt = points.begin(); pointIt != points.end() && pointIndex < numPoints;
         ++pointIt, ++pointIndex) {
      sum += SerializeTrajectoryPoint(*pointIt, stream);
    }
  }

  {
    const ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryAction>& actions = skeleton.actions();

    const uint8_t numActions = static_cast<uint8_t>(std::min(actions.size(), size_t(UINT8_MAX)));
    sum += stream << numActions;

    size_t actionIndex = 0;
    for (auto actionIt = actions.begin(); actionIt != actions.end() && actionIndex < numActions;
         ++actionIt, ++actionIndex) {
      const auto& [position, action] = *actionIt;
      sum += SerializeTrajectoryAction(position, action, stream);
    }
  }

  {
    const ThunderAutoPositionedTrajectoryItemList<ThunderAutoTrajectoryRotation>& rotations =
        skeleton.rotations();

    const uint8_t numRotations = static_cast<uint8_t>(std::min(rotations.size(), size_t(UINT8_MAX)));
    sum += stream << numRotations;

    size_t rotationIndex = 0;
    for (auto rotationIt = rotations.begin(); rotationIt != rotations.end() && rotationIndex < numRotations;
         ++rotationIt, ++rotationIndex) {
      const auto& [position, rotation] = *rotationIt;
      sum += SerializeTrajectoryRotation(position, rotation, stream);
    }
  }

  {
    const double startRotation = skeleton.startRotation().radians().value();
    sum += stream << startRotation;

    const double endRotation = skeleton.endRotation().radians().value();
    sum += stream << endRotation;
  }

  {
    const ThunderAutoTrajectorySkeletonSettings& settings = skeleton.settings();

    const double maxLinearVelocity = settings.maxLinearVelocity.value();
    sum += stream << maxLinearVelocity;

    const double maxLinearAcceleration = settings.maxLinearAcceleration.value();
    sum += stream << maxLinearAcceleration;

    const double maxCentripetalAcceleration = settings.maxCentripetalAcceleration.value();
    sum += stream << maxCentripetalAcceleration;

    const double maxAngularVelocity = settings.maxAngularVelocity.value();
    sum += stream << maxAngularVelocity;

    const double maxAngularAcceleration = settings.maxAngularAcceleration.value();
    sum += stream << maxAngularAcceleration;
  }

  sum += stream << skeleton.startAction();
  sum += stream << skeleton.endAction();

  sum += stream << uint64_t(sum);  // Trajectory hash is simple checksum for now.

  return sum;
}

static size_t DeserializeTrajectory(DataInputStream& stream,
                                    ThunderAutoProjectState& state,
                                    std::unordered_map<std::string, uint64_t>& hashes) {
  size_t sum = 0;

  std::string name;
  sum += stream >> name;

  ThunderAutoTrajectorySkeleton skeleton;

  {
    uint8_t numPoints;
    sum += stream >> numPoints;

    for (size_t pointIndex = 0; pointIndex < static_cast<size_t>(numPoints); ++pointIndex) {
      ThunderAutoTrajectorySkeletonWaypoint point;
      sum += DeserializeTrajectoryPoint(stream, point);
      skeleton.appendPoint(point);
    }
  }

  {
    uint8_t numActions;
    sum += stream >> numActions;

    for (size_t actionIndex = 0; actionIndex < static_cast<size_t>(numActions); ++actionIndex) {
      ThunderAutoTrajectoryPosition position;
      ThunderAutoTrajectoryAction action;
      sum += DeserializeTrajectoryAction(stream, position, action);
      skeleton.actions().add(position, action);
    }
  }

  {
    uint8_t numRotations;
    sum += stream >> numRotations;

    for (size_t rotationIndex = 0; rotationIndex < static_cast<size_t>(numRotations); ++rotationIndex) {
      ThunderAutoTrajectoryPosition position;
      ThunderAutoTrajectoryRotation rotation;
      sum += DeserializeTrajectoryRotation(stream, position, rotation);
      skeleton.rotations().add(position, rotation);
    }
  }

  {
    double startRotationValue;
    sum += stream >> startRotationValue;
    skeleton.setStartRotation(CanonicalAngle(units::radian_t{startRotationValue}));

    double endRotationValue;
    sum += stream >> endRotationValue;
    skeleton.setEndRotation(CanonicalAngle(units::radian_t{endRotationValue}));
  }

  {
    ThunderAutoTrajectorySkeletonSettings settings;

    double maxLinearVelocityValue;
    sum += stream >> maxLinearVelocityValue;
    settings.maxLinearVelocity = units::meters_per_second_t{maxLinearVelocityValue};

    double maxLinearAccelerationValue;
    sum += stream >> maxLinearAccelerationValue;
    settings.maxLinearAcceleration = units::meters_per_second_squared_t{maxLinearAccelerationValue};

    double maxCentripetalAccelerationValue;
    sum += stream >> maxCentripetalAccelerationValue;
    settings.maxCentripetalAcceleration = units::meters_per_second_squared_t{maxCentripetalAccelerationValue};

    double maxAngularVelocityValue;
    sum += stream >> maxAngularVelocityValue;
    settings.maxAngularVelocity = units::radians_per_second_t{maxAngularVelocityValue};

    double maxAngularAccelerationValue;
    sum += stream >> maxAngularAccelerationValue;
    settings.maxAngularAcceleration = units::radians_per_second_squared_t{maxAngularAccelerationValue};

    skeleton.setSettings(settings);
  }

  {
    std::string startActionName;
    sum += stream >> startActionName;
    skeleton.setStartAction(startActionName);
  }

  {
    std::string endActionName;
    sum += stream >> endActionName;
    skeleton.setEndAction(endActionName);
  }

  uint64_t trajectoryHash;
  sum += stream >> trajectoryHash;

  state.trajectories.emplace(name, skeleton);
  hashes.emplace(name, trajectoryHash);

  return sum;
}

static size_t SerializeTrajectories(const ThunderAutoProjectState& state, DataOutputStream& stream) {
  size_t sum = 0;

  sum += stream << uint8_t(kSectionTrajectories);

  const uint8_t numTrajectories =
      static_cast<uint8_t>(std::min(state.trajectories.size(), size_t(UINT8_MAX)));
  sum += stream << numTrajectories;

  size_t trajectoryIndex = 0;
  for (auto trajectoryIt = state.trajectories.begin();
       trajectoryIt != state.trajectories.end() && trajectoryIndex < numTrajectories;
       ++trajectoryIt, ++trajectoryIndex) {
    const auto& [name, skeleton] = *trajectoryIt;
    sum += SerializeTrajectory(name, skeleton, stream);
  }

  sum += stream << uint8_t(sum & 0xFF);

  return sum;
}

static size_t DeserializeTrajectories(DataInputStream& stream,
                                      ThunderAutoProjectState& state,
                                      std::unordered_map<std::string, uint64_t>& hashes) {
  size_t sum = 0;

  uint8_t sectionByte;
  sum += stream >> sectionByte;
  if (sectionByte != kSectionTrajectories) {
    throw RuntimeError::Construct(
        "DeserializeThunderAutoProjectStateFromTransmission: Expected trajectories section (0x02), got "
        "0x{:02X}",
        sectionByte);
  }

  uint8_t numTrajectories;
  sum += stream >> numTrajectories;

  for (size_t trajectoryIndex = 0; trajectoryIndex < static_cast<size_t>(numTrajectories);
       ++trajectoryIndex) {
    sum += DeserializeTrajectory(stream, state, hashes);
  }

  uint8_t readSum;
  stream >> readSum;

  if (readSum != (sum & 0xFF)) {
    throw RuntimeError::Construct(
        "DeserializeThunderAutoProjectStateFromTransmission: Trajectories section checksum mismatch: "
        "expected 0x{:02X}, got 0x{:02X}",
        sum & 0xFF, readSum);
  }

  return sum + readSum;
}

static size_t SerializeAutoModeStep(const ThunderAutoModeStep* autoModeStep, DataOutputStream& stream);
static size_t DeserializeAutoModeStep(DataInputStream& stream, std::unique_ptr<ThunderAutoModeStep>& step);

static size_t SerializeAutoModeActionStep(const ThunderAutoModeActionStep* actionStep,
                                          DataOutputStream& stream) {
  size_t sum = 0;

  const std::string& actionName = actionStep->actionName;
  sum += stream << actionName;

  return sum;
}

static size_t DeserializeAutoModeActionStep(DataInputStream& stream, ThunderAutoModeActionStep* actionStep) {
  size_t sum = 0;

  std::string actionName;
  sum += stream >> actionName;
  actionStep->actionName = actionName;

  return sum;
}

static size_t SerializeAutoModeTrajectoryStep(const ThunderAutoModeTrajectoryStep* trajectoryStep,
                                              DataOutputStream& stream) {
  size_t sum = 0;

  const std::string& trajectoryName = trajectoryStep->trajectoryName;
  sum += stream << trajectoryName;

  return sum;
}

static size_t DeserializeAutoModeTrajectoryStep(DataInputStream& stream,
                                                ThunderAutoModeTrajectoryStep* trajectoryStep) {
  size_t sum = 0;

  std::string trajectoryName;
  sum += stream >> trajectoryName;
  trajectoryStep->trajectoryName = trajectoryName;

  return sum;
}

static size_t SerializeAutoModeBoolBranchStep(const ThunderAutoModeBoolBranchStep* boolBranchStep,
                                              DataOutputStream& stream) {
  size_t sum = 0;

  const std::string& conditionName = boolBranchStep->conditionName;
  sum += stream << conditionName;

  {
    const uint8_t numTrueSteps =
        static_cast<uint8_t>(std::min(boolBranchStep->trueBranch.size(), size_t(UINT8_MAX)));

    sum += stream << numTrueSteps;

    size_t stepIndex = 0;
    for (auto stepIt = boolBranchStep->trueBranch.begin();
         stepIt != boolBranchStep->trueBranch.end() && stepIndex < numTrueSteps; ++stepIt, ++stepIndex) {
      sum += SerializeAutoModeStep(stepIt->get(), stream);
    }
  }

  {
    const uint8_t numElseSteps =
        static_cast<uint8_t>(std::min(boolBranchStep->elseBranch.size(), size_t(UINT8_MAX)));

    sum += stream << numElseSteps;

    size_t stepIndex = 0;
    for (auto stepIt = boolBranchStep->elseBranch.begin();
         stepIt != boolBranchStep->elseBranch.end() && stepIndex < numElseSteps; ++stepIt, ++stepIndex) {
      sum += SerializeAutoModeStep(stepIt->get(), stream);
    }
  }

  return sum;
}

static size_t DeserializeAutoModeBoolBranchStep(DataInputStream& stream,
                                                ThunderAutoModeBoolBranchStep* boolBranchStep) {
  size_t sum = 0;

  std::string conditionName;
  sum += stream >> conditionName;
  boolBranchStep->conditionName = conditionName;

  {
    uint8_t numTrueSteps;
    sum += stream >> numTrueSteps;

    for (size_t stepIndex = 0; stepIndex < static_cast<size_t>(numTrueSteps); ++stepIndex) {
      std::unique_ptr<ThunderAutoModeStep> step;
      sum += DeserializeAutoModeStep(stream, step);
      boolBranchStep->trueBranch.push_back(std::move(step));
    }
  }

  {
    uint8_t numElseSteps;
    sum += stream >> numElseSteps;

    for (size_t stepIndex = 0; stepIndex < static_cast<size_t>(numElseSteps); ++stepIndex) {
      std::unique_ptr<ThunderAutoModeStep> step;
      sum += DeserializeAutoModeStep(stream, step);
      boolBranchStep->elseBranch.push_back(std::move(step));
    }
  }

  return sum;
}

static size_t SerializeAutoModeSwitchBranchStep(const ThunderAutoModeSwitchBranchStep* switchBranchStep,
                                                DataOutputStream& stream) {
  size_t sum = 0;

  const std::string& conditionName = switchBranchStep->conditionName;
  sum += stream << conditionName;

  {
    const uint8_t numCases =
        static_cast<uint8_t>(std::min(switchBranchStep->caseBranches.size(), size_t(UINT8_MAX)));

    sum += stream << numCases;

    size_t caseIndex = 0;
    for (auto caseIt = switchBranchStep->caseBranches.begin();
         caseIt != switchBranchStep->caseBranches.end() && caseIndex < numCases; ++caseIt, ++caseIndex) {
      const auto& [caseValue, caseSteps] = *caseIt;
      sum += stream << caseValue;

      const uint8_t numCaseSteps = static_cast<uint8_t>(std::min(caseSteps.size(), size_t(UINT8_MAX)));

      sum += stream << numCaseSteps;

      size_t stepIndex = 0;
      for (auto stepIt = caseSteps.begin(); stepIt != caseSteps.end() && stepIndex < numCaseSteps;
           ++stepIt, ++stepIndex) {
        sum += SerializeAutoModeStep(stepIt->get(), stream);
      }
    }
  }

  {
    const uint8_t numDefaultSteps =
        static_cast<uint8_t>(std::min(switchBranchStep->defaultBranch.size(), size_t(UINT8_MAX)));

    sum += stream << numDefaultSteps;

    size_t stepIndex = 0;
    for (auto stepIt = switchBranchStep->defaultBranch.begin();
         stepIt != switchBranchStep->defaultBranch.end() && stepIndex < numDefaultSteps;
         ++stepIt, ++stepIndex) {
      sum += SerializeAutoModeStep(stepIt->get(), stream);
    }
  }

  return sum;
}

static size_t DeserializeAutoModeSwitchBranchStep(DataInputStream& stream,
                                                  ThunderAutoModeSwitchBranchStep* switchBranchStep) {
  size_t sum = 0;

  std::string conditionName;
  sum += stream >> conditionName;
  switchBranchStep->conditionName = conditionName;

  {
    uint8_t numCases;
    sum += stream >> numCases;

    for (size_t caseIndex = 0; caseIndex < static_cast<size_t>(numCases); ++caseIndex) {
      int caseValue;
      sum += stream >> caseValue;

      uint8_t numCaseSteps;
      sum += stream >> numCaseSteps;

      std::list<std::unique_ptr<ThunderAutoModeStep>> caseSteps;
      for (size_t stepIndex = 0; stepIndex < static_cast<size_t>(numCaseSteps); ++stepIndex) {
        std::unique_ptr<ThunderAutoModeStep> step;
        sum += DeserializeAutoModeStep(stream, step);
        caseSteps.push_back(std::move(step));
      }

      switchBranchStep->caseBranches.emplace(caseValue, std::move(caseSteps));
    }
  }

  {
    uint8_t numDefaultSteps;
    sum += stream >> numDefaultSteps;

    for (size_t stepIndex = 0; stepIndex < static_cast<size_t>(numDefaultSteps); ++stepIndex) {
      std::unique_ptr<ThunderAutoModeStep> step;
      sum += DeserializeAutoModeStep(stream, step);
      switchBranchStep->defaultBranch.push_back(std::move(step));
    }
  }

  return sum;
}

static size_t SerializeAutoModeStep(const ThunderAutoModeStep* autoModeStep, DataOutputStream& stream) {
  size_t sum = 0;

  const ThunderAutoModeStepType stepType = autoModeStep->type();
  sum += stream << static_cast<uint8_t>(stepType);

  switch (stepType) {
    using enum ThunderAutoModeStepType;
    case ACTION:
      sum += SerializeAutoModeActionStep(reinterpret_cast<const ThunderAutoModeActionStep*>(autoModeStep),
                                         stream);
      break;
    case TRAJECTORY:
      sum += SerializeAutoModeTrajectoryStep(
          reinterpret_cast<const ThunderAutoModeTrajectoryStep*>(autoModeStep), stream);
      break;
    case BRANCH_BOOL:
      sum += SerializeAutoModeBoolBranchStep(
          reinterpret_cast<const ThunderAutoModeBoolBranchStep*>(autoModeStep), stream);
      break;
    case BRANCH_SWITCH:
      sum += SerializeAutoModeSwitchBranchStep(
          reinterpret_cast<const ThunderAutoModeSwitchBranchStep*>(autoModeStep), stream);
      break;
    default:
      ThunderLibCoreUnreachable("Invalid auto mode step type");
  }

  return sum;
}

static size_t DeserializeAutoModeStep(DataInputStream& stream, std::unique_ptr<ThunderAutoModeStep>& step) {
  size_t sum = 0;

  uint8_t stepTypeByte;
  sum += stream >> stepTypeByte;

  using enum ThunderAutoModeStepType;
  ThunderAutoModeStepType stepType = static_cast<ThunderAutoModeStepType>(stepTypeByte);

  switch (stepType) {
    case ACTION: {
      auto actionStep = std::make_unique<ThunderAutoModeActionStep>();
      sum += DeserializeAutoModeActionStep(stream, actionStep.get());
      step = std::move(actionStep);
      break;
    }
    case TRAJECTORY: {
      auto trajectoryStep = std::make_unique<ThunderAutoModeTrajectoryStep>();
      sum += DeserializeAutoModeTrajectoryStep(stream, trajectoryStep.get());
      step = std::move(trajectoryStep);
      break;
    }
    case BRANCH_BOOL: {
      auto boolBranchStep = std::make_unique<ThunderAutoModeBoolBranchStep>();
      sum += DeserializeAutoModeBoolBranchStep(stream, boolBranchStep.get());
      step = std::move(boolBranchStep);
      break;
    }
    case BRANCH_SWITCH: {
      auto switchBranchStep = std::make_unique<ThunderAutoModeSwitchBranchStep>();
      sum += DeserializeAutoModeSwitchBranchStep(stream, switchBranchStep.get());
      step = std::move(switchBranchStep);
      break;
    }
    default:
      throw RuntimeError::Construct("DeserializeAutoModeStep: Invalid auto mode step type: 0x{:02X}",
                                    stepTypeByte);
  }

  return sum;
}

static size_t SerializeAutoMode(const std::string& name,
                                const ThunderAutoMode& autoMode,
                                DataOutputStream& stream) {
  size_t sum = 0;

  sum += stream << name;

  const uint8_t numSteps = static_cast<uint8_t>(std::min(autoMode.steps.size(), size_t(UINT8_MAX)));
  sum += stream << numSteps;

  size_t stepIndex = 0;
  for (auto stepIt = autoMode.steps.begin(); stepIt != autoMode.steps.end() && stepIndex < numSteps;
       ++stepIt, ++stepIndex) {
    sum += SerializeAutoModeStep(stepIt->get(), stream);
  }

  sum += stream << sum;  // Auto mode hash is simple checksum for now.

  return sum;
}

static size_t DeserializeAutoMode(DataInputStream& stream,
                                  ThunderAutoProjectState& state,
                                  std::unordered_map<std::string, uint64_t>& hashes) {
  size_t sum = 0;

  std::string name;
  sum += stream >> name;

  uint8_t numSteps;
  sum += stream >> numSteps;

  ThunderAutoMode autoMode;

  for (size_t stepIndex = 0; stepIndex < static_cast<size_t>(numSteps); ++stepIndex) {
    std::unique_ptr<ThunderAutoModeStep> step;
    sum += DeserializeAutoModeStep(stream, step);
    autoMode.steps.push_back(std::move(step));
  }

  uint64_t autoModeHash;
  sum += stream >> autoModeHash;

  state.autoModes.emplace(name, autoMode);
  hashes.emplace(name, autoModeHash);

  return sum;
}

static size_t SerializeAutoModes(const ThunderAutoProjectState& state, DataOutputStream& stream) {
  size_t sum = 0;

  sum += stream << uint8_t(kSectionAutoModes);

  const uint8_t numAutoModes = static_cast<uint8_t>(std::min(state.autoModes.size(), size_t(UINT8_MAX)));
  sum += stream << numAutoModes;

  size_t autoModeIndex = 0;
  for (auto autoModeIt = state.autoModes.begin();
       autoModeIt != state.autoModes.end() && autoModeIndex < numAutoModes; ++autoModeIt, ++autoModeIndex) {
    const auto& [name, autoMode] = *autoModeIt;
    sum += SerializeAutoMode(name, autoMode, stream);
  }

  sum += stream << uint8_t(sum & 0xFF);

  return sum;
}

static size_t DeserializeAutoModes(DataInputStream& stream,
                                   ThunderAutoProjectState& state,
                                   std::unordered_map<std::string, uint64_t>& hashes) {
  size_t sum = 0;

  uint8_t sectionByte;
  sum += stream >> sectionByte;
  if (sectionByte != kSectionAutoModes) {
    throw RuntimeError::Construct(
        "DeserializeThunderAutoProjectStateFromTransmission: Expected auto modes section (0x03), got "
        "0x{:02X}",
        sectionByte);
  }

  uint8_t numAutoModes;
  sum += stream >> numAutoModes;

  for (size_t autoModeIndex = 0; autoModeIndex < static_cast<size_t>(numAutoModes); ++autoModeIndex) {
    sum += DeserializeAutoMode(stream, state, hashes);
  }

  uint8_t readSum;
  stream >> readSum;

  if (readSum != (sum & 0xFF)) {
    throw RuntimeError::Construct(
        "DeserializeThunderAutoProjectStateFromTransmission: Auto modes section checksum mismatch: expected "
        "0x{:02X}, got 0x{:02X}",
        sum & 0xFF, readSum);
  }

  return sum + readSum;
}

static size_t SerializeAction(const ThunderAutoAction& action, DataOutputStream& stream) {
  size_t sum = 0;

  const ThunderAutoActionType type = action.type();
  sum += stream << static_cast<uint8_t>(type);

  using enum ThunderAutoActionType;
  if (type == SEQUENTIAL_ACTION_GROUP || type == CONCURRENT_ACTION_GROUP) {
    std::span<const std::string> actionGroup = action.actionGroup();

    const uint8_t numActions = static_cast<uint8_t>(std::min(actionGroup.size(), size_t(UINT8_MAX)));
    sum += stream << numActions;

    for (size_t actionIndex = 0; actionIndex < numActions; actionIndex++) {
      sum += stream << actionGroup[actionIndex];
    }
  } else if (type != COMMAND) {
    ThunderLibCoreUnreachable("Invalid action type");
  }

  return sum;
}

static size_t DeserializeAction(DataInputStream& stream, ThunderAutoAction& action) {
  size_t sum = 0;

  uint8_t typeByte;
  sum += stream >> typeByte;

  using enum ThunderAutoActionType;
  ThunderAutoActionType type = static_cast<ThunderAutoActionType>(typeByte);
  action.setType(type);

  if (type == SEQUENTIAL_ACTION_GROUP || type == CONCURRENT_ACTION_GROUP) {
    uint8_t numActions;
    sum += stream >> numActions;

    for (size_t actionIndex = 0; actionIndex < static_cast<size_t>(numActions); ++actionIndex) {
      std::string actionName;
      sum += stream >> actionName;
      action.addGroupAction(actionName);
    }
  } else if (type != COMMAND) {
    throw RuntimeError::Construct("DeserializeAction: Invalid action type: 0x{:02X}", typeByte);
  }

  return sum;
}

static size_t SerializeActions(const ThunderAutoProjectState& state, DataOutputStream& stream) {
  size_t sum = 0;

  sum += stream << uint8_t(kSectionActions);

  uint8_t numActions = static_cast<uint8_t>(std::min(state.actionsOrder.size(), size_t(UINT8_MAX)));
  sum += stream << numActions;

  for (size_t actionIndex = 0; actionIndex < numActions; actionIndex++) {
    const std::string& actionName = state.actionsOrder[actionIndex];
    sum += stream << actionName;

    const ThunderAutoAction& action = state.actions.at(actionName);
    sum += SerializeAction(action, stream);
  }

  sum += stream << sum;  // Actions hash is simple checksum for now.

  return sum;
}

static size_t DeserializeActions(DataInputStream& stream, ThunderAutoProjectState& state, uint64_t& hash) {
  size_t sum = 0;

  uint8_t sectionByte;
  sum += stream >> sectionByte;
  if (sectionByte != kSectionActions) {
    throw RuntimeError::Construct(
        "DeserializeThunderAutoProjectStateFromTransmission: Expected actions section (0x04), got 0x{:02X}",
        sectionByte);
  }

  uint8_t numActions;
  sum += stream >> numActions;

  for (size_t actionIndex = 0; actionIndex < static_cast<size_t>(numActions); ++actionIndex) {
    std::string actionName;
    sum += stream >> actionName;

    ThunderAutoAction action;
    sum += DeserializeAction(stream, action);

    state.addAction(actionName, action);
  }

  sum += stream >> hash;

  return sum;
}

std::vector<uint8_t> SerializeThunderAutoProjectStateForTransmission(
    const ThunderAutoProjectState& state) noexcept {
  std::vector<uint8_t> message;
  message.reserve(256);
  DataOutputStream stream(message);

  size_t sum = 0;

  sum += SerializeHeader(state, stream);
  sum += SerializeTrajectories(state, stream);
  sum += SerializeAutoModes(state, stream);
  sum += SerializeActions(state, stream);

  stream << uint8_t(sum & 0xFF);

  return message;
}

ThunderAutoProjectStateDataHashes DeserializeThunderAutoProjectStateFromTransmission(
    std::span<const uint8_t> data,
    ThunderAutoProjectState& state) {
  DataInputStream stream(data);

  ThunderAutoProjectStateDataHashes hashes;

  size_t sum = 0;

  sum += DeserializeHeader(stream, state);
  sum += DeserializeTrajectories(stream, state, hashes.trajectoryHashes);
  sum += DeserializeAutoModes(stream, state, hashes.autoModeHashes);
  sum += DeserializeActions(stream, state, hashes.actionsHash);

  uint8_t readSum;
  stream >> readSum;
  if (readSum != (sum & 0xFF)) {
    throw RuntimeError::Construct(
        "DeserializeThunderAutoProjectStateFromTransmission: Final checksum mismatch: expected 0x{:02X}, got "
        "0x{:02X}",
        sum & 0xFF, readSum);
  }

  return hashes;
}

ThunderAutoProjectStateDataHashes::Diff ThunderAutoProjectStateDataHashes::diff(
    const ThunderAutoProjectStateDataHashes& oldState) {
  Diff d;

  // Compare trajectories.
  {
    for (const auto& [name, oldHash] : oldState.trajectoryHashes) {
      auto newIt = trajectoryHashes.find(name);

      if (newIt == trajectoryHashes.end()) {
        d.removedTrajectories.insert(name);
        d.stateWasChanged = true;
        continue;
      }

      uint64_t newHash = newIt->second;
      if (newHash != oldHash) {
        d.updatedTrajectories.insert(name);
        d.stateWasChanged = true;
      }
    }

    for (const auto& [name, _] : trajectoryHashes) {
      if (!oldState.trajectoryHashes.contains(name)) {
        d.updatedTrajectories.insert(name);
        d.stateWasChanged = true;
      }
    }
  }

  // Compare auto modes.
  {
    for (const auto& [name, oldHash] : oldState.autoModeHashes) {
      auto newIt = autoModeHashes.find(name);

      if (newIt == autoModeHashes.end()) {
        d.removedAutoModes.insert(name);
        d.stateWasChanged = true;
        continue;
      }

      uint64_t newHash = newIt->second;
      if (newHash != oldHash) {
        d.updatedAutoModes.insert(name);
        d.stateWasChanged = true;
      }
    }

    for (const auto& [name, _] : autoModeHashes) {
      if (!oldState.autoModeHashes.contains(name)) {
        d.updatedAutoModes.insert(name);
        d.stateWasChanged = true;
      }
    }
  }

  // Compare actions.
  d.actionsWereUpdated = actionsHash != oldState.actionsHash;
  d.stateWasChanged |= d.actionsWereUpdated;

  return d;
}

}  // namespace thunder::core
