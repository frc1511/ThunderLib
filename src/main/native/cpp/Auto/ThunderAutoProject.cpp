#include <ThunderLib/Auto/ThunderAutoProject.hpp>
#include <ThunderLibDriver/Auto/ThunderAutoProject.hpp>
#include <frc2/command/Commands.h>

namespace thunder {

ThunderAutoProject::ThunderAutoProject() noexcept {
  m_handle = new driver::ThunderAutoProject();
}

ThunderAutoProject::ThunderAutoProject(const std::filesystem::path& projectPath) noexcept {
  m_handle = new driver::ThunderAutoProject(projectPath);
}

ThunderAutoProject::~ThunderAutoProject() noexcept {
  delete m_handle;
}

bool ThunderAutoProject::load(const std::filesystem::path& projectPath) noexcept {
  return m_handle->load(projectPath);
}

bool ThunderAutoProject::discoverAndLoadFromDeployDirectory() noexcept {
  return m_handle->discoverAndLoadFromDeployDirectory();
}

bool ThunderAutoProject::isLoaded() const noexcept {
  return m_handle->isLoaded();
}

ThunderAutoProject::operator bool() const noexcept {
  return isLoaded();
}

std::string ThunderAutoProject::getName() const noexcept {
  return m_handle->getName();
}

void ThunderAutoProject::registerActionCommand(const std::string& actionName, frc2::CommandPtr command) {
  registerActionCommand(actionName, std::shared_ptr<frc2::Command>(std::move(command).Unwrap()));
}

void ThunderAutoProject::registerActionCommand(const std::string& actionName,
                                               std::shared_ptr<frc2::Command> command) {
  m_actionCommands[actionName] = std::move(command);
}

bool ThunderAutoProject::isActionCommandRegistered(const std::string& actionName) const noexcept {
  return m_actionCommands.contains(actionName);
}

bool ThunderAutoProject::hasAction(const std::string& actionName) const noexcept {
  return m_handle->hasAction(actionName);
}

frc2::CommandPtr ThunderAutoProject::getActionCommand(const std::string& actionName) const noexcept {
  std::optional<core::ThunderAutoAction> actionOptional = m_handle->getAction(actionName);

  if (!actionOptional) {
    return frc2::cmd::None();
  }

  const core::ThunderAutoAction& action = *actionOptional;

  core::ThunderAutoActionType actionType = action.type();
  using enum core::ThunderAutoActionType;

  if (actionType == COMMAND) {
    auto it = m_actionCommands.find(actionName);
    if (it != m_actionCommands.end()) {
      std::shared_ptr<frc2::Command> sharedCommand = it->second;

      frc2::FunctionalCommand functionalCommand(
          [sharedCommand]() { sharedCommand->Initialize(); }, [sharedCommand]() { sharedCommand->Execute(); },
          [sharedCommand](bool interrupted) { sharedCommand->End(interrupted); },
          [sharedCommand]() { return sharedCommand->IsFinished(); });

      functionalCommand.AddRequirements(sharedCommand->GetRequirements());

      return std::move(functionalCommand).ToPtr();
    }
  } else if (actionType == SEQUENTIAL_ACTION_GROUP || actionType == CONCURRENT_ACTION_GROUP) {
    std::span<const std::string> groupActionNames = action.actionGroup();

    std::vector<frc2::CommandPtr> groupActionCommands;
    groupActionCommands.reserve(groupActionNames.size());

    for (const std::string& subActionName : groupActionNames) {
      frc2::CommandPtr subActionCommand = getActionCommand(subActionName);  // Recursion, hell yeah!
      groupActionCommands.push_back(std::move(subActionCommand));
    }

    if (actionType == SEQUENTIAL_ACTION_GROUP) {
      return frc2::cmd::Sequence(std::move(groupActionCommands));
    } else {
      return frc2::cmd::Parallel(std::move(groupActionCommands));
    }
  }

  return frc2::cmd::None();
}

void ThunderAutoProject::registerBooleanCondition(const std::string& conditionName,
                                                  BooleanConditionFunc condition) {
  m_booleanConditions[conditionName] = std::move(condition);
}

bool ThunderAutoProject::isBooleanConditionRegistered(const std::string& conditionName) const noexcept {
  return m_booleanConditions.contains(conditionName);
}

ThunderAutoProject::BooleanConditionFunc ThunderAutoProject::getBooleanCondition(
    const std::string& conditionName) const noexcept {
  auto it = m_booleanConditions.find(conditionName);
  if (it != m_booleanConditions.end()) {
    return it->second;
  }
  return nullptr;
}

void ThunderAutoProject::registerSwitchCondition(const std::string& name, SwitchConditionFunc condition) {
  m_switchConditions[name] = std::move(condition);
}

bool ThunderAutoProject::isSwitchConditionRegistered(const std::string& conditionName) const noexcept {
  return m_switchConditions.contains(conditionName);
}

ThunderAutoProject::SwitchConditionFunc ThunderAutoProject::getSwitchCondition(
    const std::string& conditionName) const noexcept {
  auto it = m_switchConditions.find(conditionName);
  if (it != m_switchConditions.end()) {
    return it->second;
  }
  return nullptr;
}

std::unique_ptr<ThunderAutoTrajectory> ThunderAutoProject::getTrajectory(
    const std::string& trajectoryName) const noexcept {
  driver::ThunderAutoTrajectory* trajectoryHandle = m_handle->getTrajectory(trajectoryName);
  // return std::make_unique<ThunderAutoTrajectory>(trajectoryHandle);
  ThunderAutoTrajectory* trajectory = new ThunderAutoTrajectory(trajectoryHandle);
  return std::unique_ptr<ThunderAutoTrajectory>(trajectory);
}

bool ThunderAutoProject::hasTrajectory(const std::string& trajectoryName) const noexcept {
  return m_handle->hasTrajectory(trajectoryName);
}

std::unordered_set<std::string> ThunderAutoProject::getTrajectoryNames() const noexcept {
  return m_handle->getTrajectoryNames();
}

std::unique_ptr<ThunderAutoMode> ThunderAutoProject::getAutoMode(
    const std::string& autoModeName) const noexcept {
  driver::ThunderAutoMode* autoModeHandle = m_handle->getAutoMode(autoModeName);
  ThunderAutoMode* autoMode = new ThunderAutoMode(autoModeHandle);
  return std::unique_ptr<ThunderAutoMode>(autoMode);
}

bool ThunderAutoProject::hasAutoMode(const std::string& autoModeName) const noexcept {
  return m_handle->hasAutoMode(autoModeName);
}

std::unordered_set<std::string> ThunderAutoProject::getAutoModeNames() const noexcept {
  return m_handle->getAutoModeNames();
}

FieldSymmetry ThunderAutoProject::getFieldSymmetry() const noexcept {
  return static_cast<FieldSymmetry>(m_handle->getFieldSymmetry());
}

FieldDimensions ThunderAutoProject::getFieldDimensions() const noexcept {
  driver::FieldDimensions driverDimensions = m_handle->getFieldDimensions();
  FieldDimensions dimensions{.width = driverDimensions.width, .length = driverDimensions.length};
  return dimensions;
}

void ThunderAutoProject::setRemoteUpdatesEnabled(bool enabled) noexcept {
  m_handle->setRemoteUpdatesEnabled(enabled);
}

void ThunderAutoProject::enableRemoteUpdates() noexcept {
  m_handle->enableRemoteUpdates();
}

void ThunderAutoProject::disableRemoteUpdates() noexcept {
  m_handle->disableRemoteUpdates();
}

bool ThunderAutoProject::areRemoteUpdatesEnabled() const noexcept {
  return m_handle->areRemoteUpdatesEnabled();
}

ThunderAutoProject::RemoteUpdateSubscriberID ThunderAutoProject::registerRemoteUpdateSubscriber(
    RemoteUpdateCallbackFunc callback) noexcept {
  return m_handle->registerRemoteUpdateSubscriber(callback);
}

bool ThunderAutoProject::unregisterRemoteUpdateSubscriber(RemoteUpdateSubscriberID id) noexcept {
  return m_handle->unregisterRemoteUpdateSubscriber(id);
}

driver::ThunderAutoProject* ThunderAutoProject::getHandle() noexcept {
  return m_handle;
}

}  // namespace thunder
