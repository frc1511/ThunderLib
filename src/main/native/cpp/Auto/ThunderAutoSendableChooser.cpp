#include <ThunderLib/Auto/ThunderAutoSendableChooser.hpp>
#include <ThunderLibDriver/Auto/ThunderAutoSendableChooser.hpp>
#include <ThunderLib/Commands/ThunderAutoTrajectoryCommand.hpp>
#include <ThunderLib/Commands/ThunderAutoModeCommand.hpp>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <functional>

namespace thunder {

ThunderAutoSendableChooser::ThunderAutoSendableChooser() noexcept {
  m_handle = new driver::ThunderAutoSendableChooser(
      std::bind(&ThunderAutoSendableChooser::addChooserSelection, this, std::placeholders::_1),
      std::bind(&ThunderAutoSendableChooser::publishChooser, this, std::placeholders::_1));
  m_chooser.SetDefaultOption("Do Nothing", ThunderAutoSendableChooserSelection{});
}

ThunderAutoSendableChooser::ThunderAutoSendableChooser(std::string_view smartDashboardKey) noexcept {
  m_handle = new driver::ThunderAutoSendableChooser(
      std::bind(&ThunderAutoSendableChooser::addChooserSelection, this, std::placeholders::_1),
      std::bind(&ThunderAutoSendableChooser::publishChooser, this, std::placeholders::_1), smartDashboardKey);
  m_chooser.SetDefaultOption("Do Nothing", ThunderAutoSendableChooserSelection{});
}

ThunderAutoSendableChooser::ThunderAutoSendableChooser(const TrajectoryRunnerProperties& runnerProps) noexcept
    : ThunderAutoSendableChooser() {
  setTrajectoryRunnerProperties(runnerProps);
}

ThunderAutoSendableChooser::ThunderAutoSendableChooser(std::string_view smartDashboardKey,
                                                       const TrajectoryRunnerProperties& runnerProps) noexcept
    : ThunderAutoSendableChooser(smartDashboardKey) {
  setTrajectoryRunnerProperties(runnerProps);
}

ThunderAutoSendableChooser::~ThunderAutoSendableChooser() noexcept {
  delete m_handle;
}

void ThunderAutoSendableChooser::setTrajectoryRunnerProperties(
    const TrajectoryRunnerProperties& runnerProps) noexcept {
  m_runnerProps.emplace(runnerProps);
}

void ThunderAutoSendableChooser::publish(std::string_view smartDashboardKey) noexcept {
  m_handle->publish(smartDashboardKey);
}

void ThunderAutoSendableChooser::includeProjectSource(std::shared_ptr<ThunderAutoProject> project,
                                                      bool addAllAutoModes /*= false*/,
                                                      bool addAllTrajectories /*= false*/) noexcept {
  m_handle->includeProjectSource(project->getHandle(), addAllAutoModes, addAllTrajectories);
  m_includedProjects.emplace(project->getName(), std::move(project));
}

void ThunderAutoSendableChooser::addAllTrajectoriesFromProject(const std::string& projectName) noexcept {
  m_handle->addAllTrajectoriesFromProject(projectName);
}

void ThunderAutoSendableChooser::addAllAutoModesFromProject(const std::string& projectName) noexcept {
  m_handle->addAllAutoModesFromProject(projectName);
}

bool ThunderAutoSendableChooser::addTrajectoryFromProject(const std::string& projectName,
                                                          const std::string& trajectoryName) noexcept {
  return m_handle->addTrajectoryFromProject(projectName, trajectoryName);
}

bool ThunderAutoSendableChooser::addCustomCommand(const std::string& name,
                                                  frc2::CommandPtr command) noexcept {
  return addCustomCommand(name, std::shared_ptr<frc2::Command>(std::move(command).Unwrap()));
}

bool ThunderAutoSendableChooser::addCustomCommand(const std::string& name,
                                                  std::shared_ptr<frc2::Command> command) noexcept {
  bool result = m_handle->addCustomCommand(name);
  if (result) {
    m_customCommands[name] = std::move(command);
  }
  return result;
}

bool ThunderAutoSendableChooser::addAutoModeFromProject(const std::string& projectName,
                                                        const std::string& autoModeName) noexcept {
  return m_handle->addAutoModeFromProject(projectName, autoModeName);
}

frc2::CommandPtr ThunderAutoSendableChooser::getSelectedCommand() const noexcept {
  const ThunderAutoSendableChooserSelection selection = getSelected();

  using enum ThunderAutoSendableChooserSelectionType;
  if (selection.type == CUSTOM_COMMAND) {
    auto it = m_customCommands.find(selection.itemName);
    if (it != m_customCommands.end()) {
      std::shared_ptr<frc2::Command> sharedCommand = it->second;

      frc2::FunctionalCommand functionalCommand(
          [sharedCommand]() { sharedCommand->Initialize(); }, [sharedCommand]() { sharedCommand->Execute(); },
          [sharedCommand](bool interrupted) { sharedCommand->End(interrupted); },
          [sharedCommand]() { return sharedCommand->IsFinished(); });

      functionalCommand.AddRequirements(sharedCommand->GetRequirements());

      return std::move(functionalCommand).ToPtr();
    }
  } else if (selection.type != NONE && m_runnerProps.has_value()) {  // The rest require project lookup
    auto projectIt = m_includedProjects.find(selection.projectName);
    if (projectIt != m_includedProjects.end()) {
      std::shared_ptr<ThunderAutoProject> project = projectIt->second;

      if (selection.type == AUTO_MODE) {
        ThunderAutoModeCommand autoModeCommand(selection.itemName, project, m_runnerProps.value());
        return std::move(autoModeCommand).ToPtr();
      } else if (selection.type == TRAJECTORY) {
        ThunderAutoTrajectoryCommand trajectoryCommand(selection.itemName, project, m_runnerProps.value());
        return std::move(trajectoryCommand).ToPtr();
      }
    }
  }

  return frc2::cmd::None();
}

ThunderAutoSendableChooserSelection ThunderAutoSendableChooser::getSelected() const noexcept {
  return m_chooser.GetSelected();
}

driver::ThunderAutoSendableChooser* ThunderAutoSendableChooser::getHandle() noexcept {
  return m_handle;
}

const driver::ThunderAutoSendableChooser* ThunderAutoSendableChooser::getHandle() const noexcept {
  return m_handle;
}

void ThunderAutoSendableChooser::addChooserSelection(
    const driver::ThunderAutoSendableChooserSelection& selection) noexcept {
  m_chooser.AddOption(selection.itemName, convertChooserSelection(selection));
}

void ThunderAutoSendableChooser::publishChooser(const std::string& key) noexcept {
  frc::SmartDashboard::PutData(key, &m_chooser);
  frc::SmartDashboard::UpdateValues();
}

ThunderAutoSendableChooserSelection ThunderAutoSendableChooser::convertChooserSelection(
    const driver::ThunderAutoSendableChooserSelection& driverSelection) noexcept {
  ThunderAutoSendableChooserSelection selection;
  selection.type = static_cast<ThunderAutoSendableChooserSelectionType>(driverSelection.type);
  selection.projectName = driverSelection.projectName;
  selection.itemName = driverSelection.itemName;
  return selection;
}

}  // namespace thunder
