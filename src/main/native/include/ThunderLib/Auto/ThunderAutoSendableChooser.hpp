#pragma once

#include <ThunderLib/Auto/ThunderAutoProject.hpp>
#include <ThunderLib/Trajectory/TrajectoryRunnerProperties.hpp>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <string_view>
#include <string>
#include <unordered_map>
#include <memory>
#include <optional>

namespace thunder {

namespace driver {

class ThunderAutoSendableChooser;

}  // namespace driver

/**
 * A wrapper around frc::SendableChooser to handle ThunderAuto trajectory, auto mode, and command selections.
 *
 * This chooser will automatically update its options when the associated project gets updated.
 */
class ThunderAutoSendableChooser final {
 public:
  /**
   * Default constructor. The chooser will not be published to SmartDashboard until publish() is called.
   */
  ThunderAutoSendableChooser() noexcept;

  /**
   * Constructs an empty chooser and publishes it to SmartDashboard under the given key.
   *
   * @param smartDashboardKey The key in SmartDashboard to publish the chooser under.
   */
  explicit ThunderAutoSendableChooser(std::string_view smartDashboardKey) noexcept;

  /**
   * Constructs a chooser with the given trajectory runner properties.
   * The chooser will not be published to SmartDashboard until publish() is called.
   *
   * @param runnerProps The trajectory runner properties to use for making trajectory commands.
   */
  explicit ThunderAutoSendableChooser(const TrajectoryRunnerProperties& runnerProps) noexcept;

  /**
   * Constructs a chooser with the given trajectory runner properties and publishes it to SmartDashboard under
   * the given key.
   *
   * @param smartDashboardKey The key in SmartDashboard to publish the chooser under.
   * @param runnerProps The trajectory runner properties to use for making trajectory commands.
   */
  ThunderAutoSendableChooser(std::string_view smartDashboardKey,
                             const TrajectoryRunnerProperties& runnerProps) noexcept;

  ~ThunderAutoSendableChooser() noexcept;

  /**
   * Sets the trajectory runner properties to use for making trajectory commands.
   * 
   * @param runnerProps The trajectory runner properties to set.
   */
  void setTrajectoryRunnerProperties(const TrajectoryRunnerProperties& runnerProps) noexcept;

  /**
   * Publishes the chooser to SmartDashboard. This only needs to be called once, as the chooser will
   * automatically refresh the chooser when changes are made.
   *
   * @param smartDashboardKey The key to publish the chooser under.
   */
  void publish(std::string_view smartDashboardKey) noexcept;

  /**
   * Include a ThunderAuto project as a source for trajectories and auto modes. Trajectories and auto modes
   * from this project can later be added to the chooser using the appropriate methods.
   *
   * @param project The ThunderAuto project to include.
   * @param addAllAutoModes If true, all auto modes from the project will be added to the chooser.
   * @param addAllTrajectories If true, all trajectories from the project will be added to the chooser.
   */
  void includeProjectSource(std::shared_ptr<ThunderAutoProject> project,
                            bool addAllAutoModes = false,
                            bool addAllTrajectories = false) noexcept;

  /**
   * Add all trajectories from a previously included project to the chooser.
   *
   * @param projectName The name of the project to add trajectories from. This must match the name of a
   *                    project previously included using includeProjectSource().
   */
  void addAllTrajectoriesFromProject(const std::string& projectName) noexcept;

  /**
   * Add all auto modes from a previously included project to the chooser.
   *
   * @param projectName The name of the project to add auto modes from. This must match the name of a
   *                    project previously included using includeProjectSource().
   */
  void addAllAutoModesFromProject(const std::string& projectName) noexcept;

  /**
   * Add a trajectory from a previously included project to the chooser.
   *
   * @param projectName The name of the project to add the trajectory from. This must match the name of a
   *                    project previously included using includeProjectSource().
   * @param trajectoryName The name of the trajectory to add from the project.
   *
   * @return True if the trajectory was successfully added, false if the project or trajectory was not found,
   *         or another item with the same name was already added to the chooser.
   */
  bool addTrajectoryFromProject(const std::string& projectName, const std::string& trajectoryName) noexcept;

  /**
   * Add an auto mode from a previously included project to the chooser.
   *
   * @param projectName The name of the project to add the auto mode from. This must match the name of a
   *                    project previously included using includeProjectSource().
   * @param autoModeName The name of the auto mode to add from the project.
   *
   * @return True if the auto mode was successfully added, false if the project or auto mode was not found, or
   *         another item with the same name was already added to the chooser.
   */
  bool addAutoModeFromProject(const std::string& projectName, const std::string& autoModeName) noexcept;

  /**
   * Add a custom command to the chooser.
   *
   * @param name The name of the custom command.
   * @param command The command to add.
   *
   * @return True if the command was successfully added, false if another item with the same name was already
   *         added to the chooser.
   */
  bool addCustomCommand(const std::string& name, frc2::CommandPtr command) noexcept;

  /**
   * Add a custom command to the chooser.
   *
   * @param name The name of the custom command.
   * @param command The command to add.
   *
   * @return True if the command was successfully added, false if another item with the same name was already
   *         added to the chooser.
   */
  bool addCustomCommand(const std::string& name, std::shared_ptr<frc2::Command> command) noexcept;

  /**
   * Get the currently selected command from the chooser.
   *
   * @return The selected command.
   */
  frc2::CommandPtr getSelectedCommand() const noexcept;

  enum class ChooserSelectionType {
    NONE,
    AUTO_MODE,
    TRAJECTORY,
    CUSTOM_COMMAND,
  };

  struct ChooserSelection {
    ChooserSelectionType type = ChooserSelectionType::NONE;
    std::string projectName;
    std::string itemName;
  };

  /**
   * Get the currently selected item from the chooser.
   */
  ChooserSelection getSelected() const noexcept;

  driver::ThunderAutoSendableChooser* getHandle() noexcept;

 private:
  driver::ThunderAutoSendableChooser* m_handle = nullptr;

  std::optional<TrajectoryRunnerProperties> m_runnerProps;
  std::unordered_map<std::string, std::shared_ptr<ThunderAutoProject>> m_includedProjects;

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> m_customCommands;
};

}  // namespace thunder
