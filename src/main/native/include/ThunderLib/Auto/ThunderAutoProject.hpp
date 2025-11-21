#pragma once

#include <ThunderLib/Auto/ThunderAutoTrajectory.hpp>
#include <ThunderLib/Auto/ThunderAutoMode.hpp>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <unordered_map>
#include <unordered_set>
#include <filesystem>
#include <string>
#include <optional>
#include <string>
#include <string_view>
#include <memory>
#include <mutex>

namespace thunder {

namespace driver {

class ThunderAutoProject;

}  // namespace driver

class ThunderAutoProject final {
 public:
  /**
   * Default constructor. Creates an empty ThunderAuto project that is not loaded.
   */
  ThunderAutoProject() noexcept;

  /**
   * Loads a ThunderAuto project from the specified path.
   *
   * @param projectPath The path to the ThunderAuto project. If an absolute path is provided, it will be used
   *                    as-is. If a relative path is provided, it will be considered relative to the deploy
   *                    directory.
   */
  explicit ThunderAutoProject(const std::filesystem::path& projectPath) noexcept;

  ~ThunderAutoProject() noexcept;

  /**
   * Loads a ThunderAuto project from the specified path.
   *
   * @param projectPath The path to the ThunderAuto project. If an absolute path is provided, it will be used
   *                    as-is. If a relative path is provided, it will be considered relative to the deploy
   *                    directory.
   *
   * @return True if the project was successfully loaded, false otherwise.
   */
  bool load(const std::filesystem::path& projectPath) noexcept;

  /**
   * Scans the deploy directory for a ThunderAuto project and loads the first one it finds.
   *
   * @return True if a project was found and loaded, false otherwise.
   */
  bool discoverAndLoadFromDeployDirectory() noexcept;

  /**
   * Returns whether a project is successfully loaded.
   *
   * @return True if a project is loaded, false otherwise.
   */
  bool isLoaded() const noexcept;

  /**
   * Returns whether a project is successfully loaded.
   *
   * @return True if a project is loaded, false otherwise.
   */
  explicit operator bool() const noexcept;

  /**
   * Get the name of the loaded project.
   *
   * @return The name of the loaded project, or an empty string if no project is loaded.
   */
  std::string getName() const noexcept;

  /**
   * Register an action command that is used in a trajectory or auto mode. If an action is already registered
   * with the same name, it will be replaced. If an action referenced during the execution of a trajectory
   * or auto mode is not found, nothing will be executed.
   *
   * @param actionName The name of the action.
   * @param command    The command to be executed when the action is called.
   */
  void registerActionCommand(const std::string& actionName, frc2::CommandPtr command);

  /**
   * Register an action command that is used in a trajectory or auto mode. If an action is already registered
   * with the same name, it will be replaced. If an action referenced during the execution of a trajectory
   * or auto mode is not found, nothing will be executed.
   *
   * @param actionNameame The name of the action.
   * @param command       The command to be executed when the action is called.
   */
  void registerActionCommand(const std::string& actionName, std::shared_ptr<frc2::Command> command);

  /**
   * Check if an action command with the given name is registered.
   *
   * @param actionName The name of the action to check.
   *
   * @return True if the action command is registered, false otherwise.
   */
  bool isActionCommandRegistered(const std::string& actionName) const noexcept;

  /**
   * Check if an action with the given name exists in the project.
   *
   * @note This function has nothing to do with registered action commands, it is simply checking the
   *       existence of an action in the project.
   *
   * @param actionName The name of the action to check.
   *
   * @return True if the action exists, false otherwise.
   */
  bool hasAction(const std::string& actionName) const noexcept;

  /**
   * Get an action command by name.
   *
   * If the action name references an action group defined in the project, the corresponding command group
   * will be constructed and returned.
   *
   * If the action name references an action command that has been registered via registerActionCommand(), the
   * registered command will be returned.
   *
   * If the action name does not reference a valid action group in the project or a registered command, a None
   * command will be returned.
   *
   * @param actionName The name of the action to get.
   *
   * @return A CommandPtr representing the action command, or a None command if not found.
   */
  frc2::CommandPtr getActionCommand(const std::string& actionName) const noexcept;

  using BooleanConditionFunc = std::function<bool()>;

  /**
   * Register a boolean condition that is used during a branch step in an auto mode to determine the next step
   * to run. If a condition is already registered with the same name, it will be replaced. If a condition that
   * is referenced during the execution of an auto mode is not found, the auto mode will stop executing and an
   * error will be logged.
   *
   * @param conditionName The name of the condition.
   * @param condition     The function that returns a boolean indicating the next step to run.
   */
  void registerBooleanCondition(const std::string& conditionName, BooleanConditionFunc condition);

  /**
   * Check if a boolean condition with the given name is registered.
   * 
   * @param conditionName The name of the condition to check.
   * 
   * @return True if the boolean condition is registered, false otherwise.
   */
  bool isBooleanConditionRegistered(const std::string& conditionName) const noexcept;

  /**
   * Get a registered boolean condition by name.
   * 
   * @param conditionName The name of the condition to get.
   * 
   * @return The boolean condition function, or nullptr if not found.
   */
  BooleanConditionFunc getBooleanCondition(const std::string& conditionName) const noexcept;

  using SwitchConditionFunc = std::function<int()>;

  /**
   * Register a switch condition that is used during a branch step in an auto mode to determine the next step
   * to run. If a condition is already registered with the same name, it will be replaced. If a condition that
   * is referenced during the execution of an auto mode is not found, the auto mode will stop executing and an
   * error will be logged.
   *
   * @param conditionName The name of the condition.
   * @param condition     The function that returns an integer indicating the next step to run.
   */
  void registerSwitchCondition(const std::string& conditionName, SwitchConditionFunc condition);

  /**
   * Check if a switch condition with the given name is registered.
   * 
   * @param conditionName The name of the condition to check.
   * 
   * @return True if the switch condition is registered, false otherwise.
   */
  bool isSwitchConditionRegistered(const std::string& conditionName) const noexcept;

  /**
   * Get a registered switch condition by name.
   * 
   * @param conditionName The name of the condition to get.
   * 
   * @return The switch condition function, or nullptr if not found.
   */
  SwitchConditionFunc getSwitchCondition(const std::string& conditionName) const noexcept;

  /**
   * Get a trajectory by name.
   *
   * @param trajectoryName The name of the trajectory to get.
   *
   * @return A unique pointer to the trajectory if it exists, or nullptr if it does not.
   */
  [[nodiscard]]
  std::unique_ptr<ThunderAutoTrajectory> getTrajectory(const std::string& trajectoryName) const noexcept;

  /**
   * Check if a trajectory with the given name exists.
   *
   * @param trajectoryName The name of the trajectory to check.
   *
   * @return True if the trajectory exists, false otherwise.
   */
  bool hasTrajectory(const std::string& trajectoryName) const noexcept;

  /**
   * Get the names of all trajectories in the project.
   *
   * @return An unordered set of trajectory names.
   */
  std::unordered_set<std::string> getTrajectoryNames() const noexcept;

  /**
   * Get an autonomous mode by name.
   *
   * @param autoModeName The name of the autonomous mode to get.
   *
   * @return A unique pointer to the autonomous mode if it exists, or nullptr if it does not.
   */
  [[nodiscard]]
  std::unique_ptr<ThunderAutoMode> getAutoMode(const std::string& autoModeName) const noexcept;

  /**
   * Check if an autonomous mode with the given name exists.
   *
   * @param autoModeName The name of the autonomous mode to check.
   *
   * @return True if the autonomous mode exists, false otherwise.
   */
  bool hasAutoMode(const std::string& autoModeName) const noexcept;

  /**
   * Get the names of all autonomous modes in the project.
   *
   * @return An unordered set of autonomous mode names.
   */
  std::unordered_set<std::string> getAutoModeNames() const noexcept;

  /**
   * Get the symmetry of the configured field for the loaded project.
   * 
   * @return The field symmetry.
   */
  FieldSymmetry getFieldSymmetry() const noexcept;

  /**
   * Get the size of the configured field for the loaded project.
   * 
   * @return The field size.
   */
  FieldDimensions getFieldDimensions() const noexcept;

  /**
   * Set whether remote project updates from ThunderAuto are enabled (enabled by default).
   * Remote updates will only occur only when the robot is in Disabled mode and FMS is not connected.
   *
   * @param enabled True to enable remote updates, false to disable them.
   */
  void setRemoteUpdatesEnabled(bool enabled) noexcept;

  /**
   * Enable remote project updates from ThunderAuto (enabled by default).
   */
  void enableRemoteUpdates() noexcept;

  /**
   * Disable remote project updates from ThunderAuto.
   */
  void disableRemoteUpdates() noexcept;

  /**
   * Check if remote project updates from ThunderAuto are enabled (enabled by default).
   *
   * @return True if remote updates are enabled, false otherwise.
   */
  bool areRemoteUpdatesEnabled() const noexcept;

  using RemoteUpdateCallbackFunc = std::function<void()>;
  using RemoteUpdateSubscriberID = size_t;

  /**
   * Register a callback function to be called when the project is updated remotely from ThunderAuto.
   *
   * @param callback The callback function to register.
   *
   * @return A subscriber ID that can be used to unregister the callback, or 0 on failure.
   */
  RemoteUpdateSubscriberID registerRemoteUpdateSubscriber(RemoteUpdateCallbackFunc callback) noexcept;

  /**
   * Unregister a previously registered remote update subscriber.
   *
   * @param id The subscriber ID returned by registerRemoteUpdateSubscriber().
   *
   * @return True if the subscriber was successfully unregistered, false otherwise.
   */
  bool unregisterRemoteUpdateSubscriber(RemoteUpdateSubscriberID id) noexcept;

  driver::ThunderAutoProject* getHandle() noexcept;

 private:
  driver::ThunderAutoProject* m_handle;

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> m_actionCommands;
  std::unordered_map<std::string, BooleanConditionFunc> m_booleanConditions;
  std::unordered_map<std::string, SwitchConditionFunc> m_switchConditions;
};

}  // namespace thunder
