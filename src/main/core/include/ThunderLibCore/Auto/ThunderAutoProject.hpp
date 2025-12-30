#pragma once

#include <ThunderLibCore/Auto/ThunderAutoTrajectorySkeleton.hpp>
#include <ThunderLibCore/Auto/ThunderAutoFieldImage.hpp>
#include <ThunderLibCore/Auto/ThunderAutoMode.hpp>
#include <ThunderLibCore/Auto/ThunderAutoEditorState.hpp>
#include <ThunderLibCore/Auto/ThunderAutoOutputTrajectory.hpp>
#include <ThunderLibCore/Types.hpp>

#include <frc/geometry/Pose2d.h>
#include <units/length.h>
#include <wpi/json.h>

#include <string_view>
#include <filesystem>
#include <string>
#include <memory>
#include <vector>
#include <map>
#include <span>
#include <optional>
#include <unordered_set>
#include <cstdint>

namespace thunder::core {

enum class DriveControllerType {
  HOLONOMIC,
  // RAMSETE,
};

/**
 * Represents the version of a ThunderAuto project (this version is independent of ThunderAuto and ThunderLib
 * versions and simply indicates changes to the project format).
 *
 * The major version is still tied to the FRC game year, as new field images are introduced each year.
 *
 * The minor version is incremented when the project format is changed mid-season. These changes should be
 * backward compatible with previous versions of ThunderLib from the same year. This usually means that a new
 * property was added to the project settings or state that older versions of ThunderLib will ignore. Note
 * that it is still not recommended to use an older version of ThunderLib with a project that was saved with a
 * newer version, as any new properties will not be saved by the older version, which may lead to loss of data
 * or unexpected behavior.
 */
struct ThunderAutoProjectVersion {
  int major;
  int minor;

  bool operator==(const ThunderAutoProjectVersion& other) const noexcept = default;
};

ThunderAutoProjectVersion CurrentThunderAutoProjectVersion() noexcept;

/**
 * Represents the settings for a ThunderAuto project.
 *
 * These are typically not touched after the project is created, with the exception of editor settings like
 * robotSize and autoSave/Export.
 */
struct ThunderAutoProjectSettings {
  std::filesystem::path projectPath;
  std::filesystem::path directory;
  std::string name;

  ThunderAutoFieldImage fieldImage;

  DriveControllerType driveController;
  Measurement2d robotSize;
  units::meter_t robotCornerRadius = 0.2_m;

  bool autoSave;
  bool autoCSVExport;
  ThunderAutoCSVExportProperties csvExportProps;

 public:
  ThunderAutoProjectSettings() noexcept;

  /**
   * Constructs settings from JSON.
   *
   * @param path The path to the project file.
   * @param json The JSON object containing the settings.
   * @param version The project version.
   */
  ThunderAutoProjectSettings(const std::filesystem::path& path,
                             const wpi::json& json,
                             const ThunderAutoProjectVersion& version);

  /**
   * Fill settings from JSON.
   *
   * @param path The path to the project file.
   * @param json The JSON object containing the settings.
   * @param version The project version.
   */
  void fromJson(const std::filesystem::path& path,
                const wpi::json& json,
                const ThunderAutoProjectVersion& version);

  void setProjectPath(const std::filesystem::path& path) noexcept;

  bool operator==(const ThunderAutoProjectSettings& other) const noexcept = default;

 private:
  void fromJsonPre2026Version(const wpi::json& json);
  void fromJsonCurrentVersion(const wpi::json& json);
};

enum class ThunderAutoActionType {
  COMMAND = 0,
  SEQUENTIAL_ACTION_GROUP,
  CONCURRENT_ACTION_GROUP,
};

const char* ThunderAutoActionTypeToString(ThunderAutoActionType type) noexcept;

class ThunderAutoAction {
  std::vector<std::string> m_actionGroupVec;
  std::unordered_map<std::string, size_t> m_actionGroupMap;

  ThunderAutoActionType m_type = ThunderAutoActionType::COMMAND;

 public:
  ThunderAutoAction() {}
  explicit ThunderAutoAction(ThunderAutoActionType type, const std::vector<std::string>& actionGroupVec = {});

  ThunderAutoAction(const ThunderAutoAction& other);
  ThunderAutoAction& operator=(const ThunderAutoAction& other) noexcept;

  bool operator==(const ThunderAutoAction& other) const noexcept = default;

  ThunderAutoActionType type() const noexcept { return m_type; };
  void setType(ThunderAutoActionType type) noexcept;

  // For SEQUENTIAL_ACTION_GROUP or CONCURRENT_ACTION_GROUP
  const std::vector<std::string>& actionGroup() const noexcept { return m_actionGroupVec; };
  void setActionGroup(const std::vector<std::string>& actionGroupVec);

  bool hasGroupAction(const std::string& actionName) noexcept;
  bool addGroupAction(const std::string& actionName) noexcept;
  void renameGroupAction(const std::string& originalActionName, const std::string& newActionName);
  bool removeGroupAction(const std::string& actionName) noexcept;
  bool swapGroupActionWithNext(std::string actionName);
  bool swapGroupActionWithPrevious(std::string actionName);
};

/**
 * Represents the state of a ThunderAuto project.
 * This is where all the important data for the project is stored.
 */
struct ThunderAutoProjectState {
 private:
  std::unordered_map<std::string, ThunderAutoAction> m_actions;
  std::vector<std::string> m_actionsOrder;

 public:
  std::map<std::string, ThunderAutoTrajectorySkeleton> trajectories;
  std::map<std::string, ThunderAutoMode> autoModes;

  const std::unordered_map<std::string, ThunderAutoAction>& actions = m_actions;
  const std::vector<std::string>& actionsOrder = m_actionsOrder;

  std::unordered_set<std::string> waypointLinks;

  ThunderAutoEditorState editorState;

 public:
  ThunderAutoProjectState() noexcept = default;

  /**
   * Constructs the state from JSON.
   *
   * @param json The JSON object containing the state.
   * @param version The project version.
   */
  ThunderAutoProjectState(const wpi::json& json, const ThunderAutoProjectVersion& version);

  ThunderAutoProjectState(const ThunderAutoProjectState& other) noexcept;
  ThunderAutoProjectState& operator=(const ThunderAutoProjectState& other) noexcept;

  /**
   * Fill the state from JSON.
   *
   * @param json The JSON object containing the state.
   * @param version The project version.
   */
  void fromJson(const wpi::json& json, const ThunderAutoProjectVersion& version);

  bool operator==(const ThunderAutoProjectState& other) const noexcept {
    return trajectories == other.trajectories && autoModes == other.autoModes &&
           m_actions == other.m_actions && m_actionsOrder == other.m_actionsOrder &&
           waypointLinks == other.waypointLinks && editorState == other.editorState;
  }

  void addAction(const std::string& actionName, const ThunderAutoAction& actionInfo) noexcept;
  ThunderAutoAction& getAction(const std::string& actionName);
  const ThunderAutoAction& getAction(const std::string& actionName) const;
  void removeAction(const std::string& actionName) noexcept;
  void renameAction(const std::string& oldName, const std::string& newName);
  void moveActionBeforeOther(const std::string& actionName, const std::string& otherActionName);
  void moveActionAfterOther(const std::string& actionName, const std::string& otherActionName);

  /**
   * Find a recursive path from a given action back to itself through group actions of type
   * SEQUENTIAL_ACTION_GROUP or CONCURRENT_ACTION_GROUP.
   *
   * @return Resursion path, or an empty list if no recursive path was found.
   */
  std::list<std::string> findActionRecursionPath(const std::string& actionName) const;

  /**
   * Updates the editor state to select the trajectory with the given name. If the editor state is not already
   * in trajectory editing mode, it will switch to that mode.
   *
   * Throws an exception if no trajectory exists in the project state with the given name.
   *
   * @param trajectoryName The name of the trajectory to select.
   */
  void trajectorySelect(const std::string& trajectoryName);

  /**
   * Returns the currently selected trajectory as referenced by the editor state.
   *
   * Throws an exception if no trajectory is currently selected, or the selected trajectory is missing from
   * the project state.
   *
   * @return Reference to the currently selected trajectory.
   */
  ThunderAutoTrajectorySkeleton& currentTrajectory();
  const ThunderAutoTrajectorySkeleton& currentTrajectory() const;

  /**
   * Returns the currently selected waypoint in the current trajectory as referenced by the editor state.
   *
   * Throws an exception if no trajectory is currently selected, or the selected trajectory is missing from
   * the project state, or if no waypoint is currently selected.
   *
   * @return Reference to the currently selected trajectory waypoint.
   */
  ThunderAutoTrajectorySkeletonWaypoint& currentTrajectorySelectedWaypoint();
  const ThunderAutoTrajectorySkeletonWaypoint& currentTrajectorySelectedWaypoint() const;

  /**
   * Trajectory manipulation functions, for use by the editor.
   *
   * Most of these functions modify the currently selected trajectory as referenced by the editor state. An
   * exception will be thrown if no trajectory is currently selected, (same as currentTrajectory()).
   */

  void currentTrajectoryInsertWaypoint(size_t index, Point2d position, CanonicalAngle outgoingHeading);
  void currentTrajectoryPrependWaypoint(Point2d position, CanonicalAngle outgoingHeading);
  void currentTrajectoryAppendWaypoint(Point2d position, CanonicalAngle outgoingHeading);
  void currentTrajectoryInsertRotation(ThunderAutoTrajectoryPosition position, CanonicalAngle angle);
  void currentTrajectoryInsertAction(ThunderAutoTrajectoryPosition position, const std::string& action);
  bool currentTrajectoryDeleteSelectedItem();
  void currentTrajectoryToggleEditorLockedForSelectedItem();
  bool currentTrajectoryIncrementSelectedItemIndex(bool forwards = true);

  void trajectoryUpdateLinkedWaypointsFromSelected();
  void trajectoryUpdateSelectedWaypointFromLink();

  void trajectoryDelete(const std::string& trajectoryName);
  void trajectoryRename(const std::string& oldTrajectoryName, const std::string& newTrajectoryName);
  void trajectoryDuplicate(const std::string& oldTrajectoryName, const std::string& newTrajectoryName);
  void trajectoryReverseDirection(const std::string& trajectoryName);

  /**
   * Updates the editor state to select the auto mode with the given name. If the editor state is not already
   * in auto mode editing mode, it will switch to that mode.
   *
   * Throws an exception if no auto mode exists in the project state with the given name.
   *
   * @param autoModeName The name of the auto mode to select.
   */
  void autoModeSelect(const std::string& autoModeName);

  /**
   * Returns the currently selected auto mode as referenced by the editor state.
   *
   * Throws an exception if no auto mode is currently selected, or the selected auto mode is missing from the
   * project state.
   *
   * @return Reference to the currently selected auto mode.
   */
  ThunderAutoMode& currentAutoMode();
  const ThunderAutoMode& currentAutoMode() const;

  /**
   * Auto mode step manipulation functions, for use by the editor. Steps can be moved or inserted to positions
   * relative to other steps.
   *
   * Most of these functions modify the currently selected auto mode as referenced by the editor state. An
   * exception will be thrown if no auto mode is currently selected, (same as currentAutoMode()).
   *
   * These functions will throw an exception if step paths are invalid.
   *
   * These functions will return true if the operation was successful, or false if no changes to the state
   * were made (for example, trying to move a step to the same position it is already in).
   */

  bool currentAutoModeMoveStepBeforeOther(const ThunderAutoModeStepPath& stepPath,
                                          const ThunderAutoModeStepPath& otherStepPath);
  bool currentAutoModeMoveStepAfterOther(const ThunderAutoModeStepPath& stepPath,
                                         const ThunderAutoModeStepPath& otherStepPath);
  bool currentAutoModeMoveStepIntoDirectory(const ThunderAutoModeStepPath& stepPath,
                                            const ThunderAutoModeStepDirectoryPath& directoryPath);
  void currentAutoModeInsertStepBeforeOther(const ThunderAutoModeStepPath& stepPath,
                                            std::unique_ptr<ThunderAutoModeStep> step);
  void currentAutoModeInsertStepAfterOther(const ThunderAutoModeStepPath& stepPath,
                                           std::unique_ptr<ThunderAutoModeStep> step);
  void currentAutoModeInsertStepInDirectory(const ThunderAutoModeStepDirectoryPath& directoryPath,
                                            std::unique_ptr<ThunderAutoModeStep> step);
  void currentAutoModeDeleteStep(const ThunderAutoModeStepPath& stepPath);

  void autoModeDelete(const std::string& autoModeName);
  void autoModeRename(const std::string& oldAutoModeName, const std::string& newAutoModeName);
  void autoModeDuplicate(const std::string& oldAutoModeName, const std::string& newAutoModeName);

 private:
  void fromJsonPre2026Version(const wpi::json& json);
  void fromJsonCurrentVersion(const wpi::json& json);

  void validateActionsAndTrajectories();
  void validateActionsAndTrajectoriesInAutoModeStep(const std::unique_ptr<ThunderAutoModeStep>& step);

  void validateWaypointLinks();

  void renameActionsInAutoModeStep(std::unique_ptr<ThunderAutoModeStep>& step,
                                   const std::string& oldName,
                                   const std::string& newName);

  void renameTrajectoryInAutoModeStep(std::unique_ptr<ThunderAutoModeStep>& step,
                                      const std::string& oldName,
                                      const std::string& newName);
};

class ThunderAutoProject {
  ThunderAutoProjectSettings m_settings;
  ThunderAutoProjectState m_state;

 public:
  ThunderAutoProject() noexcept = default;

  ThunderAutoProject(const ThunderAutoProjectSettings& settings,
                     const ThunderAutoProjectState& state) noexcept;

  ThunderAutoProject(const std::filesystem::path& path,
                     const ThunderAutoProjectVersion& version,
                     const wpi::json& json);

  ~ThunderAutoProject() noexcept = default;

  bool operator==(const ThunderAutoProject& other) const noexcept = default;

  const ThunderAutoProjectSettings& settings() const noexcept { return m_settings; }
  ThunderAutoProjectSettings& settings() noexcept { return m_settings; }

  const ThunderAutoProjectState& state() const noexcept { return m_state; }
  ThunderAutoProjectState& state() noexcept { return m_state; }

  void save() const;
};

/**
 * Loads a ThunderAuto project from the specified path.
 * This function will throw an exception if a problem occurs while loading the project.
 *
 * @param path The path to the project file.
 * @param version If provided, this will be filled with the version of the project.
 *
 * @return A unique pointer to the loaded ThunderAutoProject.
 */
std::unique_ptr<ThunderAutoProject> LoadThunderAutoProject(const std::filesystem::path& path,
                                                           ThunderAutoProjectVersion* version = nullptr);

/**
 * Saves the ThunderAuto project to the specified path.
 * This function will overwrite any existing project file at the path.
 * This function will throw an exception if a problem occurs while saving the project.
 *
 * @param settings The settings for the project.
 * @param state The state of the project.
 */
void SaveThunderAutoProject(const ThunderAutoProjectSettings& settings, const ThunderAutoProjectState& state);

std::vector<std::filesystem::path> DiscoverThunderAutoProjects(const std::filesystem::path& path) noexcept;

/**
 * Serialize a ThunderAuto Project state into a binary format suitable to be transmitted, i.e. from the
 * ThunderAuto application to a robot.
 *
 * Only essential data representing the trajectory will be serialized. Editor state information will not be
 * serialized.
 *
 * @param state The project state to serialize.
 *
 * @return The serialized state, as an array of bytes.
 */
std::vector<uint8_t> SerializeThunderAutoProjectStateForTransmission(
    const ThunderAutoProjectState& state) noexcept;

struct ThunderAutoProjectStateDataHashes {
  std::unordered_map<std::string, uint64_t> trajectoryHashes;
  std::unordered_map<std::string, uint64_t> autoModeHashes;
  uint64_t actionsHash = 0;

  struct Diff {
    std::unordered_set<std::string> updatedTrajectories;
    std::unordered_set<std::string> removedTrajectories;
    std::unordered_set<std::string> updatedAutoModes;
    std::unordered_set<std::string> removedAutoModes;
    bool actionsWereUpdated = false;
    bool stateWasChanged = false;
  };

  /**
   * Compares this set of hashes to an older set of hashes to find which trajectories and auto modes were
   * updated or removed, and whether actions were updated.
   *
   * @param oldState A set of hashes representing an older ThunderAuto project state.
   *
   * @return The changes between this state and the old state
   */
  Diff diff(const ThunderAutoProjectStateDataHashes& oldState);
};

/**
 * Deserialize a ThunderAuto project state from a binary format generated by
 * SerializeThunderAutoProjectStateForTransmission().
 *
 * This function will throw an exception if a problem occurs during deserialization, or if the serialized
 * project's version is too new.
 *
 * @param data The serialized ThunderAuto project state.
 * @param state The output state to be updated from the serialzied state data.
 *
 * @return Hashes from the serialized project state.
 */
ThunderAutoProjectStateDataHashes DeserializeThunderAutoProjectStateFromTransmission(
    std::span<const uint8_t> data,
    ThunderAutoProjectState& state);

}  // namespace thunder::core
