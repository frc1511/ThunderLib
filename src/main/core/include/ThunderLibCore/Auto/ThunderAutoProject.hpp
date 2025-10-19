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
#include <optional>
#include <unordered_set>

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

  /**
   * Finds the initial pose for a given auto mode.
   *
   * @param autoModeName The name of the auto mode to find the initial pose for.
   *
   * @return An optional Pose2d representing the initial pose of the auto mode, or std::nullopt if not
   * available.
   */
  std::optional<frc::Pose2d> getAutoModeInitialPose(std::string_view autoModeName) const noexcept;

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

  void trajectorySelect(const std::string& trajectoryName);
  ThunderAutoTrajectorySkeleton& currentTrajectory();
  const ThunderAutoTrajectorySkeleton& currentTrajectory() const;
  ThunderAutoTrajectorySkeletonWaypoint& currentTrajectorySelectedWaypoint();
  const ThunderAutoTrajectorySkeletonWaypoint& currentTrajectorySelectedWaypoint() const;
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

 private:
  void fromJsonPre2026Version(const wpi::json& json);
  void fromJsonCurrentVersion(const wpi::json& json);

  void validateActions();
  void validateWaypointLinks();
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

}  // namespace thunder::core
