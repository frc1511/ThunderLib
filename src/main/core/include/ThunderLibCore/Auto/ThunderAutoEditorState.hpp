#pragma once

#include <ThunderLibCore/Auto/ThunderAutoMode.hpp>
#include <wpi/json.h>
#include <string>
#include <optional>

namespace thunder::core {

// State for Trajectory editor.
struct ThunderAutoTrajectoryEditorState {
  std::string currentTrajectoryName;

  enum class TrajectorySelection {
    NONE,
    WAYPOINT,
    ACTION,
    ROTATION,
  };
  TrajectorySelection trajectorySelection = TrajectorySelection::NONE;
  size_t selectionIndex = 0;  // Selected waypoint/action/rotation index.

  bool operator==(const ThunderAutoTrajectoryEditorState& other) const noexcept {
    return currentTrajectoryName == other.currentTrajectoryName &&
           trajectorySelection == other.trajectorySelection &&
           (trajectorySelection == TrajectorySelection::NONE || selectionIndex == other.selectionIndex);
  }
};

const char* TrajectorySelectionToString(
    ThunderAutoTrajectoryEditorState::TrajectorySelection trajectorySelection) noexcept;

void from_json(const wpi::json& json, ThunderAutoTrajectoryEditorState& state);
void to_json(wpi::json& json, const ThunderAutoTrajectoryEditorState& state) noexcept;

// State for Auto Mode editor.
struct ThunderAutoModeEditorState {
  std::string currentAutoModeName;
  std::optional<ThunderAutoModeStepPath> selectedStepPath;

  bool operator==(const ThunderAutoModeEditorState& other) const noexcept = default;
};

void from_json(const wpi::json& json, ThunderAutoModeEditorState& state);
void to_json(wpi::json& json, const ThunderAutoModeEditorState& state) noexcept;

struct ThunderAutoEditorState {
  enum class View {
    NONE,
    TRAJECTORY,
    AUTO_MODE,
  };
  View view = View::TRAJECTORY;

  ThunderAutoTrajectoryEditorState trajectoryEditorState;
  ThunderAutoModeEditorState autoModeEditorState;

  bool operator==(const ThunderAutoEditorState& other) const noexcept = default;
};

void from_json(const wpi::json& json, ThunderAutoEditorState& state);
void to_json(wpi::json& json, const ThunderAutoEditorState& state) noexcept;

}  // namespace thunder::core
