#include <ThunderLibCore/Auto/ThunderAutoEditorState.hpp>

#include <ThunderLibCore/Error.hpp>

namespace thunder::core {

const char* TrajectorySelectionToString(
    ThunderAutoTrajectoryEditorState::TrajectorySelection trajectorySelection) noexcept {
  switch (trajectorySelection) {
    using enum ThunderAutoTrajectoryEditorState::TrajectorySelection;
    case NONE:
      return "None";
    case WAYPOINT:
      return "Point";
    case ACTION:
      return "Action";
    case ROTATION:
      return "Rotation";
    default:
      ThunderLibUnreachable("Invalid trajectory selection");
  }
}

void from_json(const wpi::json& json, ThunderAutoTrajectoryEditorState& state) {
  json.at("current_trajectory_name").get_to(state.currentTrajectoryName);
}

void to_json(wpi::json& json, const ThunderAutoTrajectoryEditorState& state) noexcept {
  json = wpi::json{
      {"current_trajectory_name", state.currentTrajectoryName},
  };
}

void from_json(const wpi::json& json, ThunderAutoModeEditorState& state) {
  json.at("current_auto_mode_name").get_to(state.currentAutoModeName);
}

void to_json(wpi::json& json, const ThunderAutoModeEditorState& state) noexcept {
  json = wpi::json{
      {"current_auto_mode_name", state.currentAutoModeName},
  };
}

void from_json(const wpi::json& json, ThunderAutoEditorState& state) {
  size_t viewIndex = 0;
  json.at("view").get_to(viewIndex);
  state.view = static_cast<ThunderAutoEditorState::View>(viewIndex);
  json.at("trajectory_editor_state").get_to(state.trajectoryEditorState);
  json.at("mode_editor_state").get_to(state.modeEditorState);
}

void to_json(wpi::json& json, const ThunderAutoEditorState& state) noexcept {
  json = wpi::json{
      {"view", static_cast<size_t>(state.view)},
      {"trajectory_editor_state", state.trajectoryEditorState},
      {"mode_editor_state", state.modeEditorState},
  };
}

}  // namespace thunder::core
