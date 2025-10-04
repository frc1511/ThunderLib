#include <ThunderLibCore/Auto/ThunderAutoMode.hpp>
#include <ThunderLibCore/Error.hpp>

namespace thunder::core {

bool operator==(const ThunderAutoModeStep& lhs, const ThunderAutoModeStep& rhs) noexcept {
  if (lhs.type() != rhs.type()) {
    return false;
  }

  ThunderAutoModeStepType type = lhs.type();

  switch (type) {
    case ThunderAutoModeStepType::ACTION:
      return static_cast<const ThunderAutoModeActionStep&>(lhs) ==
             static_cast<const ThunderAutoModeActionStep&>(rhs);
    case ThunderAutoModeStepType::TRAJECTORY:
      return static_cast<const ThunderAutoModeTrajectoryStep&>(lhs) ==
             static_cast<const ThunderAutoModeTrajectoryStep&>(rhs);
    case ThunderAutoModeStepType::BRANCH_BOOL:
      return static_cast<const ThunderAutoModeBoolBranchStep&>(lhs) ==
             static_cast<const ThunderAutoModeBoolBranchStep&>(rhs);
    case ThunderAutoModeStepType::BRANCH_SWITCH:
      return static_cast<const ThunderAutoModeSwitchBranchStep&>(lhs) ==
             static_cast<const ThunderAutoModeSwitchBranchStep&>(rhs);
    default:
      ThunderLibUnreachable("Unknown ThunderAutoModeStepType");
  }
}

bool operator==(const std::shared_ptr<ThunderAutoModeStep>& lhs,
                const std::shared_ptr<ThunderAutoModeStep>& rhs) noexcept {
  if (lhs.get() == rhs.get()) {
    return true;
  }
  if (!lhs || !rhs) {
    return false;
  }
  return (*lhs == *rhs);
}

static void to_json(wpi::json& json, const ThunderAutoModeStepType& stepType) {
  switch (stepType) {
    case ThunderAutoModeStepType::ACTION:
      json = "action";
      break;
    case ThunderAutoModeStepType::TRAJECTORY:
      json = "trajectory";
      break;
    case ThunderAutoModeStepType::BRANCH_BOOL:
      json = "branch_bool";
      break;
    case ThunderAutoModeStepType::BRANCH_SWITCH:
      json = "branch_switch";
      break;
    default:
      ThunderLibUnreachable("Unknown ThunderAutoModeStepType");
  }
}

static void from_json(const wpi::json& json, ThunderAutoModeStepType& stepType) {
  std::string typeStr = json.get<std::string>();
  if (typeStr == "action") {
    stepType = ThunderAutoModeStepType::ACTION;
  } else if (typeStr == "trajectory") {
    stepType = ThunderAutoModeStepType::TRAJECTORY;
  } else if (typeStr == "branch_bool") {
    stepType = ThunderAutoModeStepType::BRANCH_BOOL;
  } else if (typeStr == "branch_switch") {
    stepType = ThunderAutoModeStepType::BRANCH_SWITCH;
  } else {
    throw RuntimeError::Construct("Unknown ThunderAutoModeStepType: {}", typeStr);
  }
}

static void to_json(wpi::json& json, const std::shared_ptr<ThunderAutoModeStep>& step);

static void to_json(wpi::json& json, const ThunderAutoModeStep& step) {
  json["type"] = step.type();

  switch (step.type()) {
    case ThunderAutoModeStepType::ACTION: {
      const auto& actionStep = static_cast<const ThunderAutoModeActionStep&>(step);
      json["action"] = actionStep.actionName;
      break;
    }
    case ThunderAutoModeStepType::TRAJECTORY: {
      const auto& trajectoryStep = static_cast<const ThunderAutoModeTrajectoryStep&>(step);
      json["trajectory"] = trajectoryStep.trajectoryName;
      break;
    }
    case ThunderAutoModeStepType::BRANCH_BOOL: {
      const auto& branchBoolStep = static_cast<const ThunderAutoModeBoolBranchStep&>(step);

      if (!branchBoolStep.trueBranch.empty()) {
        json["true_branch"] = branchBoolStep.trueBranch;
      }
      if (!branchBoolStep.elseBranch.empty()) {
        json["else_branch"] = branchBoolStep.elseBranch;
      }

      break;
    }
    case ThunderAutoModeStepType::BRANCH_SWITCH: {
      const auto& branchSwitchStep = static_cast<const ThunderAutoModeSwitchBranchStep&>(step);

      if (!branchSwitchStep.caseBranches.empty()) {
        wpi::json caseBranchesJson;
        for (const auto& [key, value] : branchSwitchStep.caseBranches) {
          caseBranchesJson[std::to_string(key)] = value;
        }
        json["case_branches"] = caseBranchesJson;
      }
      if (!branchSwitchStep.defaultBranch.empty()) {
        json["default_branch"] = branchSwitchStep.defaultBranch;
      }

      break;
    }
    default:
      ThunderLibUnreachable("Unknown ThunderAutoModeStepType");
  }
}

static void to_json(wpi::json& json, const std::shared_ptr<ThunderAutoModeStep>& step) {
  ThunderLibAssert(step != nullptr, "Step is null?");
  json = *step;
}

static void from_json(const wpi::json& json, std::shared_ptr<ThunderAutoModeStep>& step) {
  ThunderAutoModeStepType stepType = json.at("type").get<ThunderAutoModeStepType>();
  switch (stepType) {
    case ThunderAutoModeStepType::ACTION: {
      auto actionStep = std::make_shared<ThunderAutoModeActionStep>();
      json.at("action").get_to(actionStep->actionName);
      step = actionStep;
      break;
    }
    case ThunderAutoModeStepType::TRAJECTORY: {
      auto trajectoryStep = std::make_shared<ThunderAutoModeTrajectoryStep>();
      json.at("trajectory").get_to(trajectoryStep->trajectoryName);
      step = trajectoryStep;
      break;
    }
    case ThunderAutoModeStepType::BRANCH_BOOL: {
      auto branchBoolStep = std::make_shared<ThunderAutoModeBoolBranchStep>();

      if (json.contains("true_branch")) {
        json.at("true_branch").get_to(branchBoolStep->trueBranch);
      }
      if (json.contains("else_branch")) {
        json.at("else_branch").get_to(branchBoolStep->elseBranch);
      }

      step = branchBoolStep;
      break;
    }
    case ThunderAutoModeStepType::BRANCH_SWITCH: {
      auto branchSwitchStep = std::make_shared<ThunderAutoModeSwitchBranchStep>();

      if (json.contains("case_branches")) {
        const wpi::json& nextStepsJson = json.at("case_branches");
        for (const auto& [key, value] : nextStepsJson.items()) {
          int caseValue = std::stoi(key);
          branchSwitchStep->caseBranches[caseValue] = value;
        }
      }
      if (json.contains("default_branch")) {
        json.at("default_branch").get_to(branchSwitchStep->defaultBranch);
      }

      step = branchSwitchStep;
      break;
    }
    default:
      ThunderLibUnreachable("Unknown ThunderAutoModeStepType");
  }
}

void to_json(wpi::json& json, const ThunderAutoMode& mode) {
  json["steps"] = mode.steps;
}

void from_json(const wpi::json& json, ThunderAutoMode& mode) {
  json.at("steps").get_to(mode.steps);
}

}  // namespace thunder::core
