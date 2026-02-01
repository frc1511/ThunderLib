#include <ThunderLib/Commands/ThunderAutoModeCommand.hpp>
#include <ThunderLib/Commands/ThunderAutoTrajectoryCommand.hpp>
#include <ThunderLibDriver/Logger.hpp>

namespace thunder {

ThunderAutoModeCommand::ThunderAutoModeCommand(const std::string& autoModeName,
                                               std::shared_ptr<ThunderAutoProject> project,
                                               const TrajectoryRunnerProperties& properties)
    : m_autoModeName(autoModeName), m_project(project), m_runnerProperties(properties) {
  m_runnerPropertiesNoResetPose = m_runnerProperties;
  m_runnerPropertiesNoResetPose.resetPose = nullptr;
}

bool ThunderAutoModeCommand::isValid() const {
  return m_project != nullptr && m_project->isLoaded() && m_autoMode != nullptr && m_autoMode->isValid() &&
         m_runnerProperties.isValid();
}

void ThunderAutoModeCommand::Initialize() {
  if (!m_project) {
    ThunderLibLogger::Error(
        "[ThunderAutoModeCommand] Cannot initialize command with null ThunderAutoProject for auto mode '{}'",
        m_autoModeName);
    return;
  }

  if (!m_project->isLoaded()) {
    ThunderLibLogger::Error(
        "[ThunderAutoModeCommand] Cannot initialize command with unloaded ThunderAutoProject '{}' for auto "
        "mode '{}'",
        m_project->getName(), m_autoModeName);
    return;
  }

  m_autoMode = m_project->getAutoMode(m_autoModeName);
  if (!m_autoMode || !m_autoMode->isValid()) {
    ThunderLibLogger::Error(
        "[ThunderAutoModeCommand] Cannot initialize command for auto mode '{}' that was not found in or "
        "could not be loaded from project '{}'",
        m_autoModeName, m_project->getName());
    return;
  }

  if (!m_runnerProperties.isValid()) {
    ThunderLibLogger::Error(
        "[ThunderAutoModeCommand] Invalid TrajectoryRunnerProperties provided to ThunderAutoModeCommand for "
        "auto mode '{}'",
        m_autoModeName);
    return;
  }

  if (!m_autoMode->isRunnable(*m_project)) {
    ThunderLibLogger::Error(
        "[ThunderAutoModeCommand] Auto mode '{}' of project '{}' is not runnable. It may reference missing "
        "trajectories, or one or more sequences of non-continuous trajectory steps. Open in ThunderAuto for "
        "more details.",
        m_autoModeName, m_project->getName());
    return;
  }

  if (!isValid())
    return;

  m_isFinished = false;
  m_currentStep = nullptr;
  m_firstTrajectoryWasSeen = false;
  nextStep();
}

void ThunderAutoModeCommand::Execute() {
  if (!isValid())
    return;

  if (m_isFinished || !m_currentStep)
    return;

  m_currentStepCommand.get()->Execute();

  if (m_currentStepCommand.get()->IsFinished()) {
    m_currentStepCommand.get()->End(false);
    m_currentStepWasInitialized = false;
    nextStep();
  }
}

void ThunderAutoModeCommand::End(bool interrupted) {
  if (!isValid())
    return;

  if (m_isFinished || !m_currentStep)
    return;

  m_currentStepCommand.get()->End(interrupted);
  m_isFinished = true;
}

bool ThunderAutoModeCommand::IsFinished() {
  if (!isValid())
    return true;

  return m_isFinished;
}

void ThunderAutoModeCommand::nextStep() {
  if (!isValid())
    return;

  if (m_isFinished)
    return;

  if (m_currentStep) {
    m_currentStep = m_autoMode->getNextStep(m_currentStep);
  } else {
    m_currentStep = m_autoMode->getFirstStep();
  }

  if (!m_currentStep) {
    m_isFinished = true;
    return;
  }

  setupCurrentStep();

  if (!m_isFinished && m_currentStepCommand && !m_currentStepWasInitialized) {
    m_currentStepCommand.get()->Initialize();
    m_currentStepWasInitialized = true;  // Prevent double initialization during recursive calls.
  }
}

void ThunderAutoModeCommand::setupCurrentStep() {
  if (!m_currentStep)
    return;

  switch (m_currentStep->type()) {
    using enum ThunderAutoModeStepType;
    case ACTION: {
      std::string actionName = m_currentStep->getActionName();
      m_currentStepCommand = m_project->getActionCommand(actionName);
      break;
    }
    case TRAJECTORY: {
      std::string trajectoryName = m_currentStep->getTrajectoryName();

      // It's not usually in the user's best interests to reset pose during an auto mode, so just do it once.
      const TrajectoryRunnerProperties& runnerProperties =
          m_firstTrajectoryWasSeen ? m_runnerPropertiesNoResetPose : m_runnerProperties;
      m_firstTrajectoryWasSeen = true;

      ThunderAutoTrajectoryCommand trajectoryCommand(trajectoryName, m_project, runnerProperties);
      m_currentStepCommand = std::move(trajectoryCommand).ToPtr();
      break;
    }
    case BRANCH_BOOL: {
      std::string conditionName = m_currentStep->getConditionName();
      ThunderAutoProject::BooleanConditionFunc condition = m_project->getBooleanCondition(conditionName);
      if (!condition) {
        ThunderLibLogger::Error(
            "[ThunderAutoModeCommand] Boolean condition '{}' not registered to project '{}' required by auto "
            "mode '{}'. Stopping Auto Mode now",
            conditionName, m_project->getName(), m_autoModeName);
        m_isFinished = true;
        break;
      }

      bool conditionResult = condition();
      std::shared_ptr<ThunderAutoModeStep> branchStep =
          m_autoMode->getFirstStepOfBranch(m_currentStep, conditionResult);

      if (branchStep) {
        m_currentStep = branchStep;
        setupCurrentStep();
      } else {
        nextStep();
      }
      break;
    }
    case BRANCH_SWITCH: {
      std::string conditionName = m_currentStep->getConditionName();
      ThunderAutoProject::SwitchConditionFunc condition = m_project->getSwitchCondition(conditionName);
      if (!condition) {
        ThunderLibLogger::Error(
            "[ThunderAutoModeCommand] Switch condition '{}' not registered to project '{}' required by auto "
            "mode '{}'. Stopping Auto Mode now",
            conditionName, m_project->getName(), m_autoModeName);
        m_isFinished = true;
        return;
      }

      int conditionResult = condition();
      std::shared_ptr<ThunderAutoModeStep> branchStep =
          m_autoMode->getFirstStepOfBranch(m_currentStep, conditionResult);

      if (branchStep) {
        m_currentStep = branchStep;
        setupCurrentStep();
      } else {
        nextStep();
      }
      break;
    }
    default:
      m_isFinished = true;
      break;
  }
}

}  // namespace thunder
