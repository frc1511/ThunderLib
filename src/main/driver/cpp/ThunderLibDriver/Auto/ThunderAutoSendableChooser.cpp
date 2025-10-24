#include <ThunderLibDriver/Auto/ThunderAutoSendableChooser.hpp>
#include <frc/smartdashboard/SmartDashboard.h>
#include <vector>
#include <string_view>

namespace thunder::driver {

ThunderAutoSendableChooser::ThunderAutoSendableChooser() noexcept {
  m_defaultChoice = "Do Nothing";
  m_choices[m_defaultChoice] = ChooserSelection{};
}

ThunderAutoSendableChooser::ThunderAutoSendableChooser(std::string_view smartDashboardKey) noexcept
    : ThunderAutoSendableChooser() {
  publish(smartDashboardKey);
}

ThunderAutoSendableChooser::~ThunderAutoSendableChooser() noexcept {
  for (auto& [name, source] : m_projectSources) {
    if (source.project != nullptr && source.subscriberID != 0) {
      source.project->unregisterRemoteUpdateSubscriber(source.subscriberID);
    }
  }
}

void ThunderAutoSendableChooser::publish(std::string_view smartDashboardKey) noexcept {
  frc::SmartDashboard::PutData(smartDashboardKey, this);
  m_isPublished = true;
  m_smartDashboardKey = smartDashboardKey;
}

void ThunderAutoSendableChooser::republishIfNecessary() noexcept {
  if (m_isPublished) {
    frc::SmartDashboard::PutData(m_smartDashboardKey, this);
  }
}

void ThunderAutoSendableChooser::includeProjectSource(ThunderAutoProject* project,
                                                      bool addAllAutoModes,
                                                      bool addAllTrajectories) noexcept {
  if (!project || !project->isLoaded()) {
    return;
  }

  std::string projectName = project->getName();

  if (m_projectSources.contains(projectName)) {
    return;
  }

  ProjectSource source;
  source.project = project;
  {
    using namespace std::placeholders;

    ThunderAutoProject::RemoteUpdateCallbackFunc callback =
        std::bind(&ThunderAutoSendableChooser::projectWasUpdated, this, projectName, _1, _2, _3, _4);

    source.subscriberID = project->registerRemoteUpdateSubscriber(callback);
  }

  m_projectSources.emplace(projectName, source);

  if (addAllAutoModes) {
    addAllAutoModesFromProject(projectName);
  }
  if (addAllTrajectories) {
    addAllTrajectoriesFromProject(projectName);
  }
}

void ThunderAutoSendableChooser::addAllTrajectoriesFromProject(const std::string& projectName) noexcept {
  auto it = m_projectSources.find(projectName);
  if (it == m_projectSources.end()) {
    return;
  }

  ProjectSource& source = it->second;
  source.addAllTrajectories = true;

  const ThunderAutoProject* project = source.project;
  std::unordered_set<std::string> trajectories = project->getTrajectoryNames();

  for (const auto& trajectoryName : trajectories) {
    addTrajectoryFromProject(projectName, trajectoryName);
  }
}

void ThunderAutoSendableChooser::addAllAutoModesFromProject(const std::string& projectName) noexcept {
  auto it = m_projectSources.find(projectName);
  if (it == m_projectSources.end()) {
    return;
  }

  ProjectSource& source = it->second;
  source.addAllAutoModes = true;

  const ThunderAutoProject* project = source.project;
  std::unordered_set<std::string> autoModes = project->getAutoModeNames();

  for (const auto& autoModeName : autoModes) {
    addAutoModeFromProject(projectName, autoModeName);
  }
}

bool ThunderAutoSendableChooser::addTrajectoryFromProject(const std::string& projectName,
                                                          const std::string& trajectoryName) noexcept {
  auto it = m_projectSources.find(projectName);
  if (it == m_projectSources.end()) {
    return false;
  }

  ProjectSource& source = it->second;
  source.addedTrajectories.insert(trajectoryName);

  bool result = addItem(projectName, trajectoryName, ChooserSelectionType::TRAJECTORY);
  if (result) {
    republishIfNecessary();
  }
  return result;
}

bool ThunderAutoSendableChooser::addAutoModeFromProject(const std::string& projectName,
                                                        const std::string& autoModeName) noexcept {
  auto it = m_projectSources.find(projectName);
  if (it == m_projectSources.end()) {
    return false;
  }

  ProjectSource& source = it->second;
  source.addedAutoModes.insert(autoModeName);

  bool result = addItem(projectName, autoModeName, ChooserSelectionType::AUTO_MODE);
  if (result) {
    republishIfNecessary();
  }
  return result;
}

bool ThunderAutoSendableChooser::addCustomCommand(const std::string& name) noexcept {
  if (name.empty()) {
    return false;
  }

  bool result = addItem("", name, ChooserSelectionType::CUSTOM_COMMAND);
  if (result) {
    republishIfNecessary();
  }
  return result;
}

ThunderAutoSendableChooser::ChooserSelection ThunderAutoSendableChooser::getSelected() const noexcept {
  // frc::SendableChooser<T>::GetSelected()

  std::lock_guard<wpi::mutex> lock(m_mutex);

  std::string selected = m_haveSelected ? m_selected : m_defaultChoice;

  if (selected.empty()) {
    return ChooserSelection{};
  }

  auto it = m_choices.find(selected);
  if (it == m_choices.end()) {
    return ChooserSelection{};
  }
  return it->second;
}

bool ThunderAutoSendableChooser::addItem(const std::string& projectName,
                                         const std::string& itemName,
                                         ChooserSelectionType type) noexcept {
  std::lock_guard<wpi::mutex> lock(m_mutex);

  auto it = m_choices.find(itemName);
  if (it != m_choices.end()) {
    // A different item type with the same name already exists in the chooser, so this item cannot be added.
    if (it->second.projectName != projectName || it->second.type != type) {
      return false;
    }
  }

  ChooserSelection selection;
  selection.type = ChooserSelectionType::TRAJECTORY;
  selection.projectName = projectName;
  selection.itemName = itemName;
  m_choices.emplace(itemName, selection);

  return true;
}

void ThunderAutoSendableChooser::removeItem(const std::string& itemName) noexcept {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  m_choices.erase(itemName);
}

void ThunderAutoSendableChooser::projectWasUpdated(
    const std::string& projectName,
    const std::unordered_set<std::string>& updatedTrajectories,
    const std::unordered_set<std::string>& removedTrajectories,
    const std::unordered_set<std::string>& updatedAutoModes,
    const std::unordered_set<std::string>& removedAutoModes) noexcept {
  auto it = m_projectSources.find(projectName);
  if (it == m_projectSources.end()) {
    return;
  }

  ProjectSource& source = it->second;

  for (const auto& trajectoryName : removedTrajectories) {
    source.addedTrajectories.erase(trajectoryName);
    removeItem(trajectoryName);
  }

  for (const auto& autoModeName : removedAutoModes) {
    source.addedAutoModes.erase(autoModeName);
    removeItem(autoModeName);
  }

  if (source.addAllTrajectories) {
    addAllTrajectoriesFromProject(projectName);
  } else {
    for (const auto& trajectoryName : updatedTrajectories) {
      addTrajectoryFromProject(projectName, trajectoryName);
    }
  }

  if (source.addAllAutoModes) {
    addAllAutoModesFromProject(projectName);
  } else {
    for (const auto& autoModeName : updatedAutoModes) {
      addAutoModeFromProject(projectName, autoModeName);
    }
  }

  republishIfNecessary();
}

void ThunderAutoSendableChooser::InitSendable(wpi::SendableBuilder& builder) {
  // frc::SendableChooser<T>::InitSendable(builder)

  builder.SetSmartDashboardType("String Chooser");
  builder.PublishConstInteger(kInstance, m_instance);
  builder.AddStringArrayProperty(
      kOptions,
      [=, this] {
        std::lock_guard<wpi::mutex> lock(m_mutex);
        std::vector<std::string> keys;
        for (const auto& choice : m_choices) {
          keys.emplace_back(choice.first);
        }
        return keys;
      },
      nullptr);
  builder.AddSmallStringProperty(
      kDefault, [=, this](wpi::SmallVectorImpl<char>&) -> std::string_view { return m_defaultChoice; },
      nullptr);
  builder.AddSmallStringProperty(
      kActive,
      [=, this](wpi::SmallVectorImpl<char>& buf) -> std::string_view {
        std::lock_guard<wpi::mutex> lock(m_mutex);
        if (m_haveSelected) {
          buf.assign(m_selected.begin(), m_selected.end());
          return {buf.data(), buf.size()};
        } else {
          return m_defaultChoice;
        }
      },
      nullptr);
  builder.AddStringProperty(kSelected, nullptr, [=, this](std::string_view val) {
    ChooserSelection choice{};
    std::function<void(ChooserSelection)> listener;
    {
      std::lock_guard<wpi::mutex> lock(m_mutex);
      m_haveSelected = true;
      m_selected = val;
      if (m_previousVal != val && m_listener) {
        choice = m_choices[std::string(val)];
        listener = m_listener;
      }
      m_previousVal = val;
    }
    if (listener) {
      listener(choice);
    }
  });
}

}  // namespace thunder::driver
