#include <ThunderLibDriver/Auto/ThunderAutoSendableChooser.hpp>
#include <vector>
#include <string_view>

namespace thunder::driver {

ThunderAutoSendableChooser::ThunderAutoSendableChooser(AddChooserSelectionFunc addChooserSelectionFunc,
                                                       PublishChooserFunc publishChooserFunc) noexcept
    : m_addChooserSelectionFunc(addChooserSelectionFunc), m_publishChooserFunc(publishChooserFunc) {}

ThunderAutoSendableChooser::ThunderAutoSendableChooser(AddChooserSelectionFunc addChooserSelectionFunc,
                                                       PublishChooserFunc publishChooserFunc,
                                                       std::string_view smartDashboardKey) noexcept
    : ThunderAutoSendableChooser(addChooserSelectionFunc, publishChooserFunc) {
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
  m_isPublished = true;
  m_smartDashboardKey = smartDashboardKey;

  if (m_publishChooserFunc) {
    m_publishChooserFunc(m_smartDashboardKey);
  }
}

void ThunderAutoSendableChooser::republishIfNecessary() noexcept {
  if (m_isPublished) {
    if (m_publishChooserFunc) {
      m_publishChooserFunc(m_smartDashboardKey);
    }
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

  bool result = addItem(projectName, trajectoryName, ThunderAutoSendableChooserSelectionType::TRAJECTORY);
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

  bool result = addItem(projectName, autoModeName, ThunderAutoSendableChooserSelectionType::AUTO_MODE);
  if (result) {
    republishIfNecessary();
  }
  return result;
}

bool ThunderAutoSendableChooser::addCustomCommand(const std::string& name) noexcept {
  if (name.empty()) {
    return false;
  }

  bool result = addItem("", name, ThunderAutoSendableChooserSelectionType::CUSTOM_COMMAND);
  if (result) {
    republishIfNecessary();
  }
  return result;
}

bool ThunderAutoSendableChooser::addItem(const std::string& projectName,
                                         const std::string& itemName,
                                         ThunderAutoSendableChooserSelectionType type) noexcept {
  if (itemName == "Do Nothing") {  // Reserved
    return false;
  }

  auto it = m_choices.find(itemName);
  if (it != m_choices.end()) {
    // A different item type with the same name already exists in the chooser, so this item cannot be added.
    if (it->second.projectName != projectName || it->second.type != type) {
      return false;
    }
  }

  ThunderAutoSendableChooserSelection selection;
  selection.type = type;
  selection.projectName = projectName;
  selection.itemName = itemName;
  if (m_addChooserSelectionFunc) {
    m_addChooserSelectionFunc(selection);
  }
  m_choices[itemName] = selection;

  return true;
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
    // removeItem(trajectoryName);
  }

  for (const auto& autoModeName : removedAutoModes) {
    source.addedAutoModes.erase(autoModeName);
    // removeItem(autoModeName);
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

}  // namespace thunder::driver
