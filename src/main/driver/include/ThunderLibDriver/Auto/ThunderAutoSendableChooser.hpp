#pragma once

#include <ThunderLibDriver/Auto/ThunderAutoProject.hpp>
#include <string>
#include <unordered_map>

namespace thunder::driver {

  enum class ThunderAutoSendableChooserSelectionType {
    NONE,
    AUTO_MODE,
    TRAJECTORY,
    CUSTOM_COMMAND,
  };

  struct ThunderAutoSendableChooserSelection {
    ThunderAutoSendableChooserSelectionType type = ThunderAutoSendableChooserSelectionType::NONE;
    std::string projectName;
    std::string itemName;
  };

class ThunderAutoSendableChooser {
 public:
  // We cannot use frc::SendableChooser directly, so we need a function to add choices to the actual chooser.
  using AddChooserSelectionFunc = std::function<void(const ThunderAutoSendableChooserSelection&)>;
  using PublishChooserFunc = std::function<void(const std::string& key)>;

  ThunderAutoSendableChooser(AddChooserSelectionFunc addChooserSelectionFunc, PublishChooserFunc publishChooserFunc) noexcept;
  ThunderAutoSendableChooser(AddChooserSelectionFunc addChooserSelectionFunc, PublishChooserFunc publishChooserFunc, std::string_view smartDashboardKey) noexcept;

  ~ThunderAutoSendableChooser() noexcept;

  void publish(std::string_view smartDashboardKey) noexcept;

  void includeProjectSource(ThunderAutoProject* project,
                            bool addAllAutoModes,
                            bool addAllTrajectories) noexcept;

  void addAllTrajectoriesFromProject(const std::string& projectName) noexcept;
  void addAllAutoModesFromProject(const std::string& projectName) noexcept;

  bool addTrajectoryFromProject(const std::string& projectName, const std::string& trajectoryName) noexcept;
  bool addAutoModeFromProject(const std::string& projectName, const std::string& autoModeName) noexcept;

  bool addCustomCommand(const std::string& name) noexcept;

 private:
  bool addItem(const std::string& projectName,
               const std::string& itemName,
               ThunderAutoSendableChooserSelectionType type) noexcept;

  // void removeItem(const std::string& itemName) noexcept;

  void republishIfNecessary() noexcept;

  void projectWasUpdated(const std::string& projectName,
                         const std::unordered_set<std::string>& updatedTrajectories,
                         const std::unordered_set<std::string>& removedTrajectories,
                         const std::unordered_set<std::string>& updatedAutoModes,
                         const std::unordered_set<std::string>& removedAutoModes) noexcept;

 private:
  AddChooserSelectionFunc m_addChooserSelectionFunc;
  PublishChooserFunc m_publishChooserFunc;

  std::unordered_map<std::string, ThunderAutoSendableChooserSelection> m_choices;

  std::string m_smartDashboardKey;
  bool m_isPublished = false;

  struct ProjectSource {
    ThunderAutoProject* project = nullptr;
    size_t subscriberID = 0;
    std::unordered_set<std::string> addedTrajectories;
    std::unordered_set<std::string> addedAutoModes;
    bool addAllAutoModes = false;
    bool addAllTrajectories = false;
  };

  std::unordered_map<std::string, ProjectSource> m_projectSources;
};

}  // namespace thunder::driver
