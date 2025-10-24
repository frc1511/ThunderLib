#pragma once

#include <ThunderLibDriver/Auto/ThunderAutoProject.hpp>
#include <frc/smartdashboard/SendableChooserBase.h>
#include <wpi/sendable/SendableBuilder.h>
#include <string>
#include <unordered_map>

namespace thunder::driver {

class ThunderAutoSendableChooser : private frc::SendableChooserBase {
 public:
  ThunderAutoSendableChooser() noexcept;
  explicit ThunderAutoSendableChooser(std::string_view smartDashboardKey) noexcept;

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

  enum class ChooserSelectionType {
    NONE,
    AUTO_MODE,
    TRAJECTORY,
    CUSTOM_COMMAND,
  };

  struct ChooserSelection {
    ChooserSelectionType type = ChooserSelectionType::NONE;
    std::string projectName;
    std::string itemName;
  };

  ChooserSelection getSelected() const noexcept;

 private:
  bool addItem(const std::string& projectName,
               const std::string& itemName,
               ChooserSelectionType type) noexcept;

  void removeItem(const std::string& itemName) noexcept;

  void republishIfNecessary() noexcept;

  void projectWasUpdated(const std::string& projectName,
                         const std::unordered_set<std::string>& updatedTrajectories,
                         const std::unordered_set<std::string>& removedTrajectories,
                         const std::unordered_set<std::string>& updatedAutoModes,
                         const std::unordered_set<std::string>& removedAutoModes) noexcept;

 private:
  std::function<void(ChooserSelection)> m_listener;
  std::unordered_map<std::string, ChooserSelection> m_choices;

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

 private:
  void InitSendable(wpi::SendableBuilder& builder);
};

}  // namespace thunder::driver
