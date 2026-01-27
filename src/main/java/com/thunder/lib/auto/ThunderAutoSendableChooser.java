package com.thunder.lib.auto;

import java.lang.ref.Cleaner;
import java.util.HashMap;
import java.util.Optional;

import com.thunder.lib.commands.ThunderAutoModeCommand;
import com.thunder.lib.commands.ThunderAutoTrajectoryCommand;
import com.thunder.lib.jni.ThunderLibJNI;
import com.thunder.lib.trajectory.ThunderTrajectoryRunnerProperties;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * A wrapper around SendableChooser to handle ThunderAuto trajectory, auto mode,
 * and command selections.
 *
 * This chooser will automatically update its options when the associated
 * project gets updated.
 */
public class ThunderAutoSendableChooser {
  /**
   * Default constructor. The chooser will not be published to SmartDashboard
   * until publish() is called.
   */
  public ThunderAutoSendableChooser() {
    m_handle = ThunderLibJNI.ThunderAutoSendableChooser_construct(
        this::addChooserSelection, this::publishChooser);
    m_cleaner.register(this, new ThunderAutoSendableChooserCleanup(m_handle));
    m_chooser.setDefaultOption("Do Nothing", new ChooserSelection());
  }

  /**
   * Constructs an empty chooser and publishes it to SmartDashboard under the
   * given key.
   *
   * @param smartDashboardKey The key in SmartDashboard to publish the chooser
   *                          under.
   */
  public ThunderAutoSendableChooser(String smartDashboardKey) {
    m_handle = ThunderLibJNI.ThunderAutoSendableChooser_constructWithSmartDashboardKey(
        this::addChooserSelection, this::publishChooser, smartDashboardKey);
    m_cleaner.register(this, new ThunderAutoSendableChooserCleanup(m_handle));
    m_chooser.setDefaultOption("Do Nothing", new ChooserSelection());
  }

  /**
   * Constructs a chooser with the given trajectory runner properties.
   * The chooser will not be published to SmartDashboard until publish() is
   * called.
   *
   * @param runnerProps The trajectory runner properties to use for making
   *                    trajectory commands.
   */
  public ThunderAutoSendableChooser(ThunderTrajectoryRunnerProperties runnerProps) {
    m_handle = ThunderLibJNI.ThunderAutoSendableChooser_construct(
        this::addChooserSelection, this::publishChooser);
    m_cleaner.register(this, new ThunderAutoSendableChooserCleanup(m_handle));
    m_runnerProps = Optional.of(runnerProps);
    m_chooser.setDefaultOption("Do Nothing", new ChooserSelection());
  }

  /**
   * Constructs a chooser with the given trajectory runner properties and
   * publishes it to SmartDashboard under
   * the given key.
   *
   * @param smartDashboardKey The key in SmartDashboard to publish the chooser
   *                          under.
   * @param runnerProps       The trajectory runner properties to use for making
   *                          trajectory commands.
   */
  public ThunderAutoSendableChooser(String smartDashboardKey, ThunderTrajectoryRunnerProperties runnerProps) {
    m_handle = ThunderLibJNI.ThunderAutoSendableChooser_constructWithSmartDashboardKey(
        this::addChooserSelection, this::publishChooser, smartDashboardKey);
    m_cleaner.register(this, new ThunderAutoSendableChooserCleanup(m_handle));
    m_runnerProps = Optional.of(runnerProps);
    m_chooser.setDefaultOption("Do Nothing", new ChooserSelection());
  }

  /**
   * Sets the trajectory runner properties to use for making trajectory commands.
   *
   * @param runnerProps The trajectory runner properties to set.
   */
  public void setTrajectoryRunnerProperties(ThunderTrajectoryRunnerProperties runnerProps) {
    m_runnerProps = Optional.of(runnerProps);
  }

  /**
   * Publishes the chooser to SmartDashboard. This only needs to be called once,
   * as the chooser will
   * automatically refresh the chooser when changes are made.
   *
   * @param smartDashboardKey The key to publish the chooser under.
   */
  public void publish(String smartDashboardKey) {
    ThunderLibJNI.ThunderAutoSendableChooser_publish(m_handle, smartDashboardKey);
  }

  /**
   * Include a ThunderAuto project as a source for trajectories and auto modes.
   * Trajectories and auto modes
   * from this project can later be added to the chooser using the appropriate
   * methods.
   *
   * @param project The ThunderAuto project to include.
   */
  public void includeProjectSource(ThunderAutoProject project) {
    ThunderLibJNI.ThunderAutoSendableChooser_includeProjectSource(m_handle, project.getHandle(), false, false);
    m_includedProjects.put(project.getName(), project);
  }

  /**
   * Include a ThunderAuto project as a source for trajectories and auto modes.
   * Trajectories and auto modes
   * from this project can later be added to the chooser using the appropriate
   * methods.
   *
   * @param project            The ThunderAuto project to include.
   * @param addAllAutoModes    If true, all auto modes from the project will be
   *                           added to the chooser.
   * @param addAllTrajectories If true, all trajectories from the project will be
   *                           added to the chooser.
   */
  public void includeProjectSource(ThunderAutoProject project,
      boolean addAllAutoModes,
      boolean addAllTrajectories) {
    ThunderLibJNI.ThunderAutoSendableChooser_includeProjectSource(m_handle, project.getHandle(), addAllAutoModes,
        addAllTrajectories);
    m_includedProjects.put(project.getName(), project);
  }

  /**
   * Add all trajectories from a previously included project to the chooser.
   *
   * @param projectName The name of the project to add trajectories from. This
   *                    must match the name of a
   *                    project previously included using includeProjectSource().
   */
  public void addAllTrajectoriesFromProject(String projectName) {
    ThunderLibJNI.ThunderAutoSendableChooser_addAllTrajectoriesFromProject(m_handle, projectName);
  }

  /**
   * Add all auto modes from a previously included project to the chooser.
   *
   * @param projectName The name of the project to add auto modes from. This must
   *                    match the name of a
   *                    project previously included using includeProjectSource().
   */
  public void addAllAutoModesFromProject(String projectName) {
    ThunderLibJNI.ThunderAutoSendableChooser_addAllAutoModesFromProject(m_handle, projectName);
  }

  /**
   * Add a trajectory from a previously included project to the chooser.
   *
   * @param projectName    The name of the project to add the trajectory from.
   *                       This must match the name of a
   *                       project previously included using
   *                       includeProjectSource().
   * @param trajectoryName The name of the trajectory to add from the project.
   *
   * @return True if the trajectory was successfully added, false if the project
   *         or trajectory was not found,
   *         or another item with the same name was already added to the chooser.
   */
  public boolean addTrajectoryFromProject(String projectName, String trajectoryName) {
    return ThunderLibJNI.ThunderAutoSendableChooser_addTrajectoryFromProject(m_handle, projectName, trajectoryName);
  }

  /**
   * Add an auto mode from a previously included project to the chooser.
   *
   * @param projectName  The name of the project to add the auto mode from. This
   *                     must match the name of a
   *                     project previously included using includeProjectSource().
   * @param autoModeName The name of the auto mode to add from the project.
   *
   * @return True if the auto mode was successfully added, false if the project or
   *         auto mode was not found, or
   *         another item with the same name was already added to the chooser.
   */
  public boolean addAutoModeFromProject(String projectName, String autoModeName) {
    return ThunderLibJNI.ThunderAutoSendableChooser_addAutoModeFromProject(m_handle, projectName, autoModeName);
  }

  /**
   * Add a custom command to the chooser.
   *
   * @param name    The name of the custom command.
   * @param command The command to add.
   *
   * @return True if the command was successfully added, false if another item
   *         with the same name was already
   *         added to the chooser.
   */
  public boolean addCustomCommand(String name, Command command) {
    boolean success = ThunderLibJNI.ThunderAutoSendableChooser_addCustomCommand(m_handle, name);
    if (success) {
      m_customCommands.put(name, command);
    }
    return success;
  }

  /**
   * Get the currently selected command from the chooser.
   *
   * @return The selected command.
   */
  public Command getSelectedCommand() {
    ChooserSelection selection = getSelected();

    if (selection.type == ChooserSelection.Type.CUSTOM_COMMAND) {
      if (m_customCommands.containsKey(selection.itemName)) {
        return m_customCommands.get(selection.itemName);
      }
    } else if (selection.type != ChooserSelection.Type.NONE && m_runnerProps.isPresent()) {
      if (m_includedProjects.containsKey(selection.projectName)) {
        ThunderAutoProject project = m_includedProjects.get(selection.projectName);

        if (selection.type == ChooserSelection.Type.AUTO_MODE) {
          return new ThunderAutoModeCommand(selection.itemName, project, m_runnerProps.get());
        } else if (selection.type == ChooserSelection.Type.TRAJECTORY) {
          return new ThunderAutoTrajectoryCommand(selection.itemName, project, m_runnerProps.get());
        }
      }
    }

    return Commands.none();
  }

  /**
   * Represents the current selection in the chooser.
   */
  public static class ChooserSelection {
    /**
     * What type of item is selected
     */
    public static enum Type {
      /**
       * Nothing selected
       */
      NONE,

      /**
       * An auto mode
       */
      AUTO_MODE,

      /**
       * A trajectory
       */
      TRAJECTORY,

      /**
       * A custom command, added via addCustomCommand()
       */
      CUSTOM_COMMAND,
    }

    /**
     * The type of item selected
     */
    public final Type type;

    /**
     * The name of the project the selected item belongs to (only for auto modes and
     * trajectories)
     */
    public final String projectName;

    /**
     * The name of the selected item
     */
    public final String itemName;

    /**
     * Default constructor for no selection
     */
    public ChooserSelection() {
      this(Type.NONE, "", "");
    }

    /**
     * Constructor
     * 
     * @param type        The type of item selected
     * @param projectName The name of the project the selected item belongs to
     * @param itemName    The name of the selected item
     */
    public ChooserSelection(Type type, String projectName, String itemName) {
      this.type = type;
      this.projectName = projectName;
      this.itemName = itemName;
    }
  };

  /**
   * Get the currently selected item from the chooser.
   * 
   * @return The selected item.
   */
  public ChooserSelection getSelected() {
    return m_chooser.getSelected();
  }

  /**
   * Get the native handle for this ThunderAutoSendableChooser.
   * 
   * @return The native handle.
   */
  long getHandle() {
    return m_handle;
  }

  private void addChooserSelection(ChooserSelection selection) {
    m_chooser.addOption(selection.itemName, selection);
  }

  private void publishChooser(String smartDashboardKey) {
    SmartDashboard.putData(smartDashboardKey, m_chooser);
    SmartDashboard.updateValues();
  }

  private long m_handle = 0;
  private SendableChooser<ChooserSelection> m_chooser = new SendableChooser<>();

  private Optional<ThunderTrajectoryRunnerProperties> m_runnerProps = Optional.empty();
  private HashMap<String, ThunderAutoProject> m_includedProjects = new HashMap<>();
  private HashMap<String, Command> m_customCommands = new HashMap<>();

  private final Cleaner m_cleaner = Cleaner.create();

  private static class ThunderAutoSendableChooserCleanup implements Runnable {
    private final long m_handle;

    ThunderAutoSendableChooserCleanup(long handle) {
      m_handle = handle;
    }

    @Override
    public void run() {
      ThunderLibJNI.ThunderAutoSendableChooser_delete(m_handle);
    }
  }

}
