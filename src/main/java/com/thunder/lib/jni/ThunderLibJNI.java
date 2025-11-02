package com.thunder.lib.jni;

import java.lang.Runnable;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.concurrent.atomic.AtomicBoolean;

import com.thunder.lib.trajectory.FieldSymmetry;
import com.thunder.lib.trajectory.ThunderTrajectoryState;

import edu.wpi.first.math.Pair;

/**
 * Class for loading the driver via JNI.
 * 
 * Do not interact with this class directly, use the wrapper classes instead.
 */
public class ThunderLibJNI {
  static boolean libraryLoaded = false;

  /**
   * Helper class for determining whether or not to load the driver on static
   * initialization.
   */
  public static class Helper {
    private static AtomicBoolean extractOnStaticLoad = new AtomicBoolean(true);

    /**
     * Get whether to load the driver on static init.
     * 
     * @return true if the driver will load on static init
     */
    public static boolean getExtractOnStaticLoad() {
      return extractOnStaticLoad.get();
    }

    /**
     * Set whether to load the driver on static init.
     * 
     * @param load the new value
     */
    public static void setExtractOnStaticLoad(boolean load) {
      extractOnStaticLoad.set(load);
    }
  }

  static {
    if (Helper.getExtractOnStaticLoad()) {
      System.loadLibrary("ThunderLibDriver");
      libraryLoaded = true;
    }
  }

  /**
   * Force load the library.
   */
  public static synchronized void forceLoad() {
    if (libraryLoaded) {
      return;
    }
    System.loadLibrary("ThunderLibDriver");
    libraryLoaded = true;
  }

  /**
   * ThunderAutoProject JNI methods
   */

  public static native long ThunderAutoProject_construct();

  public static native long ThunderAutoProject_constructWithPath(String projectPath);

  public static native void ThunderAutoProject_delete(long handle);

  public static native boolean ThunderAutoProject_load(long handle, String projectPath);

  public static native boolean ThunderAutoProject_discoverAndLoadFromDeployDirectory(long handle);

  public static native boolean ThunderAutoProject_isLoaded(long handle);

  public static native String ThunderAutoProject_getName(long handle);

  public static native boolean ThunderAutoProject_hasAction(long handle, String actionName);

  public static native boolean ThunderAutoProject_isActionCommand(long handle, String actionName);

  public static native boolean ThunderAutoProject_isActionGroup(long handle, String actionName);

  public static native ArrayList<String> ThunderAutoProject_getActionGroup(long handle, String actionName);

  public static native boolean ThunderAutoProject_isSequentialActionGroup(long handle, String actionName);

  public static native boolean ThunderAutoProject_isConcurrentActionGroup(long handle, String actionName);

  public static native long ThunderAutoProject_getTrajectory(long handle, String trajectoryName);

  public static native boolean ThunderAutoProject_hasTrajectory(long handle, String trajectoryName);

  public static native HashSet<String> ThunderAutoProject_getTrajectoryNames(long handle);

  public static native long ThunderAutoProject_getAutoMode(long handle, String autoModeName);

  public static native boolean ThunderAutoProject_hasAutoMode(long handle, String autoModeName);

  public static native HashSet<String> ThunderAutoProject_getAutoModeNames(long handle);

  public static native FieldSymmetry ThunderAutoProject_getFieldSymmetry(long handle);

  public static native Pair<Double, Double> ThunderAutoProject_getFieldDimensions(long handle);

  public static native void ThunderAutoProject_setRemoteUpdatesEnabled(long handle, boolean enabled);

  public static native void ThunderAutoProject_enableRemoteUpdates(long handle);

  public static native void ThunderAutoProject_disableRemoteUpdates(long handle);

  public static native boolean ThunderAutoProject_areRemoteUpdatesEnabled(long handle);

  public static native long ThunderAutoProject_registerRemoteUpdateSubscriber(long handle, Runnable callback);

  public static native boolean ThunderAutoProject_unregisterRemoteUpdateSubscriber(long handle, long subscriberID);

  /**
   * ThunderAutoTrajectory JNI Methods
   */

  public static native void ThunderAutoTrajectory_delete(long handle);

  public static native ThunderTrajectoryState ThunderAutoTrajectory_sample(long handle, double timeSeconds);

  public static native double ThunderAutoTrajectory_getDurationSeconds(long handle);

  public static native ThunderTrajectoryState ThunderAutoTrajectory_getInitialState(long handle);

  public static native ThunderTrajectoryState ThunderAutoTrajectory_getFinalState(long handle);

  public static native HashSet<String> ThunderAutoTrajectory_getStartActions(long handle);

  public static native HashSet<String> ThunderAutoTrajectory_getEndActions(long handle);

  public static native ArrayList<Double> ThunderAutoTrajectory_getStopTimes(long handle);

  public static native HashSet<String> ThunderAutoTrajectory_getStopActions(long handle, double stopTimeSeconds);

  public static native ArrayList<Double> ThunderAutoTrajectory_getActionTimes(long handle);

  public static native HashSet<String> ThunderAutoTrajectory_getActionsAtTime(long handle, double timeSeconds);
}
