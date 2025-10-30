package com.thunder.lib.jni;

import java.lang.Runnable;
import java.util.HashSet;
import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.math.Pair;

/**
 * Class for loading the driver via JNI.
 * 
 * Do not interact with this class directly, use the wrapper classes instead.
 */
public class ThunderLibJNI {
  static boolean libraryLoaded = false;

  public static class Helper {
    private static AtomicBoolean extractOnStaticLoad = new AtomicBoolean(true);

    public static boolean getExtractOnStaticLoad() {
      return extractOnStaticLoad.get();
    }

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

  public static synchronized void forceLoad() {
    if (libraryLoaded) {
      return;
    }
    System.loadLibrary("ThunderLibDriver");
    libraryLoaded = true;
  }

  // Native Functions

  class ThunderAutoProject {
    public static native long construct();

    public static native long constructWithPath(String projectPath);

    public static native void delete(long handle);

    public static native boolean load(long handle, String projectPath);

    public static native boolean discoverAndLoadFromDeployDirectory(long handle);

    public static native boolean isLoaded(long handle);

    public static native String getName(long handle);

    public static native boolean hasAction(long handle, String actionName);

    public static native long getTrajectory(long handle, String trajectoryName);

    public static native boolean hasTrajectory(long handle, String trajectoryName);

    public static native HashSet<String> getTrajectoryNames(long handle);

    public static native long getAutoMode(long handle, String autoModeName);

    public static native boolean hasAutoMode(long handle, String autoModeName);

    public static native HashSet<String> getAutoModeNames(long handle);

    public static native int getFieldSymmetry(long handle);

    public static native Pair<Double, Double> getFieldDimensions(long handle);

    public static native void setRemoteUpdatesEnabled(long handle, boolean enabled);

    public static native void enableRemoteUpdates(long handle);

    public static native void disableRemoteUpdates(long handle);

    public static native boolean areRemoteUpdatesEnabled(long handle);

    public static native long registerRemoteUpdateSubscriber(long handle, Runnable callback);

    public static native boolean unregisterRemoteUpdateSubscriber(long handle, long subscriberID);
  }
}
