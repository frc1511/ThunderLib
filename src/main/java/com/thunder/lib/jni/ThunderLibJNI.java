package com.thunder.lib.jni;

import java.lang.Runnable;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Demo class for loading the driver via JNI.
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
      System.loadLibrary("ThunderLibCore");
      libraryLoaded = true;
    }
  }

  public static synchronized void forceLoad() {
    if (libraryLoaded) {
      return;
    }
    System.loadLibrary("ThunderLibCore");
    libraryLoaded = true;
  }

  /**
   * Native functions.
   */
}

