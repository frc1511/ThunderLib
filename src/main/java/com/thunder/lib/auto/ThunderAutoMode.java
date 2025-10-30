package com.thunder.lib.auto;

import java.lang.ref.Cleaner;

/**
 * Represents a ThunderAuto autonomous mode.
 */
public class ThunderAutoMode {
  ThunderAutoMode(long handle, ThunderAutoProject project) {
    m_handle = handle;
    m_project = project;
    m_cleaner.register(this, new ThunderAutoModeCleanup(m_handle));
  }

  // TODO: Methods

  private long m_handle;
  private ThunderAutoProject m_project;

  private final Cleaner m_cleaner = Cleaner.create();

  private static class ThunderAutoModeCleanup implements Runnable {
    private final long m_handle;

    ThunderAutoModeCleanup(long handle) {
      m_handle = handle;
    }

    @Override
    public void run() {
      // TODO
    }
  }
}
