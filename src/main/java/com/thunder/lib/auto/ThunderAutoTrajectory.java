package com.thunder.lib.auto;

import java.lang.ref.Cleaner;

import com.thunder.lib.trajectory.ThunderTrajectory;

/**
 * Represents a ThunderAuto trajectory.
 */
public class ThunderAutoTrajectory extends ThunderTrajectory {
  ThunderAutoTrajectory(long handle) {
    m_handle = handle;
    m_cleaner.register(this, new ThunderAutoTrajectoryCleanup(m_handle));
  }

  // TODO: Methods

  private long m_handle;

  private final Cleaner m_cleaner = Cleaner.create();

  private static class ThunderAutoTrajectoryCleanup implements Runnable {
    private final long m_handle;

    ThunderAutoTrajectoryCleanup(long handle) {
      m_handle = handle;
    }

    @Override
    public void run() {
      // TODO
    }
  }
}
