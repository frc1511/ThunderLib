package com.thunder.lib.tests;

import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * Base class for test suites.
 */
public abstract class BaseTestSuite {
  static final double DOUBLE_TOLERANCE = 1e-9;
  static final String TEST_RESOURCES_DIR = Paths.get("src", "test", "resources").toAbsolutePath().toString();
}
