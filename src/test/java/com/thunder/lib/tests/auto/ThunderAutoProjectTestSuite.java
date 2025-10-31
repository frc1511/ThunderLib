package com.thunder.lib.tests.auto;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

import com.thunder.lib.auto.ThunderAutoProject;
import com.thunder.lib.tests.BaseTestSuite;

public class ThunderAutoProjectTestSuite extends BaseTestSuite {
  @Test
  void simpleTrajectoryTest() {
    String projectPath = TEST_RESOURCES_DIR + "/ThunderAuto/SimpleTrajectory.thunderauto";
    ThunderAutoProject project = new ThunderAutoProject(projectPath);
    assertTrue(project.isLoaded());

    // TODO: More
  }
}
