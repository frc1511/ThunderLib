package com.thunder.lib.trajectory;

/**
 * Field symmetry changes how trajectories are modified based on the selected
 * alliance color.
 */
public enum FieldSymmetry {
  /**
   * The trajectory remains the same regardless of alliance color.
   */
  None,

  /**
   * Rotated 180 degrees, like 2022 and 2025.
   * 
   * On red, both X and Y coordinates are inverted. Rotations are flipped by 180.
   */
  Rotational,

  /**
   * Mirrored across the center line, like 2023 and 2024.
   * 
   * On red, only the Y coordinate is inverted. Rotations are flipped by 180
   */
  Reflectional,
}
