package com.thunder.lib.trajectory;

/**
 * Field symmetry changes how trajectories are modified based on the selected
 * alliance color.
 */
public enum FieldSymmetry {
  /**
   * The trajectory remains the same regardless of alliance color.
   */
  NONE,

  /**
   * Rotated 180 degrees, like 2022, 2025, and 2026.
   * 
   * On red, both X and Y coordinates are inverted. Rotations are flipped by 180.
   */
  ROTATIONAL,

  /**
   * Mirrored across the center line, like 2023 and 2024.
   * 
   * On red, only the X coordinate is inverted. Rotations are flipped by 180
   */
  REFLECTIONAL,
}
