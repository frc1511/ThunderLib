package com.thunder.lib.trajectory;

/**
 * Represents the dimensions of a FRC field.
 */
public class FieldDimensions {
  /**
   * Length of the field in meters (Y dimension).
   */
  public final double length;

  /**
   * Width of the field in meters (X dimension).
   */
  public final double width;

  /**
   * Constructs FieldDimensions with the specified length and width.
   * 
   * @param length Length in meters.
   * @param width  Width in meters.
   */
  public FieldDimensions(double length, double width) {
    this.length = length;
    this.width = width;
  }
}
