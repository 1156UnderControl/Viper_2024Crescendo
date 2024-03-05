// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Java_Is_UnderControl.Limelight;

import edu.wpi.first.math.util.Units;

public final class Limelight {
  public Limelight() {
  }

  public static boolean notUsingLLforOdometry = false;

  public double getTX() {
    return LimelightHelpers.getTX("");
  }

  public boolean getTV() {
    return LimelightHelpers.getTV("");
  }

  public static void setPipeline(int index) {
    LimelightHelpers.setPipelineIndex("", index);
  }

  public static void ledBlink() {
    LimelightHelpers.setLEDMode_ForceBlink("");
    ;
  }

  public static void ledOn() {
    LimelightHelpers.setLEDMode_ForceBlink("");
    ;
  }

  public static void ledOff() {
    LimelightHelpers.setLEDMode_ForceOff("");
    ;
  }

  private static final double limelightMountAngleDegrees = 20;

  private static final double limelightLensHeightInches = 10.2362;

  private static final double goalHeightInches = 53.88;

  public double getDistance() {
    double angleToGoalDegrees = limelightMountAngleDegrees + LimelightHelpers.getTY("");
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
        / Math.tan(angleToGoalRadians);
    return Units.inchesToMeters(distanceFromLimelightToGoalInches);
  }
}
