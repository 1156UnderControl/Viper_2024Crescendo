// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.util.Units;
import frc.Java_Is_UnderControl.Limelight.LimelightHelpers;

public final class LimelightConstants {
  private LimelightConstants() {
  }

  public static final double limelightMountAngleDegrees = 20;

  public static final double limelightLensHeightInches = 10.2362;

  public static final double goalHeightInches = 53.88;

  public static double getDistance() {
    double angleToGoalDegrees = limelightMountAngleDegrees + LimelightHelpers.getTY("");
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
        / Math.tan(angleToGoalRadians);

    return Units.inchesToMeters(distanceFromLimelightToGoalInches);

  }
}
