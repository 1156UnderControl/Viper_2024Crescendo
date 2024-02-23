// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.Java_Is_UnderControl.Limelight.LimelightHelpers;

public final class Limelight {
  private Limelight() {
  }

  public static boolean notUsingLLforOdometry = false;

  public static double tx = LimelightHelpers.getTX("");
  public static boolean tv = LimelightHelpers.getTV("");
  public static boolean tvAfterTweak = false;
  public static Pose2d poseLL = LimelightHelpers.getBotPose2d_wpiBlue("");

  public static void setPipelineToOdometry() {
    LimelightHelpers.setPipelineIndex("", 0);
  }

  public static void setPipelineToTrackSpeaker() {
    LimelightHelpers.setPipelineIndex("", 1);
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

  public static double getDistance() {
    double angleToGoalDegrees = limelightMountAngleDegrees + LimelightHelpers.getTY("");
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
        / Math.tan(angleToGoalRadians);
    return Units.inchesToMeters(distanceFromLimelightToGoalInches);
  }
}
