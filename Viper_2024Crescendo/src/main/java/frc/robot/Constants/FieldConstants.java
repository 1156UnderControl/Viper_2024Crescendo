// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class FieldConstants {
  public static Translation2d blueGoalPos = new Translation2d(0.076, 5.547868);
  public static Translation2d redGoalPos = new Translation2d(16.465042, 5.547868);
  public static Pose2d blueAmp = new Pose2d(1.82, 7.7, new Rotation2d());
  public static Pose2d redAmp = new Pose2d(14.73, 7.7, new Rotation2d());
}
