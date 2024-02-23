
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Java_Is_UnderControl.Limelight.LimelightHelpers;
import frc.robot.RobotSubsystems;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve.ISwerve;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class SwerveAlignSpeaker extends Command {
  private ISwerve swerve;

  public SwerveAlignSpeaker(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    Limelight.notUsingLLforOdometry = true;
    Limelight.setPipelineToTrackSpeaker();
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getCurrentPipelineIndex("") == 1) {
      swerve.driveAlignSpeaker();
    } else {
      swerve.driveAlignAngleButton();
    }
  }

  @Override
  public void end(boolean interrupted) {
    Limelight.notUsingLLforOdometry = false;
  }

  @Override
  public boolean isFinished() {
    return RobotSubsystems.shootFinished;
  }
}
