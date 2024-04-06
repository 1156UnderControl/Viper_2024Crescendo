
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;
import frc.robot.subsystems.Swerve.ISwerve;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class SwerveAlignShootOverStage extends Command {
  private ISwerve swerve;

  public SwerveAlignShootOverStage(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    swerve.driveAlignShootOverStage();
  }

  @Override
  public boolean isFinished() {
    return RobotSubsystems.shootFinished;
  }
}
