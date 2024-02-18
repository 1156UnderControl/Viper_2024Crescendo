package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Camera.CameraSubsystem;
import frc.robot.subsystems.Camera.ICamera;
import frc.robot.subsystems.Intake.IIntake;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.IShooter;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class RobotSubsystems extends SubsystemBase {
  public IIntake intake;
  public IShooter shooter;
  public ICamera camera;

  private SubsystemBase intakeSubsystem;
  private SubsystemBase shooterSubsystem;
  private SubsystemBase cameraSubsystem;

  public RobotSubsystems() {
    this.intake = IntakeSubsystem.getInstance();
    this.shooter = ShooterSubsystem.getInstance();
    this.camera = CameraSubsystem.getInstance();
    this.intakeSubsystem = IntakeSubsystem.getInstance();
    this.shooterSubsystem = ShooterSubsystem.getInstance();
    this.cameraSubsystem = CameraSubsystem.getInstance();
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("Subsystem Intake", IntakeSubsystem.getInstance());
    SmartDashboard.putData("Subsystem Shooter", ShooterSubsystem.getInstance());
    this.intakeSubsystem.periodic();
    this.shooterSubsystem.periodic();
  }
}
