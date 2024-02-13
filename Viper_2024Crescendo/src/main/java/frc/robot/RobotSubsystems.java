package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IIntake;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.IShooter;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class RobotSubsystems extends SubsystemBase {
  public IIntake intake;
  public IShooter shooter;

  private SubsystemBase intakeSubsystem;
  private SubsystemBase shooterSubsystem;

  public RobotSubsystems() {
    this.intake = IntakeSubsystem.getInstance();
    this.shooter = ShooterSubsystem.getInstance();
    this.intakeSubsystem = IntakeSubsystem.getInstance();
    this.shooterSubsystem = ShooterSubsystem.getInstance();
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("Subsystem Intake", intakeSubsystem);
    SmartDashboard.putData("Subsystem Shooter", shooterSubsystem);
    this.intakeSubsystem.periodic();
    this.shooterSubsystem.periodic();
  }
}
