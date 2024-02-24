package frc.robot;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.IClimber;
import frc.robot.subsystems.Intake.IIntake;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.LEDs.ILed;
import frc.robot.subsystems.LEDs.LedSubsystem;
import frc.robot.subsystems.Shooter.IShooter;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.ISwerve;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class RobotSubsystems extends SubsystemBase {
  public IIntake intake;
  public IShooter shooter;
  public ISwerve swerve;
  public IClimber climber;
  public ILed led;

  private SubsystemBase intakeSubsystem;
  private SubsystemBase shooterSubsystem;
  private SubsystemBase climberSubsystem;
  private SubsystemBase ledSubsystem;

  public static boolean shootFinished;
  public boolean climbing;

  private PowerDistribution powerDistributionHub;

  private DoubleLogEntry batteryVoltageLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "/Robot/BatteryVoltage");

  private DoubleLogEntry totalCurrentDrawLogEntry = new DoubleLogEntry(DataLogManager.getLog(),
      "/Robot/TotalCurrentDraw");

  public RobotSubsystems() {
    this.intake = IntakeSubsystem.getInstance();
    this.shooter = ShooterSubsystem.getInstance();
    this.swerve = SwerveSubsystem.getInstance();
    this.climber = ClimberSubsystem.getInstance();
    this.led = LedSubsystem.getInstance();
    this.intakeSubsystem = IntakeSubsystem.getInstance();
    this.shooterSubsystem = ShooterSubsystem.getInstance();
    this.climberSubsystem = ClimberSubsystem.getInstance();
    this.ledSubsystem = LedSubsystem.getInstance();
    this.powerDistributionHub = new PowerDistribution();
    this.batteryVoltageLogEntry.append(this.powerDistributionHub.getVoltage());
    this.totalCurrentDrawLogEntry.append(this.powerDistributionHub.getTotalCurrent());
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("Subsystem Intake", IntakeSubsystem.getInstance());
    SmartDashboard.putData("Subsystem Shooter", ShooterSubsystem.getInstance());
    this.intakeSubsystem.periodic();
    this.shooterSubsystem.periodic();
    this.climberSubsystem.periodic();
    this.ledSubsystem.periodic();
    this.batteryVoltageLogEntry.append(this.powerDistributionHub.getVoltage());
    this.totalCurrentDrawLogEntry.append(this.powerDistributionHub.getTotalCurrent());
  }

  public boolean isRobotAbleToShoot() {
    return shooter.isPivotAtSetpoint() && shooter.isFlywheelAtSetpoint() && swerve.isSwerveAlignedWithSpeaker();
  }
}
