package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Limelight.LimelightHelpers;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.robot.Joysticks.ControlBoard;
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

  private ControlBoard controlBoard = ControlBoard.getInstance();

  public static boolean shootFinished = true;
  public boolean preparingShoot = false;
  public boolean climbing;

  private PowerDistribution powerDistributionHub;

  private CustomDoubleLogger batteryVoltageLogEntry = new CustomDoubleLogger("/Robot/BatteryVoltage");

  private CustomDoubleLogger totalCurrentDrawLogEntry = new CustomDoubleLogger("/Robot/TotalCurrentDraw");

  public RobotSubsystems() {
    this.swerve = SwerveSubsystem.getInstance();
    this.intake = IntakeSubsystem.getInstance();
    this.shooter = ShooterSubsystem.getInstance();
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

  public boolean robotHasNote() {
    return this.shooter.hasNote();
  }

  public void setCoastToRobot() {
    this.intake.setCoastPivot();
    this.shooter.setCoastPivot();
    this.swerve.setModulesToCoast();
  }

  public void setBrakeToRobot() {
    this.intake.setBrakePivot();
    this.shooter.setBrakePivot();
    this.swerve.setModulesToBrake();
  }

  public boolean isRobotAbleToShoot() {
    if (this.controlBoard.getDoNotShoot().getAsBoolean()) {
      return false;
    }
    return shooter.isPivotAtSetpointForAutoShoot()
        && shooter.isFlywheelAtSetpoint()
        && swerve.isSwerveAlignedWithSpeaker()
        && swerve.isSwerveInAutoShootZone()
        && swerve.isSwerveInVelocityToAutoShoot()
        && LimelightHelpers.getTV("limelight-back");

  }
}
