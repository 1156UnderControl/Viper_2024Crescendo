package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Limelight.LimelightHelpers;
import frc.robot.Joysticks.ControlBoard;
import frc.robot.commands.Autonomous.AutonomousAim;
import frc.robot.commands.Autonomous.AutonomousAimSteal;
import frc.robot.commands.Autonomous.AutonomousAimSteal1;
import frc.robot.commands.Autonomous.AutonomousAimSteal2;
import frc.robot.commands.Autonomous.AutonomousAimStealAuto2;
import frc.robot.commands.Autonomous.AutonomousAimSubwoofer;
import frc.robot.commands.Autonomous.AutonomousCollect;
import frc.robot.commands.Autonomous.AutonomousCollectAligning;
import frc.robot.commands.Autonomous.AutonomousDeployOnlyIntake;
import frc.robot.commands.Autonomous.AutonomousPassThrough;
import frc.robot.commands.Autonomous.AutonomousSecured;
import frc.robot.commands.Autonomous.AutonomousShoot;
import frc.robot.commands.States.ClimbPosition;
import frc.robot.commands.States.CollectHPposition;
import frc.robot.commands.States.CollectPosition;
import frc.robot.commands.States.ExpellPosition;
import frc.robot.commands.States.SecuredPosition;
import frc.robot.commands.States.ShootAmpPosition;
import frc.robot.commands.States.ShootOverStagePosition;
import frc.robot.commands.States.ShootSpeakerPosition;
import frc.robot.commands.Swerve.SwerveAlignShootOverStage;
import frc.robot.commands.Swerve.SwerveAlignSpeaker;
import frc.robot.commands.Swerve.SwerveTeleopControl;
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
  private SwerveSubsystem swerveSubsystem;

  private ControlBoard controlBoard = ControlBoard.getInstance();

  public static boolean shootFinished = true;
  public boolean preparingShoot = false;
  public boolean climbing;

  private PowerDistribution powerDistributionHub;

  private DoubleLogEntry batteryVoltageLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "/Robot/BatteryVoltage");

  private DoubleLogEntry totalCurrentDrawLogEntry = new DoubleLogEntry(DataLogManager.getLog(),
      "/Robot/TotalCurrentDraw");

  private final SendableChooser<Command> autoChooser;

  public RobotSubsystems() {
    autoIntakeCommands();
    autoShootCommands();
    auto5NotesStealingCommands();
    auto2notesStealCommands();
    this.swerveSubsystem = SwerveSubsystem.getInstance(new File(Filesystem.getDeployDirectory(),
        "swerve/swerve"));
    this.intake = IntakeSubsystem.getInstance();
    this.shooter = ShooterSubsystem.getInstance();
    this.climber = ClimberSubsystem.getInstance();
    this.led = LedSubsystem.getInstance();
    this.swerve = SwerveSubsystem.getInstance();
    this.intakeSubsystem = IntakeSubsystem.getInstance();
    this.shooterSubsystem = ShooterSubsystem.getInstance();
    this.climberSubsystem = ClimberSubsystem.getInstance();
    this.ledSubsystem = LedSubsystem.getInstance();
    this.powerDistributionHub = new PowerDistribution();
    this.batteryVoltageLogEntry.append(this.powerDistributionHub.getVoltage());
    this.totalCurrentDrawLogEntry.append(this.powerDistributionHub.getTotalCurrent());
    autoChooser = AutoBuilder.buildAutoChooser("None");
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
    swerveSubsystem.setDefaultCommand(new SwerveTeleopControl(swerveSubsystem));
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

  public void setHeadingCorrection(boolean active) {
    swerveSubsystem.setHeadingCorrection(active);
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

  private void configureBindings() {
    this.configureSwerveBindings();
    this.configureRobotBindings();
    this.controlBoard.getIntake().and(() -> RobotSubsystems.shootFinished)
        .onTrue(new CollectPosition(this, swerveSubsystem));
  }

  private void configureSwerveBindings() {
    this.controlBoard.resetGyro().onTrue(new InstantCommand(swerveSubsystem::zeroGyro));
    this.controlBoard.setHeadingFront().onTrue(new InstantCommand(swerveSubsystem::setAngle0));
    this.controlBoard.setHeadingBack().onTrue(new InstantCommand(swerveSubsystem::setAngle180));
    this.controlBoard.setHeadingLeft().onTrue(new InstantCommand(swerveSubsystem::setAngle90));
    this.controlBoard.setHeadingRight().onTrue(new InstantCommand(swerveSubsystem::setAngleNeg90));
    this.controlBoard.getSecured().onTrue(new SwerveTeleopControl(swerveSubsystem));
    this.controlBoard.getPrepareSpeaker().and(() -> this.shooter.hasNote())
        .onTrue(new SwerveAlignSpeaker(this.swerveSubsystem));
  }

  private void configureRobotBindings() {
    this.controlBoard.getIntakeFromHP().onTrue(new CollectHPposition(this));
    this.controlBoard.getSecured().onTrue(new SecuredPosition(this));
    this.controlBoard.getPrepareAmp().and(() -> this.shooter.hasNote())
        .onTrue(new ShootAmpPosition(this, swerveSubsystem));
    this.controlBoard.getPrepareSpeaker().and(() -> this.shooter.hasNote())
        .onTrue(new ShootSpeakerPosition(this));
    this.controlBoard.getDoNotShoot().and(() -> !this.preparingShoot).and(() -> this.shooter.hasNote())
        .onTrue(parallel(
            new ShootOverStagePosition(this), new SwerveAlignShootOverStage(swerveSubsystem)));
    this.controlBoard.release().onTrue(new ClimbPosition(this));
    this.controlBoard.expell().onTrue(new ExpellPosition(this));
    this.controlBoard.falseNegative().onTrue(new InstantCommand(() -> this.shooter.setHasNoteTrue()));
    this.controlBoard.falsePositive().onTrue(new InstantCommand(() -> this.shooter.setHasNoteFalse()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void autoShootCommands() {
    NamedCommands.registerCommand("Pass Through", new AutonomousPassThrough(this));
    NamedCommands.registerCommand("Shoot", new AutonomousShoot(this).andThen(Commands.waitSeconds(0.1)));
  }

  private void auto2notesStealCommands() {
    NamedCommands.registerCommand("Aim Shoot Subwoofer Side", new AutonomousAimSubwoofer(this));
    NamedCommands.registerCommand("Aim Shoot Steal auto 2 notes", new AutonomousAimStealAuto2(this));
    NamedCommands.registerCommand("Aim Shoot Steal", new AutonomousAimSteal(this));
    NamedCommands.registerCommand("Aim Shoot Steal 1", new AutonomousAimSteal1(this));
    NamedCommands.registerCommand("Aim Shoot Steal 2", new AutonomousAimSteal2(this));
  }

  private void autoIntakeCommands() {
    NamedCommands.registerCommand("Intake", new AutonomousCollect(this));
    NamedCommands.registerCommand("Intake Aligning", new AutonomousCollectAligning(this));
    NamedCommands.registerCommand("Deploy Intake", new AutonomousDeployOnlyIntake(this));
    NamedCommands.registerCommand("Secured", new AutonomousSecured(this));
  }

  private void auto5NotesStealingCommands() {
    NamedCommands.registerCommand("Aim Shoot 1", new AutonomousAim(this, 0));
    NamedCommands.registerCommand("Aim Shoot 2", new AutonomousAim(this, 1));
    NamedCommands.registerCommand("Aim Shoot 3", new AutonomousAim(this, 2));
    NamedCommands.registerCommand("Aim Shoot 4", new AutonomousAim(this, 3));
  }
}
