// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.Java_Is_UnderControl.Limelight.LimelightHelpers;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomStringLogger;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.States.SecuredPosition;
import frc.robot.subsystems.LEDs.LedColor;
import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this
 * class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

  private static Robot instance;
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  private Timer disabledTimer;

  private CustomStringLogger autonomousStateLogger;

  public static boolean isInAutonomous;

  public static boolean isDisabled = true;

  public static String autonomousState = "NONE";

  DigitalInput switchBrake = new DigitalInput(2);

  public Robot() {
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    DataLogManager.start("", "", 0.02);
    DriverStation.startDataLog(DataLogManager.getLog());
    autonomousStateLogger = new CustomStringLogger("Autonomous/State");
    this.robotContainer = new RobotContainer();
    addPeriodic(() -> {
      robotContainer.robot.shooter.runNoteDetection();
    }, 0.05, 0.005);
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    // Create a timer to disable motor brake a few seconds after disable. This will
    // let the robot stop
    // immediately when disabled, but then also let it be pushed more
    this.disabledTimer = new Timer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    SmartDashboard.putData("Robot", robotContainer.robot);
    isDisabled = isDisabled();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    Robot.isInAutonomous = false;
    this.disabledTimer.reset();
    this.disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;
    if (alliance == Alliance.Red) {
      LimelightHelpers.setPipelineIndex("limelight-back", 1);
    } else {
      LimelightHelpers.setPipelineIndex("limelight-back", 1);
    }
    if (switchBrake.get()) {
      this.robotContainer.setRobotToCoast();
      this.robotContainer.robot.led.setSolidColor(LedColor.RED);
    } else {
      this.robotContainer.setRobotToBrake();
      this.robotContainer.robot.led.setRainbow();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    SwerveConstants.ACELL_CORREC = false;
    // this.robotContainer.robot.setDefaultCommand(new
    // AutonomousCollect(robotContainer.robot));
    Robot.isInAutonomous = true;
    this.robotContainer.setHeadingCorrection(false);
    this.robotContainer.setRobotToBrake();
    this.autonomousCommand = robotContainer.getAutonomousCommand();
    this.robotContainer.robot.led.turnOff();

    // schedule the autonomous command (example)
    if (this.autonomousCommand != null) {
      this.autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    autonomousStateLogger.append(autonomousState);
  }

  @Override
  public void teleopInit() {
    this.robotContainer.setRobotToBrake();
    SwerveConstants.ACELL_CORREC = true;
    this.robotContainer.robot.setDefaultCommand(new SecuredPosition(robotContainer.robot));
    Robot.isInAutonomous = false;
    this.robotContainer.setHeadingCorrection(true);
    if (this.autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    ;
    this.robotContainer.robot.led.turnOff();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    try {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit() {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic() {
  }
}
