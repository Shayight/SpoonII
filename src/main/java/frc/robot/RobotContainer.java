// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootingCommand;
import frc.robot.commands.TestCommand;
import frc.robot.commands.TurnLeft;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PnuematicSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static DrivetrainSubsystem m_driveSubsystem = new DrivetrainSubsystem();
  public static IntakeSubsystem m_intakeSystem = new IntakeSubsystem();
  public static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  public static Limelight m_limelight = new Limelight("limelight");
  public static PnuematicSubsystem m_pnuematicSubsystem = new PnuematicSubsystem();
  public static ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  public static ControllerSubsystem m_controllerSubsystem = new ControllerSubsystem();

  public static AutonomousCommand m_autoCommand = new AutonomousCommand();
  private TestCommand m_tc = new TestCommand();


  //Input from the PS4 Controllers. When calling these, we must define what port the controllers are plugged into (set by the DRIVER STATION).
  public static PS4Controller m_driver = new PS4Controller(0);

  public static SendableChooser<Command> m_chooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_chooser.setDefaultOption("Right",m_tc.Autonomous3());
    m_chooser.addOption("Left",m_tc.Autonomous1());
    m_chooser.addOption("Middle",m_tc.Autonomous2());
    //executes the SmartDashboard commands.
    SetupDashboard();

  }

  public void driveRobot(){
    double driveLS = -m_driver.getLeftY(); //gets Y axis (the up and down) of the left stick for the driver's controller (port 0).
    double driveRS = -m_driver.getRightY(); //gets Y axis of the right stick for the driver's controller (port 0).
    m_driveSubsystem.tankDrive(driveLS*.9, driveRS*.9);  //The driver is able to control the robot using tank drive.
  }

  /**  public void watchIntakeControls() {
    double leftStickY = m_operator.getLeftY();
    double intakePower = leftStickY;
    m_intakeSystem.setIntakeSystem(intakePower, 1);
  } */


  public void ShooterInit(){

    //operator_shareButton.whenActive(new IntakeCommand(0.8, 1));
    m_controllerSubsystem.CommandController();
  }


  public void shooter() {
    m_controllerSubsystem.operatorPeriodic();
    m_controllerSubsystem.driverPeriodic();
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public void testing(){
    m_controllerSubsystem.testingInit();
  }

  /**
   * The intention of this method is to setup the Smart Dashboard to display global output values, such as distance, rotation, etc.
   */
  public void SetupDashboard(){
    SmartDashboard.putNumber("Turret Encoder Rotation", m_shooterSubsystem.getRawTurretRotation());
    SmartDashboard.putNumber("Distance travelled (in inches)",m_driveSubsystem.getLinearDistanceEncoder());
    SmartDashboard.putNumber("Distance from Objective", m_limelight.getDistance());
    SmartDashboard.putNumber("Motor Speed", m_driveSubsystem.getCurrentMotorSpeed());
    SmartDashboard.putBoolean("Target Locked", m_limelight.canSeeTarget());
    SmartDashboard.putNumber("Rotation Rate", m_driveSubsystem.getAngularAcceleration());


    SmartDashboard.putData(m_chooser);
  }
}
