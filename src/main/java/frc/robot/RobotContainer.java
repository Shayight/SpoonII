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

  public static AutonomousCommand m_autoCommand = new AutonomousCommand();
  private TestCommand m_tc = new TestCommand();

  //Buttons for commands from the controllers.
  private JoystickButton operator_circleButton;
  private JoystickButton operator_crossButton;
  private JoystickButton operator_shareButton;
  private JoystickButton operator_optionButton;
  private boolean operator_L1;
  private boolean operator_R1;
  private boolean operator_L2;
  private boolean operator_R2;

  private JoystickButton test_circleButton;
  private JoystickButton test_crossButton;
  private JoystickButton test_squareButton;
  private JoystickButton test_triangleButton;
  
  private double modifier = 0.6;

  //Input from the PS4 Controllers. When calling these, we must define what port the controllers are plugged into (set by the DRIVER STATION).
  public static PS4Controller m_driver = new PS4Controller(0);
  public static PS4Controller m_operator = new PS4Controller(1);
  public static PS4Controller m_testing = new PS4Controller(2);

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
    double driveLS = m_driver.getLeftY(); //gets Y axis (the up and down) of the left stick for the driver's controller (port 0).
    double driveRS = m_driver.getRightY(); //gets Y axis of the right stick for the driver's controller (port 0).
    m_driveSubsystem.tankDrive(driveLS, driveRS, 0.9);  //The driver is able to control the robot using tank drive.
  }

  public void watchIntakeControls() {
    double leftStickY = m_operator.getLeftY();
    double intakePower = leftStickY;
    m_intakeSystem.setIntakeSystem(intakePower, 1);
  }

  public void ShooterInit(){
    
    operator_circleButton = new JoystickButton(m_operator, PS4Controller.Button.kCircle.value);
    operator_crossButton = new JoystickButton(m_operator, PS4Controller.Button.kCross.value);
    operator_shareButton = new JoystickButton(m_operator, PS4Controller.Button.kShare.value);
    operator_optionButton = new JoystickButton(m_operator, PS4Controller.Button.kOptions.value);




    double maxDistance = 80;
    modifier = m_limelight.getDistance()/maxDistance;


    operator_circleButton.whenActive(new AutoAimCommand(0.6));
    operator_crossButton.whenActive(new ShootingCommand(0.8, 7000));
    //operator_shareButton.whenActive(new IntakeCommand(0.8, 1));
    operator_optionButton.whenHeld(new ConveyorCommand(0.8, 2), true);

  }


  public void shooter() {
    boolean reverseButton = m_operator.getSquareButton();
    double turretInput = m_operator.getRightX();
    boolean driver_L2 = m_driver.getL2Button();
    boolean driver_R2 = m_driver.getR2Button();

    
    operator_L1 = m_operator.getL1Button();
    operator_L2 = m_operator.getL2Button();
    operator_R1 = m_operator.getR1Button();
    operator_R2 = m_operator.getR2Button();

    if(operator_L1)
      m_climberSubsystem.setClimberSpeed(1, 0.6);
    else if(operator_R1)
      m_climberSubsystem.setClimberSpeed(-1, 0.6);
    else
      m_climberSubsystem.setClimberSpeed(0, 1);
    
    if(operator_L2)
      m_climberSubsystem.setClimberSolenoidForward();
    else if(operator_R2)
      m_climberSubsystem.setClimberSolenoidReverse();

    if(driver_L2){
      m_intakeSystem.setIntakeSystem(1, 0.7);
    }else if(driver_R2){
      m_intakeSystem.setIntakeSystem(1, -0.7);
    }else{
      m_intakeSystem.setIntakeSystem(0, 1);
    }



    boolean feederButton = m_operator.getShareButton();
    m_shooterSubsystem.setTurretSpeed(-turretInput, 1);


    SmartDashboard.putNumber("Turret Modifier", modifier);

    if(reverseButton){
      m_shooterSubsystem.setShooterSpeed(-1, 0.7); //Enables shooting the balls out, which the force can be adjusted using the modifier.
      m_intakeSystem.setFeederSystem(-1, 0.7);  
      m_intakeSystem.setConveyorSpeed(-1, 0.5);
    }
    else{
      m_shooterSubsystem.setShooterSpeed(0, 0); //turns shooters off.
      m_intakeSystem.setFeederSystem(0, 0); 
      m_intakeSystem.setConveyorSpeed(0, 0); 
    }

    if(feederButton)  {
      m_intakeSystem.setFeederSystem(1, 0.6);
      m_intakeSystem.setConveyorSpeed(1, 1);
    }


  }

  public void PnuematicControl(){
    boolean setIntakeFwd = m_driver.getL1Button();
    boolean setIntakeRev = m_driver.getR1Button();
    if(setIntakeFwd)
      m_pnuematicSubsystem.setIntakeForward();
    if(setIntakeRev)
      m_pnuematicSubsystem.setIntakeReverse();
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public void testing(){
    test_circleButton = new JoystickButton(m_testing, PS4Controller.Button.kCircle.value);
    test_crossButton = new JoystickButton(m_testing, PS4Controller.Button.kCross.value);
    test_squareButton = new JoystickButton(m_testing, PS4Controller.Button.kSquare.value);
    test_triangleButton = new JoystickButton(m_testing, PS4Controller.Button.kTriangle.value);

<<<<<<< HEAD
    test_circleButton.whenActive(new TurnLeft(-90, 0.));
=======
    test_circleButton.whenActive(new TurnLeft(90, 0.5));
>>>>>>> 98ff91f0ebf0be30c922a4941068f1f0bdcdbf44
  }

  /**
   * The intention of this method is to setup the Smart Dashboard to display global output values, such as distance, rotation, etc.
   */
  public void SetupDashboard(){
    SmartDashboard.putNumber("Turret Encoder Rotation", m_shooterSubsystem.getTurretRotation());
    SmartDashboard.putNumber("Distance travelled (in inches)",m_d5riveSubsystem.getLinearDistanceEncoder());
    SmartDashboard.putNumber("Distance from Objective", m_limelight.getDistance());
    SmartDashboard.putNumber("Motor Speed", m_driveSubsystem.getCurrentMotorSpeed());
    SmartDashboard.putBoolean("Target Locked", m_limelight.canSeeTarget());

    SmartDashboard.putData(m_chooser);
  }
}
