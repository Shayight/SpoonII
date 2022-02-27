// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight;
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
  public static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  public static Limelight m_limelight = new Limelight("limelight");
 
  //Input from the PS4 Controllers. When calling these, we must define what port the controllers are plugged into (set by the DRIVER STATION).
  public static PS4Controller m_driver = new PS4Controller(0);
  public static PS4Controller m_operator = new PS4Controller(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //executes the SmartDashboard commands.
    SetupDashboard();
  }

  public void driveRobot(){
    double driveLS = m_driver.getLeftY(); //gets Y axis (the up and down) of the left stick for the driver's controller (port 0).
    double driveRS = m_driver.getRightY(); //gets Y axis of the right stick for the driver's controller (port 0).
    m_driveSubsystem.tankDrive(driveLS, driveRS, 1);  //The driver is able to control the robot using tank drive.
  }

  public void shooter() {
    boolean shooterButton = m_operator.getCrossButton(); //binds the shooter to the crosshair button.
    double modifier = (1 - m_operator.getL2Axis())/2; //gets a value between 0 and 1 via the trigger. 1 is released, 0 is pressed down. (converted from -1 and 1)
    SmartDashboard.putNumber("Turret Modifier", modifier);

    if(shooterButton)
      m_shooterSubsystem.setShooterSpeed(1, modifier); //Enables shooting the balls out, which the force can be adjusted using the modifier.
    else
      m_shooterSubsystem.setShooterSpeed(0, 0); //turns shooters off.
  }


  /**
   * The intention of this method is to setup the Smart Dashboard to display global output values, such as distance, rotation, etc.
   */
  public void SetupDashboard(){
    SmartDashboard.putNumber("Turret Encoder Rotation", m_shooterSubsystem.getTurretRotation());
    SmartDashboard.putNumber("Distance travelled (in inches)",m_driveSubsystem.getLinearDistanceEncoder());
    SmartDashboard.putNumber("Current Angle of Robot",m_driveSubsystem.getRotation());
    SmartDashboard.putNumber("Distance from Objective", m_limelight.getDistance());
  }
}
