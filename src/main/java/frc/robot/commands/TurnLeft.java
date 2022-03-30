/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class TurnLeft extends CommandBase {
  double pigeonVal;
  double pigeonValnit;
  double mod = 0.5;
  double startingAngle;
  double targetDegrees;
  double P=0.3, I=0, D =0;
  int integral, previous_error;
  double error,derivative,rcw,time, currTime;
  double currentAngle;
  Timer timer;

  
  // private final DriveSubsystem drive_subsystem;

  public TurnLeft(double targetDegrees, double mod) {
    this.targetDegrees = targetDegrees;
    this.mod = mod;
    timer = new Timer();
    RobotContainer.m_driveSubsystem.pigeonReset();
    // drive_subsystem = subsystem;
    // addRequirements(drive_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void setTest(double P, double I, double D){
    
    this.P = P;
    this.I = I;
    this.D = D;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //get value from pigeon
    // pigeonValnit = RobotContainer.m_drive_subsystem.getYaw();
    // RobotContainer.m_drive_subsystem.tankDrive(1.0,-1.0,0.5);
    // reset angle
    startingAngle = RobotContainer.m_driveSubsystem.getRotation();
    //RobotContainer.m_driveSubsystem.tankDrive(1, -1, mod);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = RobotContainer.m_driveSubsystem.getRotation();
    currTime = timer.get();
    // pigeonVal= RobotContainer.m_drive_subsystem.getYaw();
    // RobotContainer.m_drive_subsystem.tankDrive(1.0,-1.0,0.5);
    // pid
  
    error = targetDegrees + RobotContainer.m_driveSubsystem.getRotation(); // Error = Target - Actual
    double finalMotorSpeed = P * error;
    double clampedSpeed = Math.max(Math.min(finalMotorSpeed,-1),1);
    RobotContainer.m_driveSubsystem.arcadeDrive(0,finalMotorSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_driveSubsystem.arcadeDrive(0.0, 0);
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (currentAngle <= -targetDegrees);
    // return (pigeonVal > (pigeonValnit + (targetDegrees/1.2)));
  }
}