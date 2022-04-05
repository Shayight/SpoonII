/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnRight extends CommandBase {
  double pigeonVal;
  double pigeonValnit;
  double mod;
  double targetDegrees, startingAngle;
  double P=0.1, I=0, D =0;
  int integral, previous_error;
  double error, derivative, rcw, time, currTime;
  double currentAngle;
  Timer timer;

  
  // private final DriveSubsystem drive_subsystem;

  public TurnRight(double targetDegrees, double mod) {
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
    startingAngle = RobotContainer.m_driveSubsystem.getRotation();
    RobotContainer.m_driveSubsystem.tankDrive(-1, 1);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = RobotContainer.m_driveSubsystem.getRotation();
    currTime = timer.get();
  
    error = targetDegrees - RobotContainer.m_driveSubsystem.getRotation(); // Error = Target - Actual
    integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
    derivative = (error - previous_error) / .02;
    rcw = P*error + I*integral + D*derivative;
    System.out.println(rcw);

    double clamped = Math.max(Math.min(0.5, rcw), -0.5);
    RobotContainer.m_driveSubsystem.arcadeDrive(0, clamped);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_driveSubsystem.tankDrive(0.0, 0.0);
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (currentAngle >= targetDegrees);
  }
}
