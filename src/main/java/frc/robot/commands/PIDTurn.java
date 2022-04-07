// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class PIDTurn extends CommandBase {
  PIDController pid;
    double mod = 0.5;
    double startingAngle;
    double targetAngle;
    
    /* DON'T TOUCH THESE VALUES WITHOUT MAKING A COPY OF IT

      ON SCHOOL CARPET:
      double P=0.3, I=0.04, D=0.0235;

      ON COMPETITION FLOOR:
    */
    double P=0.3, I=0.04, D=0.0235;

    // private final DriveSubsystem drive_subsystem;
  
    public PIDTurn(double targetDegrees, double mod) {
      this.targetAngle = targetDegrees;
      this.mod = mod;
      pid = new PIDController(P, I, D);
      pid.setTolerance(2.5);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      pid.reset();
      startingAngle = RobotContainer.m_driveSubsystem.getRotation();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = pid.calculate(RobotContainer.m_driveSubsystem.getRotation(), targetAngle);
        if(targetAngle < 0){
          RobotContainer.m_driveSubsystem.tankDrive(-speed, speed);
        }else if(targetAngle > 0){
          RobotContainer.m_driveSubsystem.tankDrive(speed, -speed);
        }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      RobotContainer.m_driveSubsystem.arcadeDrive(0.0, 0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
}
