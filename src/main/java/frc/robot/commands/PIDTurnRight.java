package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

public class PIDTurnRight extends CommandBase {

    PIDController pid;
    double mod = 0.5;
    double targetDegrees;
    //Almost perfect: 0.4, 0.09, 0.075
    double P=0.075, I=0.04, D=0.025;
    
    // private final DriveSubsystem drive_subsystem;
  
    public PIDTurnRight(double targetDegrees, double mod) {
      this.targetDegrees = targetDegrees;
      this.mod = mod;
      pid = new PIDController(P, I, D);
      pid.setTolerance(2);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pid.reset();
        RobotContainer.m_driveSubsystem.pigeonReset();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = pid.calculate(Math.abs(RobotContainer.m_driveSubsystem.getRotation()), targetDegrees);
        RobotContainer.m_driveSubsystem.tankDrive(speed, -speed);
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
