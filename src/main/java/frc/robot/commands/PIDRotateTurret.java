package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class PIDRotateTurret extends CommandBase{

    double startingAngle;
    double targetAngle;
    double currentAngle;
    double P = 0.6, I = 0.4, D = 0.5;
    PIDController pid;

    public PIDRotateTurret(double targetAngle) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.targetAngle = targetAngle;
        pid = new PIDController(P, I, D);
        pid.setTolerance(2.5);
      }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {
          startingAngle = RobotContainer.m_shooterSubsystem.getTurretRotation();
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
          currentAngle = RobotContainer.m_shooterSubsystem.getTurretRotation() - startingAngle;
          double speed = pid.calculate(currentAngle, targetAngle);
          RobotContainer.m_shooterSubsystem.setTurretSpeed(speed, 0.6);
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        RobotContainer.m_shooterSubsystem.setTurretSpeed(0, 0.6);
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return pid.atSetpoint();
      }
    
}
