package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RevUpShooter extends CommandBase {
    //PIDController pid;
    //double kP = 0.1, kI = 0.01, kD = 0.001;

    public RevUpShooter() {
        //pid = new PIDController(kP, kI, kD);
    }

    @Override
    public void initialize() {
        //pid.reset();
    }

    @Override
    public void execute() {


        /*
        double velocity = pid.calculate(
            RobotContainer.m_shooterSubsystem.getShooterSpeed(),
            RobotContainer.m_shooterSubsystem.getRangeOfTrajectory()
        );
        */
        
        //RobotContainer.m_shooterSubsystem.setShooterSpeed(velocity);
        RobotContainer.m_shooterSubsystem.setShooterSpeed(
            RobotContainer.m_shooterSubsystem.getRangeOfTrajectory()/6300
        );
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted)
            RobotContainer.m_shooterSubsystem.setShooterSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
        //return pid.atSetpoint();
    }
}