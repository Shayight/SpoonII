package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class PIDDriveCommand extends CommandBase {
    /*  DON'T TOUCH THESE VALUES WITHOUT MAKING A COPY OF IT

        ON SCHOOL CARPET:
        double P = 0.25, I = 0.03, D = 0.03;
        double kP = 0.1, kI = 0.2, kD = 0.005;

        ON COMPETITION FIELD:

        
    */
    
    double P = 0.25, I = 0.03, D = 0.03;
    double kP = 0.1, kI = 0.2, kD = 0.005;

    PIDController pid;
    PIDController centerPID;
    double distance;
    double modifier;

    public PIDDriveCommand(double distance, double modifier) {

        this.distance = distance;
        this.modifier = modifier;
        pid = new PIDController(P, I, D);
        pid.setTolerance(2.5);
        centerPID = new PIDController(kP, kI, kD);
        centerPID.setTolerance(6.0);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.m_driveSubsystem.pigeonReset();
        pid.reset();
        centerPID.reset();
        RobotContainer.m_driveSubsystem.encoderReset();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = pid.calculate(-RobotContainer.m_driveSubsystem.getLinearDistanceEncoder(), distance);
        double steering = centerPID.calculate(RobotContainer.m_driveSubsystem.getRotation(), 0);

        RobotContainer.m_driveSubsystem.arcadeDrive(
            // 0.8 on School Carpet
            MathUtil.clamp(speed, -0.8, 0.8),
            MathUtil.clamp(steering, -0.2, 0.2)            
        );
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_driveSubsystem.arcadeDrive(0, 0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        /**
        System.out.println(pid.atSetpoint());
        System.out.println(centerPID.atSetpoint());
         */
        return pid.atSetpoint() && centerPID.atSetpoint();
        //return pid.atSetpoint() && Math.abs(centerPID.getPositionError()) < 5;

    }
    
}
