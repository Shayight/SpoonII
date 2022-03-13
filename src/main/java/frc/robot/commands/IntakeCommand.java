package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeCommand extends CommandBase {
    boolean isForward;
    double endTime;
    double currentTime;
    double mod;
    Timer timer = new Timer();

    public IntakeCommand(double speed, double time){
        endTime = time;
        mod = speed;
    }

   // Called when the command is initially scheduled.
   @Override
   public void initialize() {
    RobotContainer.m_pnuematicSubsystem.setIntakeReverse();
    timer.reset();
    timer.start();
    RobotContainer.m_intakeSystem.setIntakeSystem(-1, mod);
    RobotContainer.m_intakeSystem.setFeederSystem(1, mod);
   }
 
   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() {
    RobotContainer.m_intakeSystem.setIntakeSystem(-1, mod);
    RobotContainer.m_intakeSystem.setFeederSystem(1, mod);
    currentTime = timer.get();
   }
 
   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
    RobotContainer.m_intakeSystem.setIntakeSystem(1, 0);
    RobotContainer.m_intakeSystem.setFeederSystem(0.6, 0);
    timer.stop();
   }
 
   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     return currentTime >= endTime;
   }  
    
}
