package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ConveyorCommand extends CommandBase {
    boolean isForward;
    double endTime;
    double currentTime;
    double mod;
    Timer timer = new Timer();

    public ConveyorCommand(double speed, double time){
        endTime = time;
        mod = speed;
    }

   // Called when the command is initially scheduled.
   @Override
   public void initialize() {
    RobotContainer.m_pnuematicSubsystem.setIntakeReverse();
    timer.reset();
    timer.start();
    RobotContainer.m_intakeSystem.setConveyorSpeed(.5, mod);
   }
 
   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() {
    RobotContainer.m_intakeSystem.setConveyorSpeed(.5, mod);
    currentTime = timer.get();
   }
 
   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
    RobotContainer.m_intakeSystem.setConveyorSpeed(1, 0);

    timer.stop();
   }
 
   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     return currentTime >= endTime;
   }  
    
}
