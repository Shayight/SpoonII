package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousCommand {

    NewAutonomous m_tc;
    
    /**
     * 
     * @param mode 0 = Autonomous will activate on left side, 
     * 1 = Autonomous will activate on middle, 
     * 2 = Autonomous will activate on right side.
     */
    public AutonomousCommand(){
        m_tc = new NewAutonomous();
    }
    
}
