package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class NewAutonomous {
    //LOW GOAL
    public Command autoLeftL(){
        return new SequentialCommandGroup(
            //Drive 50 inches with intake down
            //Pickup cargo
            new ParallelCommandGroup(
                new PIDDriveCommand(50, 0.6),
                new IntakeCommand(1, 5)
            ),
            //Go back to hub and raise intake (45 inches forward, room for error just in case it overshoots)
            new ParallelCommandGroup(
                new InstantCommand(() -> RobotContainer.m_pnuematicSubsystem.setIntakeReverse()),
                new PIDDriveCommand(-45, 0.6)
            ),
            //Shoot cargo into LOW goal (1500 RPM)
            new ShootingCommand(5, 1500) //CHANGE THIS IF IT ISN'T RIGHT
        );
    }

    public Command autoCenterL(){
        return new SequentialCommandGroup(
            //Shoot cargo into LOW goal (1500 RPM)
            new ShootingCommand(3, 1500), //CHANGE THIS IF IT ISN'T RIGHT
            //Drive out of the auto zone
            new PIDDriveCommand(35, 0.6)
        );
    }

    public Command autoRightL(){
        return new SequentialCommandGroup(
            //Pick up the ball in front of us
            new ParallelCommandGroup(
                new PIDDriveCommand(30, 0.6),
                new IntakeCommand(1, 1500)
            ),
            //Go to the hub and shoot cargo into LOW goal
            new ParallelCommandGroup(
                new PIDDriveCommand(-30, 0.8),
                new ShootingCommand(5, 1500)
            )
        );

        
    }

    //HIGH GOAL
    public Command autoLeftH(){
        return new SequentialCommandGroup(
            //Drive 50 inches with intake down
            //Pickup cargo
            new ParallelCommandGroup(
                new PIDDriveCommand(50, 0.6),
                new IntakeCommand(1, 5)
            ),
            //Go back to hub and raise intake (45 inches forward, room for error just in case it overshoots)
            new ParallelCommandGroup(
                new InstantCommand(() -> RobotContainer.m_pnuematicSubsystem.setIntakeReverse()),
                new PIDDriveCommand(-45, 0.6)
            ),
            //Shoot cargo into HIGH goal 
            new ShootingCommand(5, 3200) //CHANGE THIS IF IT ISN'T RIGHT
        );
    }

    public Command autoCenterH(){
        return new SequentialCommandGroup(
            //Shoot cargo into HIGH goal
            new PIDDriveCommand(10, 0.5),
            new ShootingCommand(3, 3300), //CHANGE THIS IF IT ISN'T RIGHT
            //Drive out of the auto zone
            new PIDDriveCommand(20, 0.6)
        );
    }

    public Command autoRightH(){
        return new SequentialCommandGroup(
            //Pick up the ball in front of us
            new ParallelCommandGroup(
                new PIDDriveCommand(30, 0.6),
                new IntakeCommand(1, 3200)
            ),
            //Go to the hub and shoot cargo into HIGH goal
            new ParallelCommandGroup(
                new PIDDriveCommand(-30, 0.8),
                new ShootingCommand(5, 3200)
            )
        );

        
    }
    
}
