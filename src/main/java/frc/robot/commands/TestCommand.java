package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestCommand {
    public Command Autonomous1() { //Left side autonomous
        return new SequentialCommandGroup(
            new ParallelCommandGroup(new IntakeCommand(1, 2), new ConveyorCommand(0.6, 1), new DriveForward(0.7, 35)),
            new TurnRight(180, 0.7),
            new AutoshootCommand(0.63, 5)
        );

    }

    public Command Autonomous2() { //Center position autonomous
        return new SequentialCommandGroup(new DriveForward(0.8, 25)
        //new IntakeCommand(1, 2),
        //new AutoshootCommand(0.6, 16700, 2)
        );
            
    }

    public Command Autonomous3() { //Right side autonomous
        return new SequentialCommandGroup(
        new ParallelCommandGroup(new DriveForward(0.7, 15)),
        new ParallelCommandGroup(new ConveyorCommand(0.5, 1), new DriveForward(0.7, 60), new IntakeCommand(1, 1.7)),
        new ParallelCommandGroup(new ConveyorCommand(0.5, 0.5), new TurnRight(150, 0.7)),
        new AutoshootCommand(0.68, 5),
        new TurnRight(65, 0.7)
        );
    }
}
