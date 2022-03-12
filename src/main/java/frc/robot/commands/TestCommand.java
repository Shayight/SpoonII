package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestCommand {
    public Command Autonomous1() { //Left side autonomous
        return new SequentialCommandGroup(new DriveForward(0.3, 7));
    }

    public Command Autonomous2() { //Center position autonomous
        return new SequentialCommandGroup(new DriveForward(0.3, 7));
    }

    public Command Autonomous3() { //Right side autonomous
        return new SequentialCommandGroup(new DriveForward(0.7, 25),
        new ParallelCommandGroup(new DriveForward(0.7, 60), new IntakeCommand(1, 2)),
        new TurnRight(45, 2),
        new AutoshootCommand(0.6, 16700, 2)
        );
    }
}
