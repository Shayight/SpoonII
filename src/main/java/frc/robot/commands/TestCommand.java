package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestCommand {
    public Command Autonomous1() { //Left side autonomous
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new IntakeCommand(1, 2),
                new ConveyorCommand(0.6, 1),
                new PIDDriveCommand(0.7, 35)
            ),
            new TurnRight(180, 0.7),
            new AutoshootCommand(0.63, 5)
        );
    }

    public Command Autonomous2() { //Center position autonomous
        return new SequentialCommandGroup(new PIDDriveCommand(0.8, 25)
            //new IntakeCommand(1, 2),
            //new AutoshootCommand(0.6, 16700, 2)
        );
            
    }

    public Command Autonomous3() { //Right side autonomous
        return new SequentialCommandGroup(
            new PIDDriveCommand(15, .5),
            new ParallelCommandGroup(
                new ConveyorCommand(0.5, 1),
                new PIDDriveCommand(20, 0.6),
                new IntakeCommand(1, 1.7)
            ),
            new ParallelCommandGroup(
                new ConveyorCommand(0.5, 0.5),
                new PIDTurn(150, 0.7)
            ),
            new AutoshootCommand(0.68, 5),
            new PIDTurn(65, 0.7)
        );
    }

    public Command Autonomous4() { //Right side autonomous
        return new SequentialCommandGroup(
            // pickup second ball
            new ParallelCommandGroup(
                new ConveyorCommand(0.5, 1),
                new PIDDriveCommand(35, 0),
                new IntakeCommand(1, 1.7)
            ),
            // point towards goal
            new ParallelCommandGroup(
                new ConveyorCommand(0.5, 0.5),
                new PIDTurn(150, 0.7)
            ),
            // shoot
            new AutoAimCommand(0.7),
            // new AutoshootCommand(0.68, 5),
            // travel to third ball
            new PIDTurnLeft(125, 0.7),
            new ParallelCommandGroup(
                new ConveyorCommand(0.5, 1),
                new PIDDriveCommand(65, 0),
                new IntakeCommand(1, 1.7)
            )
            // point towards goal
            // new ParallelCommandGroup(
            //     new ConveyorCommand(0.5, 0.5),
            //     new PIDTurnRight(150, 0.7)
            // ),
            // // shoot
            // new AutoshootCommand(0.68, 5),
        );
    }

    public Command Autonomous5() { //Center Autonomous
        return new SequentialCommandGroup(
            new PIDDriveCommand(15, 0.5),
            new PIDTurn(90, 0.5),
            new PIDRotateTurret(-45),
            new AutoAimCommand(0.6)
        );
    }
    public Command Autonomous6() { //Right autononmous, 4 ball
        return new SequentialCommandGroup(
            new ParallelCommandGroup( //Phase 1, pick up ball in front. (2 balls in feeder) (4 seconds)
                new PIDDriveCommand(65, 0.8), 
                new IntakeCommand(1, 3)
            ),
            new PIDTurn(180, 0.6), //Phase 2, shoot balls close to target. (2 seconds)
            new ParallelCommandGroup( 
                new AutoAimCommand(0.6),
                new ShootingCommand(3)    
            ),
            new PIDTurn(-158, 0.6), //Phase 3, turn and pick up the ball by the driverstation. Additionally, wait a few seconds until the player feeds the ball. (2 balls in feeder) (5 seconds)
            new PIDDriveCommand(160, 0.8),  
            new IntakeCommand(1, 3),

            new PIDDriveCommand(-30, 0.8), //Phase 4, drive backwards, turn around, and SEND IT. (4 seconds)
            new PIDTurn(180, 0.6),
            new ParallelCommandGroup(
                new AutoAimCommand(0.6),
                new ShootingCommand(3)
            ) //END OF AUTONOMOUS.
        );
    }

    }
