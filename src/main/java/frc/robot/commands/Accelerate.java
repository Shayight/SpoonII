package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Accelerate extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DrivetrainSubsystem m_drivetrain;

    public Accelerate(DrivetrainSubsystem drive){
        m_drivetrain = drive;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }

}
