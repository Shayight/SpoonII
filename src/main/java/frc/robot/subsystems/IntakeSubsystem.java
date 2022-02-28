package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    /**
     * The function of this subsystem is to power 
     * VictorSPX controllers connected to a VEX 775pro,
     * to control the intake, feeder, and conveyor.
     */
    VictorSPX intakeMotor = new VictorSPX(10);
    VictorSPX feederMotor = new VictorSPX(11);
    VictorSPX conveyorMotor = new VictorSPX(12);

    public IntakeSubsystem() {
    
    }
}
