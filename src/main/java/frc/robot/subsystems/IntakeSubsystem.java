package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    /**
     * The function of this subsystem is to power 
     * VictorSPX controllers connected to a VEX 775pro,
     * to control the intake, feeder, and conveyor.
     */
    VictorSPX intakeMotor, feederMotor, conveyorMotor;

    public IntakeSubsystem() {
        /**
         * First, we must assign the motors to their
         * respective IDs on the CAN Bus (set by PhoenixTuner)
         * This is so we can power each of them individually via 
         * new methods.
         */
        intakeMotor = new VictorSPX(10);
        feederMotor = new VictorSPX(11);
        conveyorMotor = new VictorSPX(12);

        /**
         * These motors should NOT be able to spin
         * when teleop or autonomous is not active.
         */
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        feederMotor.setNeutralMode(NeutralMode.Brake);
        conveyorMotor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Indvidual functions were created in order to power 
     * each cargo-handling motor separately to prevent
     * and better cargo management.
     */

    public void intakeSystem(double input, double mod){
        intakeMotor.set(ControlMode.PercentOutput, input*mod);
    }

    public void feederSystem(double input, double mod){
        feederMotor.set(ControlMode.PercentOutput, input*mod);
    }

    public void conveyorSystem(double input, double mod){
        feederMotor.set(ControlMode.PercentOutput, input*mod);
    }
}
