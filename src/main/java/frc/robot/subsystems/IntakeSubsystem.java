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
        feederMotor = new VictorSPX(12);
        conveyorMotor = new VictorSPX(11);

        /**
         * These motors should NOT be able to spin
         * when teleop or autonomous is not active.
         */
        intakeMotor.setNeutralMode(NeutralMode.Coast);
        feederMotor.setNeutralMode(NeutralMode.Coast);
        conveyorMotor.setNeutralMode(NeutralMode.Coast);
        intakeMotor.setInverted(true);
        conveyorMotor.setInverted(true);
        // feederMotor.setNeutralMode(NeutralMode.Brake);
        // conveyorMotor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Indvidual functions were created in order to power 
     * each cargo-handling motor separately to prevent
     * and better cargo management.
     */

    public void setIntakeSystem(double input, double mod){
        intakeMotor.set(ControlMode.PercentOutput, input*mod);
    }

    public void setFeederSystem(double input, double mod){
        feederMotor.set(ControlMode.PercentOutput, input*mod);
    }

    public void setConveyorSpeed(double input, double mod){
        conveyorMotor.set(ControlMode.PercentOutput, input*mod);
    }

}
