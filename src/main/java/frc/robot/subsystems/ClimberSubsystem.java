// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  public CANSparkMax leftMotor,rightMotor;
  public DoubleSolenoid climberSolenoid;
  public RelativeEncoder leftEncoder,rightEncoder;

  PneumaticHub ph;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    ph = new PneumaticHub(20);

    leftMotor = new CANSparkMax(16, MotorType.kBrushless);
    rightMotor = new CANSparkMax(17, MotorType.kBrushless);
    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();
    climberSolenoid = ph.makeDoubleSolenoid(2,4);
  }

  public void setClimberSpeed(double speed, double mod) {
    leftMotor.setInverted(true);
    leftMotor.set(speed*mod);
    rightMotor.set(speed*mod);
  }

  public void setClimberSolenoidForward(){
    climberSolenoid.set(Value.kForward);
  }

  public void setClimberSolenoidReverse(){
    climberSolenoid.set(Value.kReverse);
  }
}
