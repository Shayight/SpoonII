// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
    climberSolenoid = ph.makeDoubleSolenoid(3,4);

    leftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    rightMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    rightMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);




    leftMotor.setSoftLimit(SoftLimitDirection.kReverse, -175.0f);
    rightMotor.setSoftLimit(SoftLimitDirection.kReverse, -165.0f);
  }

  public void setClimberSpeed(double speed, double mod) {
    SmartDashboard.putNumber("Climber position - RIGHT", rightEncoder.getPosition());
    SmartDashboard.putNumber("Climber position - LEFT", leftEncoder.getPosition());
    leftMotor.setInverted(true);
    rightMotor.setInverted(false);
    leftMotor.set(speed*mod);
    rightMotor.set(speed*mod*.95);
  }

  public void resetClimber(){
    leftMotor.setSoftLimit(SoftLimitDirection.kForward, 0.0f);
    rightMotor.setSoftLimit(SoftLimitDirection.kForward, 0.0f);
  }

  public void setClimberSolenoidForward(){
    climberSolenoid.set(Value.kForward);
  }

  public void setClimberSolenoidReverse(){
    climberSolenoid.set(Value.kReverse);
  }

  public void setLeftClimber(double speed, double mod){
    leftMotor.setSoftLimit(SoftLimitDirection.kReverse, -250);
    leftMotor.set(speed*mod);
  }

  public void setRightClimber(double speed, double mod){
    leftMotor.setSoftLimit(SoftLimitDirection.kReverse, -250);
    rightMotor.set(speed*mod);
  }
}
