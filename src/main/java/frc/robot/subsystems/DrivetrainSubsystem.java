// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    public WPI_TalonFX m_FL,m_FR,m_RL,m_RR;
    public MotorControllerGroup m_leftSide, m_rightSide;
    public DifferentialDrive m_drive;
  /** Creates a new Drivetrain subsystem and assigns sides. */
  public DrivetrainSubsystem() {
    m_FL = new WPI_TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_TALON);
    m_RL = new WPI_TalonFX(Constants.DRIVETRAIN_REAR_LEFT_TALON);

    m_leftSide = new MotorControllerGroup(m_FL, m_RL);

    m_FR = new WPI_TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_TALON);
    m_RR = new WPI_TalonFX(Constants.DRIVETRAIN_REAR_RIGHT_TALON);

    m_rightSide = new MotorControllerGroup(m_FR, m_RR);

    m_drive = new DifferentialDrive(m_leftSide,m_rightSide);
  }
  
  public void tankDrive(double speedLeft, double speedRight, double mod){
      m_drive.tankDrive(speedLeft*mod, speedRight*mod);
  }

  public double getEncoderDistance(int motor) {
    Double distanceTotal =0.0; 
    switch(motor)
    {
      case 1:
      //get FL motor encoder rotation
       Double m_flSensorPos =  m_FL.getSelectedSensorPosition(); //in theory this should work (per 4096 sensor units, do math to figure out the accurate distance in inches)
       Double distance_M_FL = m_flSensorPos * (Constants.DISTANCE_INCHES_ONE_UNIT); 
       distanceTotal += distance_M_FL;

       return distanceTotal; 
      
       case 2:
      //get RL motor encoder rotation
      Double m_rlSensorPos = m_RL.getSelectedSensorPosition();
      Double distance_M_RL = m_rlSensorPos * (Constants.DISTANCE_INCHES_ONE_UNIT); 
      distanceTotal += distance_M_RL; 

      return distanceTotal; 
      
      case 3:
      //get FR motor encoder rotation
      Double m_frSensorPos = m_FR.getSelectedSensorPosition();
      Double distance_M_FR = m_frSensorPos * (Constants.DISTANCE_INCHES_ONE_UNIT); 
      distanceTotal += distance_M_FR; 

      return distanceTotal; 

      case 4:
      //get RR motor encoder rotation
      Double m_rrSensorPos = m_RR.getSelectedSensorPosition();
      Double distance_M_RR = m_rrSensorPos * (Constants.DISTANCE_INCHES_ONE_UNIT); 
      distanceTotal += distance_M_RR; 

      return distanceTotal; 
    }
    return distanceTotal;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
