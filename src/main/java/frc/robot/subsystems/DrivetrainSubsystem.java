// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MotorEncoder;

public class DrivetrainSubsystem extends SubsystemBase {
    /**
     * Writing variables like the WPI_TalonFX below is entirely possible
     * because all of the variables are the same type, which is WPI_TalonFX.
     * The same applies for MotorControllerGroup, where both sides are the same type of variable.
    */
    public WPI_TalonFX m_FL,m_FR,m_RL,m_RR;
    public MotorControllerGroup m_leftSide, m_rightSide;
    public DifferentialDrive m_drive;
    public WPI_PigeonIMU pigeon;
    public MotorEncoder m_FLEncoder;
    
    //For the distance encoder
    Double distanceTotal = 0.0;
    Double speedLimit = 0.8;

  /** Creates a new Drivetrain subsystem and assigns sides. */
  public DrivetrainSubsystem() {
    /**
     * Firstly, we need to identify what CAN ID the Motor Controllers are set to via Phoenix Tuner.
     * Then, we need to assign the left two motors to FL and RL, which stand for Front Left and Rear Left.
     */
    m_FL = new WPI_TalonFX(1);
    m_FL.setNeutralMode(NeutralMode.Brake);
    m_RL = new WPI_TalonFX(2);
    m_RL.setNeutralMode(NeutralMode.Brake);
    /**
     * After doing so, we need to group these motor controllers 
     * together into one side for the drivetrain which is where MotorControllerGroup comes in handy.
     * This allows us to control both of these motors as one group.
     */
    m_leftSide = new MotorControllerGroup(m_FL, m_RL);

    /**
     * As of 2022, we MUST set one side to be inverted, 
     * otherwise the robot would permanently be in a spinning state under driver control.
     * If the driver were to push both sticks either up or down, the robot would spin since 
     * the RIO assumes that the motors are all facing the same direction.
     */

    //Repeat the same process from the left side to the right side. FR = Front Right, RR = Rear Right.
    m_FR = new WPI_TalonFX(3);
    m_FR.setNeutralMode(NeutralMode.Brake);
    m_RR = new WPI_TalonFX(4);
    m_RR.setNeutralMode(NeutralMode.Brake);

    m_rightSide = new MotorControllerGroup(m_FR, m_RR);
    m_rightSide.setInverted(true);
    

    //Finally, we need to combine the two sides into one drive that can be controlled autonomously or by a driver.
    m_drive = new DifferentialDrive(m_leftSide,m_rightSide);

    //Because the Pigeon IMU is connected over a CAN bus rather than to a TalonSRX, we can use a CAN ID for it.
    pigeon = new WPI_PigeonIMU(5);
    //We need to reset it so all the values are at 0,0,0 upon starting the robot, and then follow up with a calibration test.
    pigeon.reset();
    pigeon.calibrate();

    //Set the MotorEncoder value to the FL TalonFX.
    m_FLEncoder = new MotorEncoder(m_FL, Constants.MetersPerPulse, false);
    m_FLEncoder.resetEncoder();
  }
  
  /**
   * Now, we need to actually drive the robot.
   * @param speedLeft The speed/direction of the left side of the robot (-1 to 1).
   * @param speedRight The speed/direction of the right side of the robot (1 to 1).
   * @param mod The modifier value for the drivetrain that limits the speed of the robot (between 0 and 1, can be a decimal value).
   */
  public void tankDrive(double speedLeft, double speedRight, double mod){
      m_drive.tankDrive(speedLeft*mod*speedLimit, speedRight*mod*speedLimit);
      SmartDashboard.putNumber("Current Angle of Robot",getRotation());
  }

  public void arcadeDrive(double speed, double steering){
    m_drive.arcadeDrive(speed*speedLimit, steering);
  }

  public void pigeonReset(){
    pigeon.reset();
  }

  public void encoderReset(){
    m_FLEncoder.resetEncoder();
  }

  //This gets the current angle of the Pigeon IMU, or rather the direction of the robot.
  public double getRotation() {
    /**
     * Something to note about the Pigeon: The getAngle() function is continuous.
     * Meaning that after 360 degrees, the pigeon will not start over from 0, it will
     * continue adding up (361, 362, etc.)
     */
    return pigeon.getYaw();
  }

  public double getLinearDistanceEncoder() {

    distanceTotal = m_FLEncoder.getPositionMeters();

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