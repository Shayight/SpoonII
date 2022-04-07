package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShooterSubsystem extends SubsystemBase {
    double rpmConversionFactor;

    WPI_TalonFX m_rightShooter;
    CANSparkMax m_turret;
    RelativeEncoder m_turretEncoder;
    
  //The intention of the Shooter Subsystem class is to initialize the shooter components, including the turret, shooter, and other devices.
    public ShooterSubsystem() {
        rpmConversionFactor = 600/2048;

        // This initializes the TalonFX on the CAN ID device 6, which is the right shooter.
        m_rightShooter = new WPI_TalonFX(6);
        m_rightShooter.setNeutralMode(NeutralMode.Coast);

        //The shooter should NOT be able to spin when the motor is not active in code, so we set them to brake, locking them in place.
        //m_rightShooter.setNeutralMode(NeutralMode.Brake); 

        // This initializes the motor that's on the turret, which in this case is a NEO 550.
        // You MUST designate this as a brushless motor,
        //   as it could potentially damage the motor or motor controller if handled incorrectly.
        m_turret = new CANSparkMax(25, MotorType.kBrushless);
        m_turret.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_turret.setSoftLimit(SoftLimitDirection.kForward, 1073);

        m_turret.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_turret.setSoftLimit(SoftLimitDirection.kReverse, -1073);

        // This gets the SparkMAX's (Turret Motor Controller) built-in encoder, which records data from the turret's motor, such as speed, rotation, etc.
        m_turretEncoder = m_turret.getEncoder();
        m_turretEncoder.setPosition(0);
    }

    public void setShooterSpeed(double speed) {
      // (speed*2048)/600
      m_rightShooter.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void setTurretSpeed(double speed, double modifier) {
      //rotates turret based on speed/direction, a value between -1 and 1, and the modifier, which can be a value between 0 and 1
      //if(getTurr  etRotation() >= -90 && getTurretRotation() <= 90)
      if(speed > -0.10 && speed < 0.10)
        m_turret.set(0);
      else
        m_turret.set(speed*modifier);
      /**else if(getTurretRotation() <= -90)
        m_turret.set(.25);
      else if(getTurretRotation() >= 90)
        m_turret.set(-.25);*/
    }

    public double getShooterSpeed(){
      // Speed of shooter, recorded in raw sensor units for every 100ms, converted to RPM
      double speed = m_rightShooter.getSelectedSensorVelocity();// * this.rpmConversionFactor;
      
      SmartDashboard.putNumber("Shooter Speed", speed);
      //return speed;
      return speed*600/2048;
    }

    public double getRangeOfTrajectory(){
      double g = 9.81; // Gravity is -9.81 m/s 
      double shootingAngle = 30; // Mounted angle of shooter
      double r = 1.5 * Constants.convertToMeters; // Radius of wheel

      double shootingRadians = Math.sin(Math.toRadians(2 * shootingAngle));
      //double c = Math.sin(shootingRadians)/g;
      double R = (RobotContainer.m_limelight.getDistance());
      double v = Math.sqrt((R*g)/shootingRadians);
      //double v = Math.sqrt(R/c);
      //double oRPM = ((60 * (v / r)) / (2 * Math.PI));
      double oRPM = ((60 * (v / r)) /Math.PI);
      // double oRPM = (60*(v/r))/(2*Math.PI);

      System.out.println(String.format("R: %f; v: %f; oRPM: %f", R, v, oRPM));
      SmartDashboard.putNumber("Distance From Objective", R);
      SmartDashboard.putNumber("Decimal RPM",(oRPM/6300));

      //SmartDashboard.putNumber("Target Speed", nativeRPM);

      return oRPM;
    }

    public double getTurretRotation(){
      /** 
       * So we need to convert the turretEncoder's positions into degrees. To do this, we first need to know how many units are in a full rotation.
       * According to the NEO 550's spec sheets, this is 42 units per revolution.
       * If 1 full rotation is 360 degrees, we can assume that 360/42 should result in a proper conversion (THIS NEEDS TO BE TESTED)
       */
      double convFactor = 360/42;
      //This line applies the conversion factor to the encoder to convert it into degrees.
      //m_turretEncoder.setPositionConversionFactor(convFactor);
      //This returns the value of the Encoder position after applying the conversion factor.
      double x = (m_turretEncoder.getPosition()*convFactor) / 7;
      //This returns the Encoder value to the function where it is called.
      return x; 
    }

    public double getRawTurretRotation(){
      return m_turretEncoder.getPosition();
    }
}
