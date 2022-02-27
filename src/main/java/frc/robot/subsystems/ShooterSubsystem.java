package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    
    WPI_TalonFX m_leftShooter, m_rightShooter;
    CANSparkMax m_turret;
    RelativeEncoder m_turretEncoder;
    
  //The intention of the Shooter Subsystem class is to initialize the shooter components, including the turret, shooter, and other devices.
    public ShooterSubsystem() {
        //This initializes the TalonFX on the CAN ID device 5, which is the left shooter.
        m_leftShooter = new WPI_TalonFX(5);
        //This initializes the TalonFX on the CAN ID device 6, which is the right shooter.
        m_rightShooter = new WPI_TalonFX(6);
        //This initializes the motor that's on the turret, which in this case is a NEO 550. You MUST designate this as a brushless motor,
        //as it could potentially damage the motor or motor controller if handled incorrectly.
        m_turret = new CANSparkMax(8, MotorType.kBrushless);
        //This gets the SparkMAX's (Turret Motor Controller) built-in encoder, which records data from the turret's motor, such as speed, rotation, etc.
        m_turretEncoder = m_turret.getEncoder();


        //The shooter should NOT be able to spin when the motor is not active in code, so we set them to brake, locking them in place.
        m_leftShooter.setNeutralMode(NeutralMode.Brake);
        m_rightShooter.setNeutralMode(NeutralMode.Brake); 
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }

    public void setShooterSpeed(double speed,double modifier) {
        if (speed == 0.0){
          //If there is no speed/direction input, then there is no reason to power the motor. PercentOutput refers to a value between -1 and 1,
          //where -1 represents -100% throttle, or reverse, 0 for 0% throttle, or not touched, or 1 for 100%, which represents full speed.
            m_leftShooter.set(TalonFXControlMode.PercentOutput,0.0); 
            m_rightShooter.set(TalonFXControlMode.PercentOutput,0.0); 
          }
          else{
            //We flip the motors because on the physical shooter, we have the motors flipped, and in order to actually drive them in the same direction, we must flip it.
            //The modifier refers to how much the speed value should be scaled by. This can be a value between 0 and 1, any higher and the motor won't recognize it.
            m_leftShooter.set(TalonFXControlMode.PercentOutput, -speed*modifier);
            m_rightShooter.set(TalonFXControlMode.PercentOutput, speed*modifier); //sets shooter speed 
          }
    }

    public void setTurretSpeed(double speed, double modifier) {
      //rotates turret based on speed/direction, a value between -1 and 1, and the modifier, which can be a value between 0 and 1.
      m_turret.set(speed*modifier);
    }

    public double shooterEncoder(){
      // System.out.println("shooter encoder"+  m_shooterRight.getSelectedSensorVelocity());
    
      //This gets the speed of the shooter, recorded in raw sensor units for every 100ms.
      return m_rightShooter.getSelectedSensorVelocity();
    }

    
    public double getTurretRotation(){
      /** 
       * So we need to convert the turretEncoder's positions into degrees. To do this, we first need to know how many units are in a full rotation.
       * According to the NEO 550's spec sheets, this is 42 units per revolution.
       * If 1 full rotation is 360 degrees, we can assume that 360/42 should result in a proper conversion (THIS NEEDS TO BE TESTED)
       */
      double convFactor = 360/42;
      //This line applies the conversion factor to the encoder to convert it into degrees.
      m_turretEncoder.setPositionConversionFactor(convFactor);
      //This returns the value of the Encoder position after applying the conversion factor.
      double x = m_turretEncoder.getPosition();
      //This returns the Encoder value to the function where it is called.
      return x; 
    }
}
