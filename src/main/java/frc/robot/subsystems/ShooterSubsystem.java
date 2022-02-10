package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    
    WPI_TalonFX m_leftShooter, m_rightShooter;
    CANSparkMax m_turret;
    RelativeEncoder m_turretEncoder;
    

    public ShooterSubsystem() {
        m_leftShooter = new WPI_TalonFX(5);
        m_rightShooter = new WPI_TalonFX(6);
        m_turret = new CANSparkMax(8, MotorType.kBrushless);
        m_turretEncoder = m_turret.getEncoder();

        m_leftShooter.setNeutralMode(NeutralMode.Brake);
        m_rightShooter.setNeutralMode(NeutralMode.Brake); //shooter should not be moving by default.
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
            m_leftShooter.set(TalonFXControlMode.PercentOutput,0.0*modifier); //no shooter speed, shooter disabled.
            m_rightShooter.set(TalonFXControlMode.PercentOutput,0.0*modifier); 
          }
          else{
            m_leftShooter.set(TalonFXControlMode.PercentOutput, -speed*modifier);
            m_rightShooter.set(TalonFXControlMode.PercentOutput, speed*modifier); //sets shooter speed 
          }
    }

    public void setTurretSpeed(double speed, double modifier) {
      //rotates turret
      m_turret.set(speed*modifier);
    }

    public double shooterEncoder(){
      // System.out.println("shooter encoder"+  m_shooterRight.getSelectedSensorVelocity());
      return m_rightShooter.getSelectedSensorVelocity();
    }
    
    public double turretEncoder(){
      return m_turretEncoder.getPosition(); 
      //TODO: need to test for how many steps are in one full revolution, and convert the steps to degrees.
    }
}
