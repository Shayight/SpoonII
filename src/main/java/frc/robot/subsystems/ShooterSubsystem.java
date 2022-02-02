package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    
    WPI_TalonFX m_leftShooter, m_rightShooter;

    public ShooterSubsystem() {
        m_leftShooter = new WPI_TalonFX(5);
        m_rightShooter = new WPI_TalonFX(6);

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

    public double shooterEncoder(){
        // System.out.println("shooter encoder"+  m_shooterRight.getSelectedSensorVelocity());
        return m_rightShooter.getSelectedSensorVelocity();
      }
}
