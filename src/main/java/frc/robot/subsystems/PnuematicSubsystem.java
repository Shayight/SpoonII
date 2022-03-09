package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PnuematicSubsystem extends SubsystemBase {
    /**
     * This class controls the pnuematic hub and
     * all of the solenoids/pistons connected to it.
     * This also controls the pressure switch and the
     * compressor.
     */
    PneumaticHub ph;
    Compressor compressor;
    DoubleSolenoid intakeSolenoid;

    public boolean enabled;
    public boolean pressureSwitch;
    public double current;


    public PnuematicSubsystem() {

        ph = new PneumaticHub(20);
        compressor = ph.makeCompressor();
        intakeSolenoid = ph.makeDoubleSolenoid(0, 1);

        enabled = ph.getCompressor();
        pressureSwitch = compressor.getPressureSwitchValue();
        current = compressor.getCurrent();

        compressor.enableDigital();
    }

    public void setIntakeForward() {
        intakeSolenoid.set(Value.kForward);
    }
    public void setIntakeReverse(){
        intakeSolenoid.set(Value.kReverse);
    }

}
