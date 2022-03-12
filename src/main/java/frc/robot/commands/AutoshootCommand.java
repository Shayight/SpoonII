package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;

public class AutoshootCommand extends CommandBase{
    Timer timer;
    boolean buttonPressed;
    double mod;
    double maximum = 17300;
    double acc;
    double currentTime;
    double runTime;
    Limelight turret_Limelight;
  
    // TurretSubsystem turret_subsystem;
    // IntakeSubsystem intake_subsystem;
  
  
    
    public AutoshootCommand(double modifier, double accuracy, double time) {
      // System.out.println("constr ");
      // if the button is pressed the command runs, modifier is used to regulate the speed of the shooter for now
      // turret_subsystem = subsystem;
      // oi = subsystem2;
      // addRequirements(subsystem);
      // addRequirements(subsystem2);
      timer = new Timer();
      turret_Limelight = new Limelight("limelight");
      // turret_subsystem = new TurretSubsystem();
      // intake_subsystem = new IntakeSubsystem();
      // oi = new OI();
      mod = modifier;
      acc = accuracy;
      runTime = time;
    }
  
    // Called just before this Command runs the first time
    @Override
    public void initialize() {
      RobotContainer.m_shooterSubsystem.setShooterSpeed(1.0, mod);
      double adjust = turret_Limelight.steeringAdjust();//if there is a target, get the distance from it
      RobotContainer.m_shooterSubsystem.setTurretSpeed(-adjust, 0.25);
    }
  
    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
      double adjust = turret_Limelight.steeringAdjust();//if there is a target, get the distance from it
      RobotContainer.m_shooterSubsystem.setTurretSpeed(-adjust, 0.25);
          
      if (RobotContainer.m_shooterSubsystem.shooterEncoder() >= acc) {//Once at that speed, fire/load balls
          //17300 for
          //System.out.println("Execute shooter stuff");
          RobotContainer.m_shooterSubsystem.setShooterSpeed(1.0, mod);
          RobotContainer.m_intakeSystem.setFeederSystem(1, 1);
      }
        else{
          RobotContainer.m_shooterSubsystem.setShooterSpeed(1.0,mod);//Charges falcon motors until they reach certain speed
          timer.start();//Starts the timer
        }
  
        currentTime = timer.get();
        
    }
  
  
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      // System.out.println(" end");
      //Stops all motors and resets timer
      RobotContainer.m_shooterSubsystem.setShooterSpeed(0.0, mod);
      RobotContainer.m_intakeSystem.setFeederSystem(0.0,0.0);
      timer.reset();
    }
  
    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
      // System.out.println(" is finished");    
      return (currentTime >= runTime);
      }
    }