// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.PIDDriveCommand;
import frc.robot.commands.PIDTurnLeft;
import frc.robot.commands.RevUpShooter;
import frc.robot.commands.ShootingCommand;
import frc.robot.utils.ButtonBoard;

public class ControllerSubsystem extends SubsystemBase {
  public PS4Controller m_driverController = new PS4Controller(0);
  public ButtonBoard m_operatorController = new ButtonBoard(1);

  public double modifier = 0;

  /** Creates a new ControllerSubsystem. */
  public ControllerSubsystem() {

  }

  /**
   * * * *Current Controls:* * * *
   * OPERATOR
   * A: Automatic Aim
   * B: Shoot Cargo
   * X: Higher RPM
   * Y: Lower RPM
   * L1: Conveyor Forward
   * L2: Eject Cargo
   * R1: Low Goal Shoot
   * R2: 
   * Joystick Vertical: Feeder Forward/Backward
   * Joystick Horizontal: Turret Left/Right
   * POV Vertical: Climber Up/Down
   * POV Horizontal: Solenoid Reverse/Forward
   */

  public void setButtonListeners() {
      m_operatorController.A.whenHeld(new AutoAimCommand(0.5),true);
      m_operatorController.B.whenHeld(new ShootingCommand(10,0), true);
      
      m_operatorController.LB.whenHeld(new ConveyorCommand(0.8, 2), true);

      //Adjustable Shooter Speeds
      m_operatorController.X.whenPressed(new InstantCommand(() -> RobotContainer.m_shooterSubsystem.modifyTurretSpeed(50)));
      m_operatorController.Y.whenPressed(new InstantCommand(() -> RobotContainer.m_shooterSubsystem.modifyTurretSpeed(-50)));

      //fixed shooting speeds (low, highz)
      m_operatorController.RT.whileActiveContinuous(new ShootingCommand(10, 1600), true);
      m_operatorController.RB.whenHeld(new ShootingCommand(10, 3500),true);
      
      //m_operatorController.options.whenPressed(new PIDTurnLeft(90, 0.8));


  }

  public void driverPeriodic(){
    // Intake control
    if(m_driverController.getL2Button()) { 
      RobotContainer.m_intakeSystem.setIntakeSystem(1, 0.7);
      RobotContainer.m_pnuematicSubsystem.setIntakeReverse();

    } else if(m_driverController.getR2Button()) {
      RobotContainer.m_intakeSystem.setIntakeSystem(1, -0.7);
    } else {
      RobotContainer.m_intakeSystem.setIntakeSystem(0, 1);
    }

    if(m_driverController.getL1Button())
      RobotContainer.m_pnuematicSubsystem.setIntakeForward();
    if(m_driverController.getR1Button())
      RobotContainer.m_pnuematicSubsystem.setIntakeReverse();
    /** 
    if(RobotContainer.m_limelight.getDistance() > 1 && RobotContainer.m_limelight.getDistance() < 1.5){
      m_operatorController.setRumble(1);
      m_operatorController.setRumble(1);
    }else{
      m_operatorController.setRumble(0);
      m_operatorController.setRumble(0);
    }*/
  }

  public void operatorPeriodic(){    
    if(m_operatorController.getPOVHorizontal() == 1)
      RobotContainer.m_climberSubsystem.setClimberSolenoidForward();
    else if(m_operatorController.getPOVHorizontal() == -1)
      RobotContainer.m_climberSubsystem.setClimberSolenoidReverse();
      

    if(m_operatorController.getPOVVertical() == 1)
      RobotContainer.m_climberSubsystem.setClimberSpeed(-1, 0.8);
    else if(m_operatorController.getPOVVertical() == -1)
      RobotContainer.m_climberSubsystem.setClimberSpeed(1, 0.8);
    else
      RobotContainer.m_climberSubsystem.setClimberSpeed(0, 1);

    if(m_operatorController.getSL()){
      RobotContainer.m_climberSubsystem.setLeftClimber(-1, 0.5);
    }

    if(m_operatorController.getSR()){
      RobotContainer.m_climberSubsystem.setRightClimber(-1, 0.5);
    }

    if(m_operatorController.getShare()){
      RobotContainer.m_shooterSubsystem.centerForClimb();
    }





    RobotContainer.m_intakeSystem.setConveyorSpeed(-m_operatorController.getYAxis(), .45);
    RobotContainer.m_intakeSystem.setFeederSystem(-m_operatorController.getYAxis(), .6);

    RobotContainer.m_shooterSubsystem.setTurretSpeed(-m_operatorController.getXAxis(), .4);

    double maxDistance = 170.59;
    modifier = RobotContainer.m_limelight.getDistance()/maxDistance;

    SmartDashboard.putNumber("Turret Modifier", modifier);

    if(m_operatorController.RB.getAsBoolean()){
      RobotContainer.m_driveSubsystem.pigeonReset();
    }

    if(m_operatorController.getLT()){
      // RobotContainer.m_shooterSubsystem.setShooterSpeed(-1, 0.7); //Enables shooting the balls out, which the force can be adjusted using the modifier.
      RobotContainer.m_intakeSystem.setFeederSystem(-1, 0.8);  
      RobotContainer.m_intakeSystem.setConveyorSpeed(-1, 0.5);
    }
    else{
      // RobotContainer.m_shooterSubsystem.setShooterSpeed(0, 0); //turns shooters off.
      /**
      RobotContainer.m_intakeSystem.setFeederSystem(0, 0); 
      RobotContainer.m_intakeSystem.setConveyorSpeed(0, 0);  */
    }
    
  }


  public void testingInit(){

    m_operatorController.X.whenActive(new PIDTurnLeft(90, 0.9));
    m_operatorController.Y.whenActive(new PIDDriveCommand(50, 0.9));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
