package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

//This class is to create a Button Board (PXN Arcade Fightstick) to readable values for the code.
public class ButtonBoard {
    public GenericHID controller;
    
    public Button A,B,X,Y,LB,RB,SL,SR,share,options;
    public Trigger RT,LT;

    public ButtonBoard(int port) {
        controller = new GenericHID(port);
        A = new JoystickButton(controller, 1);
        B = new JoystickButton(controller, 2);
        X = new JoystickButton(controller, 3);
        Y = new JoystickButton(controller, 4);
        LB = new JoystickButton(controller, 5);
        RB = new JoystickButton(controller, 6);
        share = new JoystickButton(controller, 7);
        options = new JoystickButton(controller, 8);
        SL = new JoystickButton(controller, 9);
        SR = new JoystickButton(controller, 10);
        LT = new Trigger(() -> controller.getRawAxis(2) > 0);
        RT = new Trigger(() -> controller.getRawAxis(3) > 0);
        

    }

    public double getYAxis(){
        return controller.getRawAxis(1);
    }

    public double getXAxis(){
        return controller.getRawAxis(0);
    }

    public boolean getLB(){
        return controller.getRawButton(5);
    }


    public boolean getRB(){
        return controller.getRawButton(6);
    }

    public boolean getX(){
        return controller.getRawButton(3);
    }

    public boolean getY(){
        return controller.getRawButton(4);
    }

    public boolean getA(){
        return controller.getRawButton(1);
    }

    public boolean getB(){
        return controller.getRawButton(2);
    }

    public boolean getLT(){
        return controller.getRawAxis(2) == 1;
    }

    public boolean getRT(){
        return controller.getRawAxis(3) == 1;
    }

    public boolean getSL(){
        return controller.getRawButton(9);
    }

    public boolean getSR(){
        return controller.getRawButton(10);
    }

    public boolean getShare(){
        return controller.getRawButton(7);
    }

    public boolean getOptions(){
        return controller.getRawButton(8);
    }

    public double getPOVVertical(){
        if(controller.getPOV() == 0){
            return 1;
        }else if(controller.getPOV() == 180){
            return -1;
        }else{
            return 0;
        }
    }

    public double getPOVHorizontal(){
        if(controller.getPOV() == 90){
            return 1;
        }else if(controller.getPOV() == 270){
            return -1;
        }else{
            return 0;
        }
    }
}
