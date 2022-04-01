package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;

//This class is to create a Button Board (PXN Arcade Fightstick) to readable values for the code.
public class ButtonBoard {
    public GenericHID controller;
    
    public ButtonBoard(int port) {
        controller = new GenericHID(port);
    }

    public double getYAxis(){
        return controller.getRawAxis(1);
    }

    public double getXAxis(){
        return controller.getRawAxis(0);
    }

    public boolean getLB(){
        return controller.getRawButton(4);
    }

    public boolean getRB(){
        return controller.getRawButton(5);
    }

    public boolean getX(){
        return controller.getRawButton(2);
    }

    public boolean getY(){
        return controller.getRawButton(3);
    }

    public boolean getA(){
        return controller.getRawButton(0);
    }

    public boolean getB(){
        return controller.getRawButton(1);
    }

    public boolean getLT(){
        return controller.getRawAxis(2) == 1;
    }

    public boolean getRT(){
        return controller.getRawAxis(3) == 1;
    }

    public boolean getSL(){
        return controller.getRawButton(8);
    }

    public boolean getSR(){
        return controller.getRawButton(9);
    }

    public boolean getShare(){
        return controller.getRawButton(6);
    }

    public boolean getOptions(){
        return controller.getRawButton(7);
    }
}
