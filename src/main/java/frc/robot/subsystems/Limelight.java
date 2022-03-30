package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.*;

public class Limelight extends SubsystemBase {
    private NetworkTable table;
    private HttpCamera LLFeed;
    private String LLName;

    //THIS ENTIRE CLASS NEEDS ORGANIZING

    public Limelight(String limelightName) {
        table = NetworkTableInstance.getDefault().getTable(limelightName);
        LLName = limelightName;
        //System.out.println(NetworkTableInstance.getDefault().getTable("CameraPublisher").getEntry(limelightName).getString("source"));
    }

    //Checks if the Limelight can see a target. Returns true if it can, returns false if it can't.
    public boolean canSeeTarget() {
        return table.getEntry("tv").getDouble(0) == 1;
    }

    //Gets the X (horizontal) axis of the target relative to the center of the camera.
    public double offsetX() {
        return table.getEntry("tx").getDouble(0.00);
        
    }

    public double rotationY() {
        return table.getEntry("ts").getDouble(0.00);
    }

    //Gets the Y (vertical) axis of the target relative to the center of the camera.
    public double offsetY() {
        return table.getEntry("ty").getDouble(0.0);
    }

    //Checks what percent of the camera lens the target is occupying.
    public double areaPercent() {
        return table.getEntry("ta").getDouble(0.00);
    }

    //a value determined on the number of pipelines we have (created within limelight-local:5801)
    public void setPipeline(int mode) {
        /** 
         * How this is set up in our Limelight:
         * 0: Limelight vision mode
         * 1: Driver Camera mode
        */
        
        table.getEntry("pipeline").setNumber(mode);
    }
    //a value setting the LED mode of the limelight.
    public void setLEDMode(int mode) {
        /*
        0	use the LED Mode set in the current pipeline
        1	force off
        2	force blink
        3	force on
        */

        table.getEntry("ledMode").setNumber(mode);

    }
    //a value between 0 and 1, 0 being on, 1 being off 
    public void setCamMode(int mode) {
        
        table.getEntry("camMode").setNumber(mode);
        /*
        0	Vision processor
        1	Driver Camera (Increases exposure, disables vision processing)
        */
    }

    public void setLightPipeline() {

    }

    // public double steeringAdjust() {
    //     float kp = -.05f;//Adjusts the value returned from Limelight
    //     float minCommand = .005f;//Minimum value a value can have
    //     float steeringAdjust = 0.05f;//Default value of adjust
    //     float tx = (float)offsetX();
    //     //SmartDashboard.setDefaultNumber("TX", tx);
    //     float headingError = -tx;
      
    //     if(tx > 1) {
    //         steeringAdjust = kp*headingError -minCommand;
    //     }else if (tx < 1){
    //         steeringAdjust = kp*headingError + minCommand;
    //     }
    //     return steeringAdjust;
    //   }

    public double steeringAdjust() {
        float kp = -.22f; //Adjusts the value returned from Limelight
        float minCommand = .01f; //Minimum value a value can have
        float steeringAdjust = 0.07f; //Default value of adjust
        float tx = (float)offsetX();
        //SmartDashboard.setDefaultNumber("TX", tx);
        float headingError = -tx;
        //System.out.println(tx);
        if(tx > 1) {
            steeringAdjust = kp*headingError -minCommand;
        }else if (tx < 1){
            steeringAdjust = kp*headingError + minCommand;
        }
            if(steeringAdjust > 1){
                steeringAdjust = 1;
            }else if(steeringAdjust < -1){
                steeringAdjust = -1;
            }
        return steeringAdjust;
      }

      public double distanceAdjust(){
        float KpDist = -0.1f;
        float Yoffset = (float)offsetY();
        float distance_error = -Yoffset;
        float distance_adjust = distance_error*KpDist;
        if (distance_adjust > 1){
          distance_adjust = 1;
        }else if(distance_adjust <-1){
          distance_adjust = -1;
        }
        return distance_adjust;
      }

      public void Vision() {
        ShuffleboardTab mainTab = Shuffleboard.getTab("SmartDashboard");
        if(LLName==("limelight")){
            LLFeed = new HttpCamera("limelight", "http://10.3.69.44:5800/stream.mjpeg");
        }
        mainTab.add("LimeLight", LLFeed).withPosition(0, 0).withSize(15, 8);

    }

    public double getDistance() {
        double targetOffsetAngle_Vertical = offsetY();
        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = Constants.limelightMountAngleDegrees;
        // distance from the center of the Limelight lens to the floor
        double limelightHeight = Constants.limelightHeight;
        // distance from the target to the floor
        double goalHeightInches = Constants.goalHeightInches;
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightHeight)/Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalInches;
    }
}