// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Assigns drivetrain motors, this is kinda extra cause we don't really need it.
    public static final int DRIVETRAIN_FRONT_LEFT_TALON = 0;
    public static final int DRIVETRAIN_REAR_LEFT_TALON = 2;
    public static final int DRIVETRAIN_FRONT_RIGHT_TALON = 3;
    public static final int DRIVETRAIN_REAR_RIGHT_TALON = 4; 

    //Distance conversion for the Encoder from angular units (4096) to linear inches.
    public static final double DISTANCE_INCHES_ONE_UNIT = ((18.84955592/360)/0.09);  
}
