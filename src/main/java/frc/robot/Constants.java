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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class MotorConstants {
    public static final int kVortexFreeSpeedRPM = 6784;
    public static final int kNeoFreeSpeedRPM = 5676;

    //CAN ID
    public static final int kFLDriveID = 1;
    public static final int kFRDriveID = 2;
    public static final int kBLDriveID = 3;
    public static final int kBRDriveID = 4;

    public static final int kFLTurnID = 5;
    public static final int kFRTurnID = 6;
    public static final int kBLTurnID = 7;
    public static final int kBRTurnID = 8;

    public static final int kElevatorFrontID = 9; //in heading of robot
    public static final int kElevatorBackID = 10;

    public static final int kArmID = 11;
    public static final int kSuctionID = 12;

    public static final int kIntakePivotID = 13;
    public static final int kIntakeRollersID = 14;

    public static final int kFeederLeftID = 15; //in heading of robot
    public static final int kFeederRightID = 16;

    //Analog Port ID
    public static final int kVacuumSensorPortID = 0; 
  }

  public static class DriveConstants {

  }
  
  public static class ArmConstants{
    public static final double intakeAngle = -90;
    public static final double lowestAtZeroElevator = -35;
    //THESE ALL NEED TO BE CHANGED
    public static final double scoreL1 = 0;
    public static final double scoreL2 = 5;
    public static final double scoreL3 = 10;
    public static final double scoreL4 = 20;
    public static final double barge = 90;
  }

  public static class ElevatorConstants{
    public static final double preIntake = 16.0;
    public static final double intake = 13.0;
  }

}
