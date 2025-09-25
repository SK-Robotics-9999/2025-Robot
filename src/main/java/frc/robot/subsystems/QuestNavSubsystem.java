// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
  QuestNav questNav = new QuestNav();
  Field2d questField = new Field2d();
  
  /** Creates a new QuestNavSubsystem. */
  public QuestNavSubsystem() {
    // questNav.setPose(new Pose2d(1,2,new Rotation2d()));
    questNav.setPose(new Pose2d());
    SmartDashboard.putData("questField", questField);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      questNav.commandPeriodic();

      PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

      if (poseFrames.length > 0) {
        // Get the most recent Quest pose
        Pose2d questPose = poseFrames[poseFrames.length - 1].questPose();

        // Transform by the mount pose to get your robot pose
        Pose2d robotPose = questPose;
        questField.setRobotPose(robotPose);
        SmartDashboard.putNumber("/questnav/x", robotPose.getX());
        SmartDashboard.putNumber("/questnav/y", robotPose.getY());
        SmartDashboard.putNumber("/questnav/rotDeg", robotPose.getRotation().getDegrees());
        SmartDashboard.putBoolean("/questnav/updating", true);
      }
      else{
        SmartDashboard.putBoolean("/questnav/updating", false);
      }
      SmartDashboard.putBoolean("/questnav/tracking", questNav.isTracking());
      SmartDashboard.putNumber("/questnav/lost", questNav.getTrackingLostCounter().orElse(-1));
  }
}
