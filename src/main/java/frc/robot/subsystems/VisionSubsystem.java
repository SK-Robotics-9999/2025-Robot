// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.lang.StackWalker.Option;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new Vision. */
  
  SwerveSubsystem swerve;
  SuperStructure superStructure;

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();

  NetworkTableInstance table = NetworkTableInstance.getDefault();
  
  Optional<PhotonCamera> leftCamera;

  Optional<PhotonCamera> rightCamera;

  Optional<PhotonCamera> objectCamera;

  Transform3d leftCameraTransform = new Transform3d(new Translation3d(
    Inches.of(-5.378).in(Meters),
    Inches.of(5.5).in(Meters), 
    Inches.of(7.684).in(Meters)),
    new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(180.0))
  );

  Transform3d rightCameraTransform = new Transform3d(new Translation3d(
    Inches.of(-5.378).in(Meters),
    Inches.of(-5.5).in(Meters), 
    Inches.of(7.684).in(Meters)),
    new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(180.0))
  );
  
  PhotonPoseEstimator leftPoseEstimator = new PhotonPoseEstimator(
    aprilTagFieldLayout, 
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    leftCameraTransform
  );

  PhotonPoseEstimator rightPoseEstimator = new PhotonPoseEstimator(
    aprilTagFieldLayout, 
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    rightCameraTransform
  );

  private final double objectHeight = Inches.of(37.792).in(Meters);
  //robotToCamera
  Transform3d objectCameraTransform = new Transform3d(new Translation3d(
      Inches.of(6.182).in(Meters),
      Inches.of(-7.582).in(Meters),
      objectHeight
    ),
    new Rotation3d(0.0, Math.toRadians(25.77), Math.toRadians(14.08))
  );

  Pose2d lastSeenObjectPose = new Pose2d();
  double lastSeenTimestamp = -1.0;

  Field2d visionField = new Field2d();

  /** Creates a new Vision. */
  public VisionSubsystem(SwerveSubsystem swerve, SuperStructure superStructure) {
    this.swerve = swerve;
    this.superStructure = superStructure;
    SmartDashboard.putData("visionfield", visionField);
      
    try{
      leftCamera = Optional.of(new PhotonCamera("left_cam"));
      if (!leftCamera.get().isConnected()) {
        leftCamera = Optional.empty();
      }
    }catch(Error e){
      System.err.print(e);
      leftCamera = Optional.empty();
    }

    try{
      rightCamera = Optional.of(new PhotonCamera("right_cam"));
      if (!rightCamera.get().isConnected()) {
        rightCamera = Optional.empty();
      }
    }catch(Error e){
      System.err.print(e);
      rightCamera = Optional.empty();
    }

    try{
      objectCamera = Optional.of(new PhotonCamera("object_cam"));
      if (!objectCamera.get().isConnected()) {
        objectCamera = Optional.empty();
      }
    }catch(Error e){
      System.err.print(e);
      objectCamera = Optional.empty();
    }
   
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("vision/leftCameraPresent", leftCamera.isPresent());
    SmartDashboard.putBoolean("vision/rightCameraPresent", rightCamera.isPresent());
    SmartDashboard.putBoolean("vision/objectCameraPresent", objectCamera.isPresent());

    // Optional<Translation2d> object = getObjectTranslationRelative();
    // if(object.isPresent()){
    //   visionField.getObject("relativePose").setPose(new Pose2d(object.get(), new Rotation2d()));
    // }
    visionField.setRobotPose(swerve.getPose());

    updateOdometry();

  }

  

  public void updateOdometry(){
  
    if(leftCamera.isPresent()){
      updateCameraSideOdometry(leftPoseEstimator, leftCamera.get());
    }
    if(rightCamera.isPresent()){
      updateCameraSideOdometry(rightPoseEstimator, rightCamera.get());
    }
  }

  private void updateCameraSideOdometry(PhotonPoseEstimator photonPoseEstimator, PhotonCamera camera){

    List<PhotonPipelineResult> latestResults = camera.getAllUnreadResults();
    for(PhotonPipelineResult result : latestResults){
      if (
        !result.hasTargets() 
        || result.getBestTarget().getPoseAmbiguity()>0.4 
        || swerve.getVelocity()>2 
        || swerve.getAngularVelocity()>Math.toRadians(45.0)
        || superStructure.isIntaking()
      ){
        return;
      }

      Pose2d position = aprilTagFieldLayout.getTagPose(result.getBestTarget().getFiducialId()).orElse(new Pose3d()).toPose2d();
      Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update(result);
      
      if(estimatedPose.isPresent()){
        //Get some data to help diagnose issues
        visionField.getObject(camera.getName()).setPose(estimatedPose.get().estimatedPose.toPose2d());


        //Manage std deviations for this result
        // Matrix<N3, N1> stddev;
        // stddev = getStandardDeviationNearest(estimatedPose.get());

        //we don't want to update swerve, should be dependent on quest, but need a plan for redundancy.  maybe update directly when close like jib
        swerve.swerveDrive.addVisionMeasurement(
          estimatedPose.get().estimatedPose.toPose2d(),
          estimatedPose.get().timestampSeconds
        );
        
        Transform2d transform = position.minus(estimatedPose.get().estimatedPose.toPose2d());
        SmartDashboard.putNumber("/vision/x", transform.getX());

        visionField.getObject("robotToTag").setPose(new Pose2d(transform.getTranslation(), transform.getRotation()));
      }
    }

  }

  public Optional<Translation2d> getObjectTranslationRelative(){
    if(objectCamera.isEmpty()){return Optional.empty();}
    
    List<PhotonPipelineResult> results = objectCamera.get().getAllUnreadResults();
    if(results.isEmpty()){return Optional.empty();}
    
    PhotonPipelineResult result = results.get(results.size()-1);
    
    if(!result.hasTargets()){return Optional.empty();}

    PhotonTrackedTarget target = result.getBestTarget();

    double pitch = -Math.toRadians(target.getPitch()); // in radians, need to check this
    // SmartDashboard.putNumber("vision/object/pitch", pitch);
    double yaw = -Math.toRadians(target.getYaw()); // in radians

    double truePitch = pitch + objectCameraTransform.getRotation().getY();
    // SmartDashboard.putNumber("vision/object/truePitch", Math.toDegrees(truePitch));

    double zDiff = objectCameraTransform.getTranslation().getZ()-Inches.of(4.5).in(Meters);

    double range = zDiff / Math.tan(truePitch); //?, pls check
    // SmartDashboard.putNumber("vision/object/range", range);
    
    double xCam = range * Math.cos(yaw);
    double yCam = range * Math.sin(yaw);
    // SmartDashboard.putNumber("vision/object/xCam", xCam);
    // SmartDashboard.putNumber("vision/object/yCam", yCam);

    
    Translation2d cameraRelative = new Translation2d(xCam, yCam);
    
    // Rotate into robot frame and offset by camera’s position
    Translation2d robotHeadingRelative = cameraRelative.rotateBy(new Rotation2d(objectCameraTransform.getRotation().getZ()));
    // visionField.getObject("camToTargetTranslation").setPose(new Pose2d(robotHeadingRelative, new Rotation2d()));
    
    Translation2d robotRelative = robotHeadingRelative.plus(objectCameraTransform.getTranslation().toTranslation2d());
    // visionField.getObject("robotToTargetTranslation").setPose(new Pose2d(robotRelative, new Rotation2d()));
    
    return Optional.of(robotRelative);
  }

  public Optional<Pose2d> getObjectFieldRelativePose(Pose2d robotPose){
    Optional<Translation2d> translation = getObjectTranslationRelative();
    if(translation.isEmpty()){return Optional.empty();}
    // final double middleToIntake = Inches.of(16.0).in(Meters);
    final double middleToIntake = Inches.of(6).in(Meters);
    Translation2d ojbectIntakeRelative = translation.get().minus(new Translation2d(middleToIntake, 0.0));
    
    //technically off by 16 inches, but whatever.  
    Pose2d fieldRelativeNotePose = robotPose.plus(new Transform2d(ojbectIntakeRelative, ojbectIntakeRelative.getAngle()));
    lastSeenObjectPose = fieldRelativeNotePose;
    lastSeenTimestamp = Timer.getFPGATimestamp();
    // visionField.getObject("coral").setPose(fieldRelativeNotePose); // whatevvvaaa
    return Optional.of(fieldRelativeNotePose);
  }

  public Pose2d getLastSeenObjectPose(){
    return lastSeenObjectPose;
  }

  public double getLastSeenTimestamp(){
    return lastSeenTimestamp;
  }
}
