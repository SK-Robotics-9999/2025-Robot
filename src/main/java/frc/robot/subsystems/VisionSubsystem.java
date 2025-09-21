// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new Vision. */
  
  SwerveSubsystem swerve ; 
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();

  NetworkTableInstance table = NetworkTableInstance.getDefault();
  
  Optional<PhotonCamera> leftCamera;

  Optional<PhotonCamera> objectCamera;

  Transform3d leftCameraTransform = new Transform3d(new Translation3d(
    Inches.of(-5.378).in(Meters),
    Inches.of(5.5).in(Meters), 
    Inches.of(7.684).in(Meters)),
    new Rotation3d(0.0, Math.toRadians(20.0), Math.toRadians(180.0))
  );
  
  PhotonPoseEstimator leftPoseEstimator = new PhotonPoseEstimator(
    aprilTagFieldLayout, 
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    leftCameraTransform
  );

  private final double objectHeight = Inches.of(20.0).in(Meters);
  private final double objectPitch = Math.toRadians(-45.0);
  private final double objectYaw = Math.toRadians(45.0);
  //not right
  Transform3d objectCameraTransform = new Transform3d(new Translation3d(
      Inches.of(5.0).in(Meters),
      Inches.of(-5.0).in(Meters),
      objectHeight
    ),
    new Rotation3d(0.0, objectPitch, objectYaw)
  );

  Field2d visionField = new Field2d();

  /** Creates a new Vision. */
  public VisionSubsystem(SwerveSubsystem swerve) {
    this.swerve = swerve;
    SmartDashboard.putData("visionfield", visionField);
      
    try{
      leftCamera = Optional.of(new PhotonCamera("Arducam_OV9782_Shooter_Vision"));
    }catch(Error e){
      System.err.print(e);
      leftCamera = Optional.empty();
    }

    try{
      objectCamera = Optional.of(new PhotonCamera("objectCamera"));
    }catch(Error e){
      System.err.print(e);
      objectCamera = Optional.empty();
    }
   
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("vision/leftCameraPresent", leftCamera.isPresent());
    SmartDashboard.putBoolean("vision/objectCameraPresent", objectCamera.isPresent());



  }

  

  public void updateOdometry(){
  
    if(leftCamera.isPresent()){
      updateCameraSideOdometry(leftPoseEstimator, leftCamera.get());
    }
  }

  private void updateCameraSideOdometry(PhotonPoseEstimator photonPoseEstimator, PhotonCamera camera){

    List<PhotonPipelineResult> latestResults = camera.getAllUnreadResults();
    for(PhotonPipelineResult result : latestResults){
      Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update(result);
      
      if(estimatedPose.isPresent()){
        //Get some data to help diagnose issues
        double neartestTag=distanceToNearestTag(estimatedPose.get());
        SmartDashboard.putNumber("camera/"+camera.getName()+"/nearestTagDist", neartestTag);
        visionField.getObject(camera.getName()).setPose(estimatedPose.get().estimatedPose.toPose2d());


        //Manage std deviations for this result
        // Matrix<N3, N1> stddev;
        // stddev = getStandardDeviationNearest(estimatedPose.get());

        
        //we don't want to update swerve, should be dependent on quest, but need a plan for redundancy.  maybe update directly when close like jib
        // swerve.swerveDrive.addVisionMeasurement(
        //   estimatedPose.get().estimatedPose.toPose2d(),
        //   result.getTimestampSeconds(),
        //   stddev
        // );
        
      }
    }
  }

  public double distanceToNearestTag(EstimatedRobotPose estimatedPose){
    var targets = estimatedPose.targetsUsed;
    double distanceToEstimatedPose = 10; //m
    var estimate = estimatedPose.estimatedPose.toPose2d();

    for (var tgt : targets){
      var tagPose = aprilTagFieldLayout.getTagPose(tgt.getFiducialId());
      if(tagPose.isEmpty()) continue;

      var tag = tagPose.get().toPose2d().getTranslation();
      distanceToEstimatedPose = Math.min(tag.getDistance(estimate.getTranslation()), distanceToEstimatedPose) ;
    }

    var bot = swerve.getPose().getTranslation();

    return Math.min(bot.getDistance(estimate.getTranslation()), distanceToEstimatedPose); //??? wtf
  }
  

  public Matrix<N3, N1> getStandardDeviationNearest(EstimatedRobotPose estimatedPose){
    //Get one that's easy to work with
    var stddev = VecBuilder.fill(1, 1, Double.MAX_VALUE);

    var targets = estimatedPose.targetsUsed;

    // Avoid using tags with high ambiguity; These generate jitter and bad poses
    for (var tgt : targets){
      if(tgt.poseAmbiguity > 0.2) return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    //Why are we even here then?
    if(targets.isEmpty()) return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

    double poseToTagDistance = 10; //m
    double distanceToCurrentPose = 10; //m
    var estimate = estimatedPose.estimatedPose.toPose2d();
    var bot = swerve.getPose().getTranslation();

    //See how far away we're being told to move
    distanceToCurrentPose = Math.min(bot.getDistance(estimate.getTranslation()), distanceToCurrentPose);    
    
    //Check how far away the tag is from where the tags say we *should* be.
    for (var tgt : targets){
      var tagPose = aprilTagFieldLayout.getTagPose(tgt.getFiducialId());
      if(tagPose.isEmpty()) continue;

      var tag = tagPose.get().toPose2d().getTranslation();
      poseToTagDistance = Math.min(tag.getDistance(estimate.getTranslation()), poseToTagDistance) ;
    }

    //our base confidence
    var scalar = 1.0;

    var devTagDistance = 1;
    if( poseToTagDistance > 2) devTagDistance*=4;
    if( poseToTagDistance <0.3) devTagDistance*=4;

    return stddev.times(devTagDistance);

  }

  //poseRelativeToRobot
  public Optional<Translation2d> getObjectTranslationRelative(){
    if(objectCamera.isEmpty()){return Optional.empty();}
    
    List<PhotonPipelineResult> results = objectCamera.get().getAllUnreadResults();
    if(results.isEmpty()){return Optional.empty();}

    PhotonPipelineResult result = results.get(results.size()-1);

    if(!result.hasTargets()){return Optional.empty();}

    PhotonTrackedTarget target = result.getBestTarget();

    //degrees, angle from horizontal
    double pitch = target.getPitch()+Math.toDegrees(objectPitch);
    SmartDashboard.putNumber("/vision/object/pitch", pitch);

    //pls check this geometry
    
    //height of camera - height of middle of coral
    final double height = objectCameraTransform.getZ()-Inches.of(4.5/2.0).in(Meters);

    //due to alternate interior angles, our pitch is equivalent to the angle starting from the ground-coral to coral-camera
    //opposite(height)/x distance = tan(pitch), opposite/tan(pitch)
    double relativeX = height/Math.tan(Math.toRadians(pitch));

    double hypot = Math.hypot(relativeX, height);

    double yaw = target.getYaw();

    double relativeY = Math.tan(yaw)*hypot;

    Translation2d relativeTranslation = new Translation2d(relativeX, relativeY);

    Translation2d robotRelativeTranslation = relativeTranslation.rotateAround(objectCameraTransform.getTranslation().toTranslation2d(), new Rotation2d(-yaw));

    return Optional.of(robotRelativeTranslation);
  }
  
  public double getRotationDouble(){
    var value = getRotationToObject().orElse(new Rotation2d()).rotateBy(swerve.getHeading()).getRotations();
    return value;
  }

  public Optional<Rotation2d> getRotationToObject(){
    
    return Optional.empty();
    
  }


}
