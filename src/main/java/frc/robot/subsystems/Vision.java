// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  
  SwerveSubsystem swerve ; 
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();

  NetworkTableInstance table = NetworkTableInstance.getDefault();
  
  Optional<PhotonCamera> leftCamera;

  Transform3d leftCameraTransform = new Transform3d(new Translation3d(
    Inches.of(-5.378).in(Meters),
    Inches.of(5.5).in(Meters), 
    Inches.of(7.684).in(Meters)),
    new Rotation3d(0.0, 20, Math.toRadians(180.0))
  );
  
  PhotonPoseEstimator leftPoseEstimator = new PhotonPoseEstimator(
    aprilTagFieldLayout, 
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    leftCameraTransform
  );

  Field2d visionField2d = new Field2d();

  

  /** Creates a new Vision. */
  public Vision(SwerveSubsystem swerve) {
    this.swerve = swerve;
    SmartDashboard.putData("visionfield", visionField2d);

  
    
    try{
      leftCamera = Optional.of(new PhotonCamera("Arducam_OV9782_Shooter_Vision"));
    
    }catch(Error e){
      System.err.print(e);
      leftCamera = Optional.empty();
    }
   
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("vision/leftCamera", leftCamera.isPresent());

  }

  

  public void updateOdometry(){
  
    if(leftCamera.isPresent()){
      updateCameraSideOdometry(leftPoseEstimator, leftCamera.get());
    }
  }
  private void updateCameraSideOdometry(PhotonPoseEstimator photonPoseEstimator, PhotonCamera camera){

    var latesResults = camera.getAllUnreadResults();
    for(PhotonPipelineResult result : latesResults){
      Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update(result);
      
      if(estimatedPose.isPresent()){
        //Get some data to help diagnose issues
        double neartestTag=distanceToNearestTag(estimatedPose.get());
        SmartDashboard.putNumber("camera/"+camera.getName()+"/nearestTagDist", neartestTag);
        visionField2d.getObject(camera.getName()).setPose(estimatedPose.get().estimatedPose.toPose2d());


        //Manage std deviations for this result
        Matrix<N3, N1> stddev;
        stddev = getStandardDeviationNearest(estimatedPose.get());

        
        swerve.swerveDrive.addVisionMeasurement(
          estimatedPose.get().estimatedPose.toPose2d(),
          result.getTimestampSeconds()//,
          // stddev
        );
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

    return Math.min(bot.getDistance(estimate.getTranslation()), distanceToEstimatedPose);
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
  
  public double getRotationDouble(){
    var value = getRotationToObject().orElse(new Rotation2d()).rotateBy(swerve.getHeading()).getRotations();
    return value;
  }

  public Optional<Rotation2d> getRotationToObject(){
    
    return Optional.empty();
    
  }


}
