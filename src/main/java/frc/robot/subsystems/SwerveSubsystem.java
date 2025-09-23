// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.io.File;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldNavigation;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;


public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  public Field2d odometryField = new Field2d();

  
  double maximumSpeed = 5.033;

  SwerveDrive swerveDrive;

  public static enum WantedState{
    DRIVE_TO_POINT,
    IDLE,
    TELEOP_DRIVE
  }

  public static enum SystemState{
    DRIVING_TO_POINT,
    IDLING,
    TELEOP_DRIVING
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;
  private WantedState previousWantedState;
  private Pose2d targetPose;

  DoubleSupplier translationX = ()->0.0;
  DoubleSupplier translationY = ()->0.0;
  DoubleSupplier angularRotationX = ()->0.0;

  //2.24-1.88
  public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1, 2.75, 0.35);

  public SwerveSubsystem() {
    SmartDashboard.putData("odometryField", odometryField);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
        try
        {
          swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
          // swerveDrive.resetOdometry(new Pose2d(2,2,new Rotation2d()));
        } catch (Exception e)
        {
          throw new RuntimeException(e);

  }}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveDrive.updateOdometry();
    odometryField.setRobotPose(swerveDrive.getPose());

    SmartDashboard.putNumber("swerve/wheelencoder", swerveDrive.getModules()[0].getPosition().distanceMeters);
    
    SmartDashboard.putNumber("swerve/heading", swerveDrive.getOdometryHeading().getDegrees());
    SmartDashboard.putNumber("swerve/x", swerveDrive.getPose().getX());
    SmartDashboard.putNumber("swerve/y", swerveDrive.getPose().getY());
    
    //SmartDashboard.putBoolean("Navx/Connected", isGyroConnected());

    systemState = handleStateTransitions();
    ApplyStates();

  }

  public SystemState handleStateTransitions(){
    switch (wantedState){
      case TELEOP_DRIVE:
        return SystemState.TELEOP_DRIVING;
      case IDLE:
        return SystemState.IDLING;
      case DRIVE_TO_POINT:
        return SystemState.DRIVING_TO_POINT;
    }
    return SystemState.IDLING;
  }

  public void ApplyStates(){
    switch (systemState) {
      case IDLING:
        //change nothing
        break;
      case DRIVING_TO_POINT:
        pidToPoseFast(targetPose);
        break;
      case TELEOP_DRIVING:
        //TELEOP DRIVE
        driveAllianceManaged(translationX, translationY, angularRotationX);
        break;
    }}
    
  public void resetGyro(){
    swerveDrive.zeroGyro();
  }

  public void driveAllianceManaged(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){

    SmartDashboard.putNumber("/swerve/controllerX", translationX.getAsDouble());
    BooleanSupplier isRed = () -> {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };

    if(isRed.getAsBoolean()){
      swerveDrive.drive(
        new Translation2d(
          -translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
          -translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()
        ),
        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
        true,
        false
      );
    }
    else{
      swerveDrive.drive(
        new Translation2d(
          translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()
        ),
        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
        true,
        false
      );
    }
  }

  public Command driveCommandRobotRelative(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                        false,
                        false);
    });
  }

  private void stop(){
    swerveDrive.setChassisSpeeds(new ChassisSpeeds());
  }

  public boolean isGyroConnected(){
    AHRS navx = (AHRS)swerveDrive.getGyro().getIMU();
    if(navx.isConnected()){
      return true;
    }
    return false;
  }

  public Pose2d getPose(){
    return swerveDrive.getPose();
  }
  public Rotation2d getHeading(){
    return swerveDrive.getGyro().getRotation3d().toRotation2d();
  }

  public boolean isNearEnoughToPIDAuto(Pose2d target){
    var distance = target.getTranslation().getDistance(getPose().getTranslation());
    var result = distance <= Inches.of(5.0).in(Meters);
    SmartDashboard.putBoolean("swerve/isNearEnough",result);
    return result;
  }

  public boolean isNearEnoughToPIDPrecise(Pose2d target){
    var distance = target.getTranslation().getDistance(getPose().getTranslation());
    var result = distance <= Inches.of(48).in(Meters);
    SmartDashboard.putBoolean("swerve/isNearEnough",result);
    return result;
  }

  public boolean isNearEnoughToScore(Pose2d target){
    var distance = target.getTranslation().getDistance(getPose().getTranslation());
    return distance <= Inches.of(1).in(Meters);
  }

  public boolean isNear(Pose2d target,double distance){
    var delta = target.getTranslation().getDistance(getPose().getTranslation());
    return delta <= Inches.of(distance).in(Meters);
  }



  private void pidToPoseFast(Pose2d pose){
    final double transltionP = 3.0*1.2;
    final double thetaP = 2.0*4*1.2 ;

    double clamp = 2.0;

    Pose2d delta = pose.relativeTo(swerveDrive.getPose());

    swerveDrive.setChassisSpeeds(new ChassisSpeeds(
      MathUtil.clamp(delta.getX()*transltionP,-clamp, clamp),
      MathUtil.clamp(delta.getY()*transltionP,-clamp,clamp),
      delta.getRotation().getRadians()*thetaP
    ));

    odometryField.getObject("pidtarget").setPose(pose);
    SmartDashboard.putNumber("swerve/pid/deltax", pose.getX());
    SmartDashboard.putNumber("swerve/pid/deltay", pose.getY());

  }

  public void SetWantedState(WantedState wantedState){
    this.wantedState = wantedState;
  }

  public void SetWantedState(WantedState wantedState, Pose2d targetPose){
    this.targetPose = targetPose;
    this.wantedState = wantedState;
  }

  public void setWantedState(WantedState wantedState, DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    this.wantedState = wantedState;
    this.translationX = translationX;
    this.translationY = translationY;
    this.angularRotationX = angularRotationX;
  }
  



}
