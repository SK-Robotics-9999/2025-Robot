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
    //SmartDashboard.putBoolean("Navx/Callibrating", isGyroCallibrating());

    }
    
    //idk if this works
    public void resetGyro(){
      AHRS navx = (AHRS)swerveDrive.getGyro().getIMU();
      navx.reset();
    }

  public Command driveCommandAllianceManaged(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    BooleanSupplier isRed = () -> {
      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };

    return run(() -> {
      // Make the robot move
      if(isRed.getAsBoolean()){
        swerveDrive.drive(new Translation2d(-translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                            -translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                          angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                          true,
                          false);
      }
      else{
        swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                          angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                          true,
                          false);
      }
    });
  }

  private Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
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
  public boolean isGyroCallibrating(){
    AHRS navx = (AHRS)swerveDrive.getGyro().getIMU();
    if(navx.isCalibrating()){
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

  public Command pathToCoralLeft(){
    return new DeferredCommand(()->privatePathToPose(FieldNavigation.getCoralLeft(getPose())), Set.of(this));
  }

  public Command pathToCoralRight(){
    return new DeferredCommand(()->privatePathToPose(FieldNavigation.getCoralRight(getPose())), Set.of(this));
  }

  public Command pathToCoralSource(){
    return new DeferredCommand(()->privatePathToPose(FieldNavigation.getCoralSource(getPose())), Set.of(this));
  }

  public Command pathToOffsetLeft(){
    return new DeferredCommand(()->privatePathToOffset(FieldNavigation.getOffsetCoralLeft(getPose())), Set.of(this));
  }

  public Command pathToOffsetRight(){
    return new DeferredCommand(()->privatePathToOffset(FieldNavigation.getOffsetCoralRight(getPose())), Set.of(this));
  }

  public Command pidToCoralLeft(){
    return new DeferredCommand(()->pidToPoseFastCommand(FieldNavigation.getCoralLeft(getPose())), Set.of(this));
  }

  public Command pidToCoralRight(){
    return new DeferredCommand(()->pidToPoseFastCommand(FieldNavigation.getCoralRight(getPose())), Set.of(this));
  }

  public Command pidToCoralLeftHuman(){
    return new DeferredCommand(()->privatePathToPosePrecise(FieldNavigation.getCoralLeft(getPose())), Set.of(this));
  }

  public Command pidToCoralRightHuman(){
    return new DeferredCommand(()->privatePathToPosePrecise(FieldNavigation.getCoralRight(getPose())), Set.of(this));
  }

  public Command pidToCoralSource(){
    return new DeferredCommand(()->pidToPoseFastCommand(FieldNavigation.getCoralSource(getPose())), Set.of(this));
  }

  

  public boolean isNearEnoughToPID(Pose2d target){
    var distance = target.getTranslation().getDistance(getPose().getTranslation());
    var result = distance <= Inches.of(35).in(Meters);
    SmartDashboard.putBoolean("swerve/isNearEnough",result);
    return result;
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

  private void pidToPosePrecise(Pose2d pose){
    final double transltionP = 3.0*1.2*1.5*1.5*1.5;
    final double thetaP = 2.0*4*1.2;
    

    double clamp = 0.75;

    Pose2d delta = pose.relativeTo(swerveDrive.getPose());

    swerveDrive.setChassisSpeeds(new ChassisSpeeds(
      MathUtil.clamp(delta.getX()*transltionP,-clamp, clamp),
      MathUtil.clamp(delta.getY()*transltionP,-clamp,clamp),
      delta.getRotation().getRadians()*thetaP
    ));

    SmartDashboard.putNumber("swerve/pidTargetPoseX", pose.getX());
    SmartDashboard.putNumber("swerve/pidTargetPoseY", pose.getY());

  }

  public Command pidToPoseFastCommand(Pose2d poseSupplier){
    return run(()->pidToPoseFast(poseSupplier)).finallyDo(()->stop());
  }

  public Command pidToPosePreciseCommand(Pose2d poseSupplier){
    return run(()->pidToPosePrecise(poseSupplier));
  }

  private Command privatePathToOffset(Pose2d pose){
    // return pidToPoseCommand(pose).until(()->isNear(pose, 12.0)).withTimeout(4.5);
    return pidToPoseFastCommand(pose)
      .until(()->isNear(pose, 12.0))
      .withTimeout(4.5)
    ;
  }

  private Command privatePathToPosePrecise(Pose2d pose){
    return Commands.sequence(
      pidToPoseFastCommand(pose).until(()->isNearEnoughToPIDPrecise(pose)).withTimeout(4.5),
      pidToPosePreciseCommand(pose).until(()->isNearEnoughToScore(pose)).withTimeout(1.5),
      new InstantCommand(this::stop,this)
    );
  }
  private Command privatePathToPose(Pose2d pose){
    return Commands.sequence(
      pidToPoseFastCommand(pose)
      .until(()->isNearEnoughToPID(pose))
      .withTimeout(5),
      pidToPosePreciseCommand(pose).until(()->isNearEnoughToScore(pose)).withTimeout(3.0),
      new InstantCommand(this::stop,this)
    );
  }

  



}
