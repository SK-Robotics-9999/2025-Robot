// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldNavigation;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  public Field2d odometryField = new Field2d();

  
  double maximumSpeed = 5.033;
  double maxVelocityForPID = 2.0; //meters
  double maxRotationalVelocityForPID = 4.0;

  SwerveDrive swerveDrive;

  public static enum WantedState{
    DRIVE_TO_POINT,
    IDLE,
    TELEOP_DRIVE,
    ASSISTED_TELEOP_DRIVE
  }

  public static enum SystemState{
    DRIVING_TO_POINT,
    IDLING,
    TELEOP_DRIVING,
    ASSISTED_TELEOP_DRIVING
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;
  private WantedState previousWantedState = WantedState.IDLE;
  private Pose2d targetPose;

  private DoubleSupplier xAssister = ()->0.0;
  private DoubleSupplier yAssister = ()->0.0;
  private boolean assisterIsRobotRelative=false;


  private final PIDController autoCont = new PIDController(3, 0, 0.1);
  private final PIDController teleOpCont = new PIDController(3, 0, 0.1);

  private double staticFrictionConstant = 0.01;

  DoubleSupplier translationX = ()->0.0;
  DoubleSupplier translationY = ()->0.0;
  DoubleSupplier angularRotationX = ()->0.0;

  //2.24-1.88
  public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1, 2.75, 0.35);

  BooleanSupplier isRed = () -> {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  };

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

  }
  SwerveDriveTelemetry.verbosity = TelemetryVerbosity.NONE;

  swerveDrive.resetOdometry(new Pose2d(1, 1, new Rotation2d()));
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveDrive.updateOdometry();
    odometryField.setRobotPose(swerveDrive.getPose());

    // SmartDashboard.putNumber("swerve/wheelencoder", swerveDrive.getModules()[0].getPosition().distanceMeters);
    
    // SmartDashboard.putNumber("swerve/heading", swerveDrive.getOdometryHeading().getDegrees());
    // SmartDashboard.putNumber("swerve/x", swerveDrive.getPose().getX());
    // SmartDashboard.putNumber("swerve/y", swerveDrive.getPose().getY());
    
    //SmartDashboard.putBoolean("Navx/Connected", isGyroConnected());
    
    // SmartDashboard.putString("swerve/systemState", systemState.toString());
    // SmartDashboard.putBoolean("swerve/isTooClose", FieldNavigation.getTooCloseToTag(getPose()));
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
      case ASSISTED_TELEOP_DRIVE:
        return SystemState.ASSISTED_TELEOP_DRIVING;
    }
    return SystemState.IDLING;
  }

  public void ApplyStates(){
    switch (systemState) {
      case IDLING:
        //change nothing
        break;
      case DRIVING_TO_POINT:
        pidToPose(targetPose);
        break;
      case TELEOP_DRIVING:
        //TELEOP DRIVE
        driveAllianceManaged(translationX, translationY, angularRotationX);
        break;
      case ASSISTED_TELEOP_DRIVING:
        if(assisterIsRobotRelative){
          driveRobotRelativeCombined(translationX, translationY, angularRotationX);
        }
        else{
          driveAllianceManaged(()->translationX.getAsDouble()+xAssister.getAsDouble(), ()->translationY.getAsDouble()+yAssister.getAsDouble(), angularRotationX);
        }
        break;
    }}
    
    //idk if this works
    public void resetGyro(){
      swerveDrive.zeroGyro();
    }

  public void driveAllianceManaged(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
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

  //alliance managed, whatever
  public void driveRobotRelativeCombined(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    if(isRed.getAsBoolean()){
      swerveDrive.driveFieldOrientedAndRobotOriented(
        new ChassisSpeeds(
          -translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
          -translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
          angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity()
        ),
        new ChassisSpeeds(
          xAssister.getAsDouble(),
          yAssister.getAsDouble(),
          0.0
        )
      );
    }
    else{
      swerveDrive.driveFieldOrientedAndRobotOriented(
        new ChassisSpeeds(
          translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
          angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity()
        ),
        new ChassisSpeeds(
          xAssister.getAsDouble(),
          yAssister.getAsDouble(),
          0.0
        )
      );
    } 
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



  private void pidToPose(Pose2d pose){
    var frictionConstant = 0.0;
  
    var delta = pose.relativeTo(getPose());
    var linearDistance = delta.getTranslation().getNorm();

    if(linearDistance > Inches.of(1.0).in(Meters)){
      frictionConstant = staticFrictionConstant*maximumSpeed;
    }

    Rotation2d angle = delta.getTranslation().getAngle();
    double velocity = 0.0;

    if(DriverStation.isAutonomous()){
      velocity =  Math.min(
        Math.abs(autoCont.calculate(linearDistance, 0)) + frictionConstant,
        maxVelocityForPID);
    }
    else{
      velocity =  Math.min(
        Math.abs(teleOpCont.calculate(linearDistance, 0)) + frictionConstant,
        maxVelocityForPID);
    }

    double xcomponent = angle.getCos()*velocity;
    double ycomponent = angle.getSin()*velocity;
    double rotationalComponent = Math.min(delta.getRotation().getRadians()*2.0,
      maxRotationalVelocityForPID);

    swerveDrive.setChassisSpeeds(new ChassisSpeeds(
      xcomponent,
      ycomponent,
      rotationalComponent
    ));

    odometryField.getObject("pidtarget").setPose(pose);
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

  public void setWantedState(WantedState wantedState, DoubleSupplier xAssister, DoubleSupplier yAssister, DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX, boolean assisterIsRobotRelative){
    this.wantedState = wantedState;
    this.xAssister = xAssister;
    this.yAssister = yAssister;
    this.translationX = translationX;
    this.translationY = translationY;
    this.angularRotationX = angularRotationX;
    this.assisterIsRobotRelative = assisterIsRobotRelative;
  }
  
  //Meters
  public double getVelocity(){
    ChassisSpeeds speeds = swerveDrive.getRobotVelocity();
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  public double getAngularVelocity(){
    ChassisSpeeds speeds = swerveDrive.getRobotVelocity();
    return speeds.omegaRadiansPerSecond;
  }

  public boolean getOnTarget(){
    Transform2d delta = targetPose.minus(getPose());

    return delta.getTranslation().getNorm()<Inches.of(1.0).in(Meters) 
    && delta.getRotation().getDegrees()<3.0
    && systemState==SystemState.DRIVING_TO_POINT
    && getVelocity()<Inches.of(6.0).in(Meters);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return swerveDrive.getRobotVelocity();
  }



}
