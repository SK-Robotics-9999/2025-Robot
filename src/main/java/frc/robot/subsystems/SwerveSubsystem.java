// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.io.File;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldNavigation;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  public Field2d odometryField = new Field2d();

  
  double maximumSpeed = 5.033;
  public static final double MAXVELOCITYFORPID = 5.0; //meters
  double maxVelocityForPID = MAXVELOCITYFORPID; //meters
  public static final double MAXOMEGA = 4.0; //radians per second
  double maxRotationalVelocityForPID = MAXOMEGA;

  SwerveDrive swerveDrive;

  public static enum WantedState{
    DRIVE_TO_POINT,
    IDLE,
    TELEOP_DRIVE,
    ASSISTED_TELEOP_DRIVE,
    STOP
  }

  public static enum SystemState{
    DRIVING_TO_POINT,
    IDLING,
    TELEOP_DRIVING,
    ASSISTED_TELEOP_DRIVING,
    STOPPED
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;
  private WantedState previousWantedState = WantedState.IDLE;
  private Pose2d targetPose = new Pose2d();

  private DoubleSupplier xAssister = ()->0.0;
  private DoubleSupplier yAssister = ()->0.0;
  private boolean assisterIsRobotRelative=false;


  private final PIDController autoCont = new PIDController(3.0, 0, 0.3);
  private final PIDController teleOpCont = new PIDController(3.0, 0, 0.3);

  private final PIDController thetaCont = new PIDController(2.5, 0, 0);

  private double staticFrictionConstant = 0.03;

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
  swerveDrive.resetOdometry(new Pose2d(1,1,new Rotation2d()));

  thetaCont.enableContinuousInput(-Math.PI, Math.PI);
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

    SmartDashboard.putString("swerve/currentState", systemState.toString());

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
      case STOP:
        return SystemState.STOPPED;
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
      case STOPPED:
        driveAllianceManaged(()->0, ()->0, ()->0);
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
    return result;
  }

  public boolean isNearEnoughToPIDPrecise(Pose2d target){
    var distance = target.getTranslation().getDistance(getPose().getTranslation());
    var result = distance <= Inches.of(48).in(Meters);
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

    if(linearDistance > Inches.of(0.5).in(Meters)){
      frictionConstant = staticFrictionConstant*maximumSpeed;
    }

    linearDistance = Math.pow(linearDistance, 5.0 / 6.0);

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
    double rotationalComponent = Math.min(delta.getRotation().getRadians()*5.0,
      maxRotationalVelocityForPID);
    // double rotationalComponent = Math.min(thetaCont.calculate(pose.getRotation().getRadians(), getPose().getRotation().getRadians()),
    //   maxRotationalVelocityForPID);

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
    // && getVelocity()<Inches.of(6.0).in(Meters)
    ;
  }
  
  /**
   * @param angularTolerance in degrees
   */
  public boolean getOnTarget(Pose2d pose, double translationalTolerance, double angularTolerance, double velocityTolerance){
    Transform2d delta = pose.minus(getPose());
    
    return delta.getTranslation().getNorm()<translationalTolerance
    && delta.getRotation().getDegrees()<angularTolerance
    && systemState==SystemState.DRIVING_TO_POINT
    && getVelocity()< velocityTolerance;    
  }

  /**
   * @param angularTolerance in degrees
   */
  public boolean getOnTarget(double translationalTolerance, double angularTolerance, double velocityTolerance){
    Transform2d delta = targetPose.minus(getPose());
    
    return delta.getTranslation().getNorm()<translationalTolerance
    && delta.getRotation().getDegrees()<angularTolerance
    && systemState==SystemState.DRIVING_TO_POINT
    && getVelocity()< velocityTolerance;    
  }
  
  // I lowkey don't know if the edge cases will destroy this.
  public boolean getOnScoringPose(){
    Transform2d tagToTarget = targetPose.minus(FieldNavigation.getNearestReef(getPose()));
    
    return this.getOnTarget()
    && tagToTarget.getX()<FieldNavigation.botCenterToRearX+1
    && Math.abs(tagToTarget.getMeasureY().in(Inches)) <25.0;
  }
  
  public boolean getCloseEnough(){
    Transform2d delta = targetPose.minus(getPose());

    return delta.getTranslation().getNorm()<Inches.of(6.0).in(Meters) 
    && delta.getRotation().getDegrees()<10.0
    && swerveDrive.getFieldVelocity().omegaRadiansPerSecond<0.5
    && systemState==SystemState.DRIVING_TO_POINT
    && getVelocity()<0.6;
    
  }
  
  public ChassisSpeeds getRobotRelativeSpeeds(){
    return swerveDrive.getRobotVelocity();
  }
  
  
  //I have no idea if this works
  public Pose2d getFieldRelativeIntakePose(Translation2d noteRobotRelative){
    // final double middleToIntake = Inches.of(16.0).in(Meters);
    final double middleToIntake = Inches.of(0).in(Meters);
    Translation2d noteIntakeRelative = noteRobotRelative.minus(new Translation2d(middleToIntake, 0.0));
    
    //technically off by 16 inches, but whatever.  
    Pose2d fieldRelativeNotePose = getPose().plus(new Transform2d(noteIntakeRelative, noteRobotRelative.getAngle()));
    odometryField.getObject("coral").setPose(fieldRelativeNotePose); // whatevvvaaa
    return fieldRelativeNotePose;
    
  }

  public void setMaxPIDSpeed(double max){
    this.maxVelocityForPID = max;
  }

  public void setMaxPIDOmega(double max){
    this.maxRotationalVelocityForPID = max;
  }
  
  // public Command pidToPoseCommand(Pose2d pose){
  //   return new InstantCommand(()->SetWantedState(WantedState.DRIVE_TO_POINT, pose),  this);
  // }

  public Command driveAwayFromReef(CommandXboxController driver){
    return new RunCommand(()->setWantedState(
        SwerveSubsystem.WantedState.ASSISTED_TELEOP_DRIVE, 
        ()->{
          Rotation2d poseRotation = FieldNavigation.getNearestReef(getPose()).getRotation();
          double xAssist = 0.5*Math.cos(poseRotation.getRadians());
          xAssist *= isRed.getAsBoolean() ? -1 : 1;

          return xAssist;
        },
        ()->{
          Rotation2d poseRotation = FieldNavigation.getNearestReef(getPose()).getRotation();
          double yAssist = 0.5*Math.sin(poseRotation.getRadians());
          yAssist *= isRed.getAsBoolean() ? -1 : 1;
         
          return yAssist;
        },
        ()->-driver.getLeftY(),
        ()->-driver.getLeftX(),
        ()->-driver.getRightX(),
        false
      ), this)

      .until(()->!FieldNavigation.getTooCloseToTag(getPose()));
  }
  
  public Command intakeAssist(VisionSubsystem visionSubsystem, CommandXboxController driver){
    return new RunCommand(()->setWantedState(
          SwerveSubsystem.WantedState.ASSISTED_TELEOP_DRIVE, 
          ()->0,
          ()->{
            Optional<Translation2d> optional = visionSubsystem.getObjectTranslationRelative();
            if(optional.isEmpty()){return 0.0;}
            Translation2d translation = optional.get();
            if(Math.abs(translation.getY())<Inches.of(1).in(Meters)){return 0.0;}
            ChassisSpeeds robotRelative = getRobotRelativeSpeeds();
            
            double xSpeed = robotRelative.vxMetersPerSecond;
            if(xSpeed<0.0){return 0.0;}

            double yAssist = MathUtil.clamp(translation.getY(), -2.0, 2.0);
            yAssist *= xSpeed;

            SmartDashboard.putNumber("Swervedrive/intaking/yAssist", yAssist);

            if(Math.abs(yAssist)<0.05){return 0.0;}
            
            return yAssist;
          },
          ()->-driver.getLeftY(),
          ()->-driver.getLeftX(),
          ()->-driver.getRightX(),
          true
        ), this)

        .finallyDo((e)->setWantedState(
          SwerveSubsystem.WantedState.TELEOP_DRIVE, 
          ()->-driver.getLeftY(),
          ()->-driver.getLeftX(),
          ()->-driver.getRightX()
        ));
  }

  public Command waitUntil(BooleanSupplier... suppliers){
    BooleanSupplier combined = ()->{
      for (BooleanSupplier supplier : suppliers){
        if (!supplier.getAsBoolean()){
          return false;
        }
      }
      return true;
    };

    return new WaitCommand(0.05).andThen(new RunCommand(()->{}).until(combined));
  }


  public Command getToScoringPose(Supplier<Pose2d> offsetPose, Supplier<Pose2d> scoringPose, BooleanSupplier... pauses){
    return new SequentialCommandGroup(
      new ConditionalCommand(
        new InstantCommand(()->SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, offsetPose.get()), this)
        .andThen(waitUntil(pauses)
          .withTimeout(3.0)),
        new InstantCommand(),
        ()->{
          Pose2d delta = scoringPose.get().relativeTo(getPose());
          return Math.abs(delta.getRotation().getDegrees())<20.0  && Math.abs(delta.getTranslation().getAngle().getDegrees())<30.0;
        }
      ),
      new InstantCommand(()->SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, scoringPose.get()), this)
    );
  }
}
