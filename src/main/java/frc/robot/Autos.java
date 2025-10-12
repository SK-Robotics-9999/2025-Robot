// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.concurrent.CompletableFuture;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SuctionSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.CurrentSuperState;
import frc.robot.subsystems.SuperStructure.WantedSuperState;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;


/** Add your docs here. */
//the indents r just cooked, and thats ok. we only have to deal with it this one time.
public class Autos {
  /** The future value we care about */
  private CompletableFuture<Command> selectedAutoFuture = CompletableFuture.supplyAsync(()->new InstantCommand());

  /** Alliance color; Must be properly set before we can build autos, so we do that */
  private Alliance alliance = DriverStation.Alliance.Red;

  SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<>();

  /** The value last selected on the chooser.  */
  private Supplier<Command> selectedAuto = ()->new InstantCommand();

  /** Track if we need to re-build the auto due to having ran it and done a field reset */
  private boolean requireRebuild=true;

  //Pass in subsystems so we can access them easily
  SwerveSubsystem swerveSubsystem;
  SuperStructure superStructure;
  ElevatorSubsystem elevatorSubsystem;
  ArmSubsystem armSubsystem;
  SuctionSubsystem suctionSubsystem;
  IntakeSubsystem intakeSubsystem;
  VisionSubsystem visionSubsystem;

  RobotContainer robotContainer;


  public Autos(
      // SwerveSubsystem swerveSubsystem,
      // SuperStructure superStructure,
      // ElevatorSubsystem elevatorSubsystem,
      // ArmSubsystem armSubsystem,
      // SuctionSubsystem suctionSubsystem,
      // IntakeSubsystem intakeSubsystem,
      // VisionSubsystem visionSubsystem

      RobotContainer robotContainer

      // Vision vision
  ){
      this.swerveSubsystem = robotContainer.swerveSubsystem;
      this.superStructure = robotContainer.superStructure;
      this.elevatorSubsystem = robotContainer.elevatorSubsystem;
      this.armSubsystem = robotContainer.armSubsystem;
      this.suctionSubsystem = robotContainer.suctionSubsystem;
      this.intakeSubsystem = robotContainer.intakeSubsystem;
      this.visionSubsystem = robotContainer.visionSubsystem;

      this.robotContainer=robotContainer;

      // Add options to our chooser; This could be done manually if we wanted
      SmartDashboard.putData("AutoSelector/chooser",autoChooser);
      autoChooser.setDefaultOption("Select Auto",()->new InstantCommand());

      autoChooser.addOption("L4 Right Auto", this::L4RightAuto);
      autoChooser.addOption("L4 Left Auto", this::L4LeftAuto);
      autoChooser.addOption("LOOONGGG4 Right Auto", this::L4LongRightAuto);
      autoChooser.addOption("LOOONGGG4 Left Auto", this::L4LongLeftAuto);
      //INSERT TESTED AUTOS HERE; Drivers wil use these.
      //New L4 autos
      // autoChooser.addOption("L4 Basic Left Auto", this::leftL4CoralAuto);
      // autoChooser.addOption("L4 Basic Right Auto", this::rightL4CoralAuto);
      // autoChooser.addOption("L4 Basic Center Auto", this::centerL4CoralAuto);

      // autoChooser.addOption("Multicoral Right Auto", this::rightMultiCoralAuto);
      // autoChooser.addOption("Multicoral Left Auto", this::leftMultiCoralAuto);

      // autoChooser.addOption("Drive Forward Score", this::driveForwardScore);
      // autoChooser.addOption("Dead  Reckoning Center", this::deadReckoningRightBranch);
      
      // autoChooser.addOption("v TEST AUTOS v",()->new InstantCommand());
      // //test autos here
      // //autoChooser.addOption("testCenterAuto", this::basicCenterAutoTest);
      // //autoChooser.addOption("testLeftAuto", this::basicLeftAutoTest);
      //  autoChooser.addOption("L4 Multicoral Left Optimized", this::leftMultiCoralAutoOptimized);
      //  autoChooser.addOption("L4 Multicoral RIGHT Optimized", this::rightMultiCoralAutoOptimized);

      //PUT UNTESTED AUTOS HERE; Drivers should not select these
      // autoChooser.addOption("1MeterNoTurn", ()->swerve.followPath("1Meter"));
      // autoChooser.addOption("1MeterTurn", ()->swerve.followPath("1MeterTurn"));
      // autoChooser.addOption("LongSpline", ()->swerve.followPath("LongSpline"));

  }

  /** Should be checked in Robot.java::autonomousInit
   *  This will return the selected auto, and *will* block the
   *  main thread if necessary for it to complete. However, this should 
   *  almost never happen, but is safely handled internally if it does
   */
  public Command getAutonomousCommand(){
      this.requireRebuild = true;
      try{
            return selectedAutoFuture.get();
      }
      catch(Exception e){
          //If the auto cannot build, then we get an error and print it.
          System.err.println("Failed to build auto command ");
          System.err.println(e);
      }
      return new InstantCommand();
  }

  /**
   * The poll operation that watches for updates that affect autos.
   * This should be called in robot.java::DisabledPeriodic. 
   */
  void periodic(){
      //make sure we have our team color: It's necessary to guarantee this before building autos
      if(DriverStation.getAlliance().isEmpty()) return; 

      //Make sure either a default exists or a option was selected
      if(autoChooser.getSelected() == null) return;

      //Just show the status of our current build on the dashboard
      var ready = selectedAutoFuture.isDone() && requireRebuild==false;
      var gyroReady = swerveSubsystem.isGyroConnected();
      SmartDashboard.putBoolean("AutoSelector/ready",ready);

      //If we haven't changed auto or alliance, nothing to do
      if(selectedAuto == autoChooser.getSelected() 
          && alliance == DriverStation.getAlliance().get()
          && requireRebuild==false
      ){
          return;
      }

      if( ! selectedAutoFuture.isDone() ){
        //Ideally we want to cancel the future, but it doesn't work properly
        //work around it by waiting until it's done first, then swap out the 
        //future
        return; 
      }

      //Save which one we're running now
      selectedAuto = autoChooser.getSelected();
      alliance = DriverStation.getAlliance().get();
      
      //No longer needs a rebuild
      requireRebuild=false;
      
      //Run the function that builds the desired auto in the background
      selectedAutoFuture = CompletableFuture.supplyAsync(selectedAuto);
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

///////////////////////////////////////////////////
/// Helpful snippets ////////////////////////////////
///////////////////////////////////////////////////

public Command getBackupSequence(Supplier<Pose2d> backupPose){
  return new SequentialCommandGroup(
    new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, backupPose.get()), swerveSubsystem),
    waitUntil(()->swerveSubsystem.getOnTarget(backupPose.get(), Inches.of(8).in(Meters), 5.0, 2.0))
  )
  .withTimeout(3.0)
  ;
}

public Command getIntakeSequence(Supplier<Pose2d> intakePose, Supplier<Rotation2d> panRotation){
  return new SequentialCommandGroup(
    new InstantCommand(()->swerveSubsystem.setMaxPIDSpeed(3.0)),

    new SequentialCommandGroup(
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.CORAL_GROUND_INTAKE),superStructure),
      waitUntil(()->visionSubsystem.getObjectFieldRelativePose(swerveSubsystem.getPose()).isPresent()).withTimeout(0.5),

      //if we saw a coral, then we dont do anything and skip. if we didnt see a coral, then we pan slowly.
      new InstantCommand(()->{
        if(visionSubsystem.getLastSeenObjectPose().isEmpty()){
          swerveSubsystem.setMaxPIDOmega(Math.PI/3.0);
          swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, swerveSubsystem.getPose().transformBy(new Transform2d(new Translation2d(), panRotation.get())));
        }
      }),
      new RunCommand(()->visionSubsystem.getObjectFieldRelativePose(swerveSubsystem.getPose()))
      .until(()->visionSubsystem.getLastSeenObjectPose().isPresent())
      .withTimeout(2.0),

      new InstantCommand(()->swerveSubsystem.setMaxPIDOmega(SwerveSubsystem.MAXOMEGA)),

      //if weve seen one in the last 2 seconds, drive to that pose correctly. otherwise, drive to predefined pose and hope we get one
      new ConditionalCommand(
        new SequentialCommandGroup(
          //Set a rotation target at the 30% mark
          new InstantCommand(()->{
            Pose2d interpolated = swerveSubsystem.getPose().interpolate(visionSubsystem.getLastSeenObjectPose().get(), 0.3);
            swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, new Pose2d(interpolated.getX(), interpolated.getY(), visionSubsystem.getLastSeenObjectPose().get().getRotation()));
          }),
          waitUntil(()->swerveSubsystem.getOnTarget(Inches.of(8).in(Meters), 5, 2.0)),
          new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, visionSubsystem.getLastSeenObjectPose().get())),
          waitUntil(swerveSubsystem::getOnTarget)
        ),
        //our "luck" pose if we never saw any coral
        new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, intakePose.get()))
        .andThen(waitUntil(swerveSubsystem::getOnTarget)),
        ()->visionSubsystem.getLastSeenObjectPose().isPresent()
      )
    )
    .withTimeout(5.0)
    .until(()->FieldNavigation.getTooCloseToSource(swerveSubsystem.getPose()) || intakeSubsystem.getBeamBreakTrigger().getAsBoolean()),
    new InstantCommand(()->{
      swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.STOP);
      swerveSubsystem.setMaxPIDSpeed(SwerveSubsystem.MAXVELOCITYFORPID);
      visionSubsystem.clearLastSeenObjectPose();
    }),
    //if we got too close, we want to give a few seconds, in case we're still intaking
    new ConditionalCommand(new WaitCommand(0.5), new InstantCommand(), ()->FieldNavigation.gotTooClose)
  );
}

public Command getPlaceSequence(Supplier<Pose2d> offsetPose, Supplier <Pose2d> scoringPose){
  return new ParallelCommandGroup(
    new SequentialCommandGroup(
      new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, offsetPose.get()), swerveSubsystem),
      waitUntil(swerveSubsystem::getCloseEnough, ()->superStructure.getCurrentSuperState()==CurrentSuperState.MOVE_TO_L4, ()->armSubsystem.getOnTarget(ArmConstants.moveL4, 30), ()->elevatorSubsystem.getOnTarget(ElevatorConstants.moveL4,5)).withTimeout(3.0),
      new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, scoringPose.get()), swerveSubsystem),
      waitUntil(swerveSubsystem::getOnScoringPose)
    ),

    new SequentialCommandGroup(
      robotContainer.pickupCoralSequence(),
      robotContainer.moveAndPlace(WantedSuperState.MOVE_TO_L4, WantedSuperState.PLACE_L4)
    )
  )
  .withTimeout(5.0)
  ;
}

public Command getCycleSequence(Supplier<Pose2d> backupPose, Supplier<Pose2d> intakePose, Supplier<Rotation2d> panRotation, Supplier<Pose2d> offsetPose, Supplier<Pose2d> scoringPose){
  // Supplier<Rotation2d> panRotation = ()->{
  //   double radiansRotation = Math.toRadians(0)
  // }
  return new SequentialCommandGroup(
    getBackupSequence(backupPose),
    getIntakeSequence(intakePose, panRotation),
    getPlaceSequence(offsetPose, scoringPose)
  );
}

public Command getStartAuto(Supplier<Pose2d> offsetPose, Supplier<Pose2d> scorePose){
  return Commands.sequence(
      new InstantCommand(()->robotContainer.automationEnabled=true),
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.START_AUTO)),
      waitUntil(suctionSubsystem::getCoralSuctionGood),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, offsetPose.get()), swerveSubsystem),
          waitUntil(swerveSubsystem::getCloseEnough),
          new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, scorePose.get()), swerveSubsystem),
          waitUntil(swerveSubsystem::getOnTarget)
        ),
          new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.MOVE_TO_L4))
      ),
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PLACE_L4)),
      waitUntil(elevatorSubsystem::getOnTarget, armSubsystem::getOnTarget).withTimeout(1.5)
    );
}

  public Command L4RightAuto(){
    return Commands.sequence(
      new InstantCommand(()->System.out.println("L4LeftAuto")),
      getStartAuto(()->FieldNavigation.getOffsetCoralRight(swerveSubsystem.getPose()), ()->FieldNavigation.getCoralRight(swerveSubsystem.getPose())),
      new WaitCommand(1.5), //technically should be elevator on target, etc.
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.HOME))
    );
  }
  
  public Command L4LeftAuto(){
    return Commands.sequence(
      new InstantCommand(()->System.out.println("L4LeftAuto")),
      //the right to the vision of the apriltag is left when facing at the apriltag.
      getStartAuto(()->FieldNavigation.getOffsetCoralLeft(swerveSubsystem.getPose()), ()->FieldNavigation.getCoralLeft(swerveSubsystem.getPose())),
      new WaitCommand(1.5), //technically should be elevator on target, etc.
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.HOME))
    );
  }


  public Command L4LongRightAuto(){
    return Commands.sequence(
      new InstantCommand(()->System.out.println("L4RightAutoLong")),
      getStartAuto(()->FieldNavigation.getOffsetCoralTag(false, 22, 9), ()->FieldNavigation.getCoralTag(false, 22, 9)),
      getCycleSequence(()->FieldNavigation.getCustomBackup(swerveSubsystem.getPose()), ()->FieldNavigation.getCustomSource(swerveSubsystem.getPose()), ()->new Rotation2d(Math.toRadians(90)), ()->FieldNavigation.getOffsetCoralTag(false, 17, 8), ()->FieldNavigation.getCoralTag(false, 17, 8)),
      getCycleSequence(()->FieldNavigation.getOffsetCoralRight(swerveSubsystem.getPose()), ()->FieldNavigation.getCoralSource(swerveSubsystem.getPose()), ()->new Rotation2d(Math.toRadians(-60)),()->FieldNavigation.getOffsetCoralTag(true, 17, 8), ()->FieldNavigation.getCoralTag(true, 17, 8)),
      getCycleSequence(()->FieldNavigation.getOffsetCoralRight(swerveSubsystem.getPose()), ()->FieldNavigation.getCoralSource(swerveSubsystem.getPose()), ()->new Rotation2d(Math.toRadians(-60)),()->FieldNavigation.getOffsetCoralTag(true, 17, 8), ()->FieldNavigation.getCoralTag(true, 17, 8))
    );
  }

  public Command L4LongLeftAuto(){
    return Commands.sequence(
      new InstantCommand(()->System.out.println("L4LeftAutoLong")),
      getStartAuto(()->FieldNavigation.getOffsetCoralTag(true, 20, 11), ()->FieldNavigation.getCoralTag(true, 20, 11)),
      getCycleSequence(()->FieldNavigation.getCustomBackup(swerveSubsystem.getPose()), ()->FieldNavigation.getCustomSource(swerveSubsystem.getPose()), ()->new Rotation2d(Math.toRadians(-90)), ()->FieldNavigation.getOffsetCoralTag(true, 19, 6), ()->FieldNavigation.getCoralTag(true, 19, 6)),
      getCycleSequence(()->FieldNavigation.getOffsetCoralLeft(swerveSubsystem.getPose()), ()->FieldNavigation.getCoralSource(swerveSubsystem.getPose()), ()->new Rotation2d(Math.toRadians(60)), ()->FieldNavigation.getOffsetCoralTag(false, 19, 6), ()->FieldNavigation.getCoralTag(false, 19, 6)),
      getCycleSequence(()->FieldNavigation.getOffsetCoralRight(swerveSubsystem.getPose()), ()->FieldNavigation.getCoralSource(swerveSubsystem.getPose()), ()->new Rotation2d(Math.toRadians(-60)),()->FieldNavigation.getOffsetCoralTag(true, 17, 8), ()->FieldNavigation.getCoralTag(true, 17, 8))
    );
  }


}