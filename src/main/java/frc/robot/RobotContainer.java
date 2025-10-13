// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SuctionSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.CurrentSuperState;
import frc.robot.subsystems.SuperStructure.WantedSuperState;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem.WantedState;
import frc.robot.subsystems.LEDSubsystem.BlinkinPattern;
import frc.robot.Autos;

/** 
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  BooleanSupplier isRed = () -> {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  };

  private boolean hasCoral=false;//TODO: technically true at the start of auto, will fix after

  private boolean coralMode=true;
  // public boolean automationEnabled=true;

  // The robot's subsystems and commands are defined here...
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  ArmSubsystem armSubsystem = new ArmSubsystem(()->!coralMode);
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  SuctionSubsystem suctionSubsystem = new SuctionSubsystem();
  SuperStructure superStructure = new SuperStructure( intakeSubsystem, elevatorSubsystem, armSubsystem, suctionSubsystem);

  LEDSubsystem ledSubsystem = new LEDSubsystem(()->!coralMode);

  VisionSubsystem visionSubsystem = new VisionSubsystem(swerveSubsystem, superStructure);
  // QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem();

  public final Autos autos = new Autos(this);
  
  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController vitaliy = new CommandXboxController(1);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

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

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    enableTeleopDriving();

    // driver.start().onTrue(new InstantCommand(()->swerveSubsystem.resetGyro()));
    driver.povRight().onTrue(new InstantCommand(()->coralMode=!coralMode));

    //Rumble when breakbeam activated
    intakeSubsystem.getBeamBreakTrigger()
    .onTrue(new StartEndCommand(()->driver.setRumble(RumbleType.kBothRumble, 0.3), ()->driver.setRumble(RumbleType.kBothRumble, 0.0)).withTimeout(0.5))
    .onTrue(new InstantCommand(()->ledSubsystem.blink(BlinkinPattern.GREEN, 1.5)))
    ;
    
    new Trigger(()->!coralMode && suctionSubsystem.getAlgaeSuctionGood())
    .onTrue(new StartEndCommand(()->driver.setRumble(RumbleType.kBothRumble, 0.3), ()->driver.setRumble(RumbleType.kBothRumble, 0.0)).withTimeout(0.5))
    .onTrue(new InstantCommand(()->ledSubsystem.blink(BlinkinPattern.BLUE, 1.5)))
    ;

    //intake
    driver.leftTrigger()
    .whileTrue(
      swerveSubsystem.driveAwayFromReef(driver)
        .withTimeout(5)
      .andThen(new ConditionalCommand(
        swerveSubsystem.intakeAssist(visionSubsystem, driver),
        new InstantCommand(),
        ()->coralMode
      ))
      .finallyDo((e)->enableTeleopDriving())
    )
    .onTrue(new ConditionalCommand(
      new SequentialCommandGroup(
        waitUntil(()->!FieldNavigation.getTooCloseToTag(swerveSubsystem.getPose())).withTimeout(2),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.CORAL_GROUND_INTAKE),superStructure),
        waitUntil(intakeSubsystem.getBeamBreakTrigger(),armSubsystem::getOnTarget, elevatorSubsystem::getOnTarget),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_RECEIVE),superStructure),
        waitUntil(armSubsystem::getOnTarget, elevatorSubsystem::getOnTarget),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.CORAL_GROUND_RECEIVE),superStructure),
        waitUntil(suctionSubsystem::getCoralSuctionGood).withTimeout(2.0),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_PLACE),superStructure)
      ),
      pulloutAlgae(),
      ()->coralMode
    ));

      //Move To l1
    driver.a().onTrue(
      pickupCoralSequence()
      .until(()->getHasCoral())
      .andThen(moveAndPlace(WantedSuperState.MOVE_TO_L1, WantedSuperState.PLACE_L1))
    );
      
    //Move To l2
    driver.b().onTrue(new ConditionalCommand(
      pickupCoralSequence()
      .until(()->getHasCoral())
      .andThen(moveAndPlace(WantedSuperState.MOVE_TO_L2, WantedSuperState.PLACE_L2)),
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.ALGAE_INTAKE_L2), superStructure),
      ()->coralMode
    )); 

    //Move To l3
    driver.x().onTrue(new ConditionalCommand(
      pickupCoralSequence()
      .until(()->getHasCoral())
      .andThen(moveAndPlace(WantedSuperState.MOVE_TO_L3, WantedSuperState.PLACE_L3)),
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.ALGAE_INTAKE_L3),superStructure),
      ()->coralMode
      ));
      
    //Move to L4
    driver.y().onTrue(new ConditionalCommand(
      pickupCoralSequence()
      .until(()->getHasCoral())
      .andThen(moveAndPlace(WantedSuperState.MOVE_TO_L4, WantedSuperState.PLACE_L4)),
      new SequentialCommandGroup(
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.MOVE_TO_BARGE),superStructure),
        waitUntil(swerveSubsystem::getCloseEnough),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.RELEASE_ALGAE_INTAKE), superStructure),
        new InstantCommand(()->ledSubsystem.blink(BlinkinPattern.BLUE_VIOLET, 1.5))
      
      ),
      ()->getCoralMode()
    ))
    .whileTrue(
      new ConditionalCommand(
        new InstantCommand(), 
        new SequentialCommandGroup(
          new InstantCommand(()->swerveSubsystem.setMaxPIDSpeed(1.0)),
          new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, FieldNavigation.getBargeScorePose(swerveSubsystem.getPose()))),
          waitUntil(swerveSubsystem::getCloseEnough)
        )
        .finallyDo((e)->{
          swerveSubsystem.setMaxPIDSpeed(SwerveSubsystem.MAXVELOCITYFORPID);
          enableTeleopDriving();
        }), 
        ()->getCoralMode()
      )
    )
    ;
          
    driver.rightTrigger().onTrue(new ConditionalCommand(
      new InstantCommand(()->{
        switch(superStructure.getCurrentSuperState()){
          case MOVE_TO_L1:
            superStructure.SetWantedState(WantedSuperState.PLACE_L1);
            break;
          case MOVE_TO_L2:
            superStructure.SetWantedState(WantedSuperState.PLACE_L2);
            break;
          case MOVE_TO_L3:
            superStructure.SetWantedState(WantedSuperState.PLACE_L3);
            break;
          case MOVE_TO_L4:
            superStructure.SetWantedState(WantedSuperState.PLACE_L4);
            break;
        }
      },superStructure)
      .alongWith(new InstantCommand(()->hasCoral=false)),
      new InstantCommand(()->{
        superStructure.SetWantedState(WantedSuperState.RELEASE_ALGAE_INTAKE);
      }, superStructure),
      ()->coralMode
    ));

    driver.leftBumper().whileTrue(new SequentialCommandGroup(
        new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, FieldNavigation.getOffsetCoralRight(swerveSubsystem.getPose())), swerveSubsystem),
        waitUntil(swerveSubsystem::getCloseEnough).withTimeout(3.0),
        new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, FieldNavigation.getCoralRight(swerveSubsystem.getPose())), swerveSubsystem),
        waitUntil(()->false)
      )
      .until(()->!superStructure.getIsAtReefState() && FieldNavigation.getTooCloseToTag(swerveSubsystem.getPose()))
      .finallyDo((e)->enableTeleopDriving())
    );
    
    driver.rightBumper().whileTrue(new SequentialCommandGroup(
        //the right to the vision of the apriltag is left when facing at the apriltag.
        new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, FieldNavigation.getOffsetCoralLeft(swerveSubsystem.getPose())), swerveSubsystem),
        waitUntil(swerveSubsystem::getCloseEnough).withTimeout(3.0),
        new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, FieldNavigation.getCoralLeft(swerveSubsystem.getPose())), swerveSubsystem),
        waitUntil(()->false)
      )
      .until(()->!superStructure.getIsAtReefState() && FieldNavigation.getTooCloseToTag(swerveSubsystem.getPose()))
      .finallyDo((e)->enableTeleopDriving())
    );

    driver.back().whileTrue(new SequentialCommandGroup(
        new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, FieldNavigation.getOffsetCoralMid(swerveSubsystem.getPose())), swerveSubsystem),
        waitUntil(swerveSubsystem::getCloseEnough),
        new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, FieldNavigation.getCoralMid(swerveSubsystem.getPose())), swerveSubsystem),
        new ConditionalCommand(
          new SequentialCommandGroup(
            waitUntil(suctionSubsystem::getAlgaeSuctionGood),
            // new InstantCommand(()->ledSubsystem.blink(BlinkinPattern.BLUE_VIOLET, 1.5)),
            swerveSubsystem.driveAwayFromReef(driver),
            waitUntil(()->!FieldNavigation.getTooCloseToTag(swerveSubsystem.getPose())),
            pulloutAlgae(),
            new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.STOW_ALGAE), superStructure)
          ),
          new RunCommand(()->{}),
          ()->!coralMode
        )
      )
      .until(()->!superStructure.getIsAtReefState() && FieldNavigation.getTooCloseToTag(swerveSubsystem.getPose()))
      .finallyDo((e)->enableTeleopDriving())
    );

    driver.povUp().onTrue(
      new SequentialCommandGroup(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.HOME),superStructure),
        new InstantCommand(()->enableTeleopDriving()),
        new WaitCommand(5)
      )
      );

    driver.povDown().onTrue(
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.EJECT))
    );

    // driver.povLeft().onTrue(
    //   new InstantCommand(()->automationEnabled=!automationEnabled)
    // );

    // vitaliy.start().whileTrue(
    //   new SequentialCommandGroup(
    //     new InstantCommand(()->intakeSubsystem.SetWantedState(WantedState.TRUE_HOME)),
    //     new WaitCommand(0.5),
    //     waitUntil(intakeSubsystem::intakeSlowedDown),
    //     new InstantCommand(()->intakeSubsystem.zeroOnDown())
    //   )
    //   .finallyDo(()->intakeSubsystem.SetWantedState(WantedState.HOME))
    // );
  }

  public Command pickupCoralSequence(){
    return new SequentialCommandGroup(
      new ConditionalCommand(
        new SequentialCommandGroup(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_RECEIVE),superStructure),
        waitUntil(armSubsystem::getOnTarget, elevatorSubsystem::getOnTarget),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.CORAL_GROUND_RECEIVE),superStructure),
        waitUntil(suctionSubsystem::getCoralSuctionGood).withTimeout(2.0),
        new RepeatCommand(
          new SequentialCommandGroup(
            new InstantCommand(()->suctionSubsystem.SetWantedState(frc.robot.subsystems.SuctionSubsystem.WantedState.RELEASE)),
            new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_RECEIVE),superStructure),
            new WaitCommand(0.2),
            new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.CORAL_GROUND_RECEIVE),superStructure),
            waitUntil(suctionSubsystem::getCoralSuctionGood).withTimeout(2.0)
          )
        )
      )
      .until(suctionSubsystem::getCoralSuctionGood),
      new InstantCommand(),
      ()->superStructure.getCurrentSuperState()==CurrentSuperState.PREPARE_TO_PLACE
      ),
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_PLACE),superStructure),
      waitUntil(elevatorSubsystem::getOnTarget, armSubsystem::getOnTarget),
      new InstantCommand(()->hasCoral=true)
    );
  }

  private Command pulloutAlgae(){
    return new InstantCommand(()->{
      switch(superStructure.getCurrentSuperState()){
        case ALGAE_INTAKE_L2:
          superStructure.SetWantedState(WantedSuperState.PULLOUT_ALGAE_INTAKE_L2);
          break;
        case ALGAE_INTAKE_L3:
          superStructure.SetWantedState(WantedSuperState.PULLOUT_ALGAE_INTAKE_L3);
          break;
      }
    }, superStructure);
  }

  public Command moveAndPlace(WantedSuperState moveTo, WantedSuperState placeAt){
    return new SequentialCommandGroup(
      new InstantCommand(()->superStructure.SetWantedState(moveTo),superStructure),
      waitUntil(elevatorSubsystem::getOnTarget, armSubsystem::getOnTarget, swerveSubsystem::getOnScoringPose),
      new InstantCommand(()->superStructure.SetWantedState(placeAt),superStructure),
      waitUntil(elevatorSubsystem::getOnTarget, armSubsystem::getOnTarget).withTimeout(0.5),
      new InstantCommand(()->ledSubsystem.blink(BlinkinPattern.RED_ORANGE, 2)),
      new InstantCommand(()->hasCoral=false)
    );
  }

  public void enableTeleopDriving(){
    swerveSubsystem.setWantedState(
      SwerveSubsystem.WantedState.TELEOP_DRIVE, 
      ()->-driver.getLeftY(),
      ()->-driver.getLeftX(),
      ()->-driver.getRightX()
    );
  }

  public boolean getHasCoral(){
    hasCoral = hasCoral && coralMode && suctionSubsystem.getCoralSuctionApplied();
    return hasCoral;
  }

  public boolean getCoralMode(){
    return coralMode;
  }

  // public boolean getAutomationEnabled(){
  //   return automationEnabled;
  // }
}
