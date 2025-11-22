// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

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

  
  public boolean coralMode=true;
  public boolean hasCoral=false;//TODO: technically true at the start of auto, will fix after
  public boolean hasAlgae = false;
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

  CommandXboxController funniDriver = new CommandXboxController(2);
  CommandXboxController funniOperator = new CommandXboxController(3);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    enableTeleopDriving(); //NECESSARY!!!!

    configureBindings();
    configureFunniButtons();
  }

  /**
   * Will always wait 50 milliseconds minimum
   */
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
   * Does NOT wait 50 milliseconds minimum
   */
  public Command trueWaitUntil(BooleanSupplier... suppliers){
    BooleanSupplier combined = ()->{
      for (BooleanSupplier supplier : suppliers){
        if (!supplier.getAsBoolean()){
          return false;
        }
      }
      return true;
    };

    return new RunCommand(()->{}).until(combined);
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
    // driver.start().onTrue(new InstantCommand(()->swerveSubsystem.resetGyro()));
    driver.povRight().debounce(0.025).onTrue(new InstantCommand(()->coralMode=!coralMode));

    //Rumble when breakbeam activated
    intakeSubsystem.getBeamBreakTrigger()
    .onTrue(new StartEndCommand(()->driver.setRumble(RumbleType.kBothRumble, 0.3), ()->driver.setRumble(RumbleType.kBothRumble, 0.0)).withTimeout(0.5))
    .onTrue(new InstantCommand(()->ledSubsystem.blink(BlinkinPattern.GREEN, 0.8)))
    ;
    
    new Trigger(()->!coralMode && suctionSubsystem.getAlgaeSuctionGood())
    .onTrue(new StartEndCommand(()->driver.setRumble(RumbleType.kBothRumble, 0.3), ()->driver.setRumble(RumbleType.kBothRumble, 0.0)).withTimeout(0.5))
    .onTrue(new InstantCommand(()->ledSubsystem.blink(BlinkinPattern.BLUE, 0.8)))
    ;

    new Trigger(swerveSubsystem::getOnTarget)
    .onTrue(new InstantCommand(()->ledSubsystem.blink(BlinkinPattern.HOT_PINK, 0.8)))
    ;

    //intake
    driver.leftTrigger()
    .whileTrue(
      swerveSubsystem.driveAwayFromReef(driver).withTimeout(5)
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
        new ConditionalCommand(
          new SequentialCommandGroup(
            new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.CORAL_GROUND_INTAKE_WITH_ALGAE),superStructure),
            trueWaitUntil(intakeSubsystem.getBeamBreakTrigger()),
            new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.STOW_ALGAE), superStructure)
          ),
          new SequentialCommandGroup(
            new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.CORAL_GROUND_INTAKE),superStructure),
            trueWaitUntil(intakeSubsystem.getBeamBreakTrigger()),
            pickupCoralSequence().until(this::getHasCoral),
            new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_PLACE))
          ),
          ()->getHasAlgae()//don't need to repoll result
        )
      ),

      pulloutAlgae(),
      ()->coralMode || getHasAlgae()
    ));

      //Move To l1
    driver.a()
    .and(()->getCoralMode()&&getHasCoral())
    .onTrue(
      new SequentialCommandGroup(
      moveAndPlace(WantedSuperState.MOVE_TO_L1, WantedSuperState.PLACE_L1)
        // new WaitCommand(0.2), //do we need to wait briefly?
        // new StartEndCommand(()->driver.setRumble(RumbleType.kBothRumble, 0.5), ()->driver.setRumble(RumbleType.kBothRumble, 0.0)).withTimeout(0.5)
      )
    );
    driver.a()
    .and(()->!getCoralMode())
    .onTrue(
      new SequentialCommandGroup(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.ALGAE_GROUND_INTAKE),superStructure),
        waitUntil(suctionSubsystem::getAlgaeSuctionGood),
        new InstantCommand(()->hasAlgae=true)
      )
    ); 
      
    //Move To l2
    driver.b()
    .and(()->getCoralMode()&&getHasCoral())
    .onTrue(
      moveAndPlace(WantedSuperState.MOVE_TO_L2, WantedSuperState.PLACE_L2)
    ); 
    driver.b()
    .and(()->!getCoralMode())
    .onTrue(
      new SequentialCommandGroup(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.ALGAE_INTAKE_L2),superStructure),
        waitUntil(suctionSubsystem::getAlgaeSuctionGood),
        new InstantCommand(()->hasAlgae=true)
      )
    ); 

    //Move To l3
    driver.x()
    .and(()->getCoralMode()&&getHasCoral())
    .onTrue(
      moveAndPlace(WantedSuperState.MOVE_TO_L3, WantedSuperState.PLACE_L3)
    )
    ;
    driver.x()
    .and(()->!getCoralMode())
    .onTrue(
      new SequentialCommandGroup(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.ALGAE_INTAKE_L3),superStructure),
        waitUntil(suctionSubsystem::getAlgaeSuctionGood),
        new InstantCommand(()->hasAlgae=true)
      )
    );

    //HOPEFULLY WORKS
    driver.povLeft()
    .onTrue(new InstantCommand(()->swerveSubsystem.resetGyro())
    );

    //and(has Coral) on true then we will run the command, so does not requirement conflict until the intake has satisfied
      
    //Move to L4
    driver.y()
    .whileTrue(
      new ConditionalCommand(
        new InstantCommand(), 
        new SequentialCommandGroup(
          new InstantCommand(()->swerveSubsystem.setMaxPIDSpeed(3.0)),
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
    //in theory, get has coral changes the state. by putting it in trigger, it will reupdate every loop. idk if thats desirable
    .and(()->getCoralMode()&&getHasCoral())
    .onTrue(moveAndPlace(WantedSuperState.MOVE_TO_L4, WantedSuperState.PLACE_L4))
    ;
    driver.y()
    .and(()->!getCoralMode())
    .onTrue(
      new SequentialCommandGroup(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.MOVE_TO_BARGE),superStructure),
        waitUntil(swerveSubsystem::getCloseEnough),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.SCORE_ALGAE_BARGE), superStructure),
        new InstantCommand(()->ledSubsystem.blink(BlinkinPattern.BLUE_VIOLET, 1.5)),
        new InstantCommand(()->coralMode=!coralMode),
        new InstantCommand(()->hasAlgae=false)
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
        superStructure.SetWantedState(WantedSuperState.SCORE_ALGAE_BARGE);
        hasAlgae=false;
        coralMode =! coralMode;
      }, superStructure),
      ()->coralMode
    ));

    driver.leftBumper().whileTrue(new SequentialCommandGroup(
        // new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, FieldNavigation.getOffsetCoralRight(swerveSubsystem.getPose())), swerveSubsystem),
        // waitUntil(swerveSubsystem::getCloseEnough).withTimeout(3.0),
        new InstantCommand(()->{
          Pose2d rightAlign = FieldNavigation.getCoralRight(swerveSubsystem.getPose());
          if(superStructure.getCurrentSuperState()==CurrentSuperState.MOVE_TO_L1 || superStructure.getCurrentSuperState()==CurrentSuperState.PLACE_L1){
            rightAlign = rightAlign.transformBy(new Transform2d(Inches.of(-2).in(Meters),Inches.of(-6).in(Meters),new Rotation2d()));
          }
          swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, rightAlign);
        }, swerveSubsystem),
        waitUntil(()->false)
      )
      .until(()->!superStructure.getIsAtReefState() && FieldNavigation.getTooCloseToTag(swerveSubsystem.getPose()))
      .finallyDo((e)->enableTeleopDriving())
    );
    
    driver.rightBumper().whileTrue(new SequentialCommandGroup(
        // new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, FieldNavigation.getOffsetCoralLeft(swerveSubsystem.getPose())), swerveSubsystem),
        // waitUntil(swerveSubsystem::getCloseEnough).withTimeout(3.0),
        new InstantCommand(()->{
          Pose2d leftAlign = FieldNavigation.getCoralLeft(swerveSubsystem.getPose());
          if(superStructure.getCurrentSuperState()==CurrentSuperState.MOVE_TO_L1 || superStructure.getCurrentSuperState()==CurrentSuperState.PLACE_L1){
            leftAlign = leftAlign.transformBy(new Transform2d(Inches.of(-2).in(Meters),Inches.of(6).in(Meters),new Rotation2d()));
          }
          swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, leftAlign);
        }, swerveSubsystem),
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
            new InstantCommand(()->hasAlgae=true),
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
      new ConditionalCommand(
        new SequentialCommandGroup(
          new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.HOME),superStructure),
          new InstantCommand(()->enableTeleopDriving()),
          new WaitCommand(5)
      ), 
        new SequentialCommandGroup(
          new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.STOW_ALGAE),superStructure),
          new InstantCommand(()->enableTeleopDriving()),
          new WaitCommand(5)
      ), 
      ()->!getHasAlgae())
      );

    driver.povDown().onTrue(
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.EJECT), superStructure)
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

  //TODO: make all commands turn on either coral or algae mode immediately
  public void configureFunniButtons(){
    funniDriver.start().onTrue(
      new InstantCommand(()->swerveSubsystem.resetGyro())
    );

    //a - L3 algae
    funniOperator.button(1).onTrue(
      new SequentialCommandGroup(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.ALGAE_INTAKE_L3), superStructure),
        waitUntil(suctionSubsystem::getAlgaeSuctionGood),
        new InstantCommand(()->hasAlgae=true),
        pulloutAlgae()
      )
    );

    //back - L2 algae
    funniOperator.button(7).onTrue(
      new SequentialCommandGroup(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.ALGAE_INTAKE_L3), superStructure),
        waitUntil(suctionSubsystem::getAlgaeSuctionGood),
        new InstantCommand(()->hasAlgae=true),
        pulloutAlgae()
      )
    );

    //TODO: Make sure this works
    //idk - algae eject
    funniOperator.button(13).onTrue(
      new InstantCommand(()->suctionSubsystem.SetWantedState(frc.robot.subsystems.SuctionSubsystem.WantedState.RELEASE), superStructure)
    );

    //TODO: Make sure goes back to stow correctly
    //b - algae intake - ground
    funniOperator.button(2).onTrue(
      new SequentialCommandGroup(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.ALGAE_GROUND_INTAKE),superStructure),
        waitUntil(suctionSubsystem::getAlgaeSuctionGood),
        new InstantCommand(()->hasAlgae=true),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.STOW_ALGAE), superStructure)
      )
    );

    //TODO: Check the onTrue onFalse
    //start - algae net
    funniOperator.button(8).onTrue(
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.MOVE_TO_BARGE), superStructure)
    )
    .onFalse(
      new InstantCommand(()->suctionSubsystem.SetWantedState(frc.robot.subsystems.SuctionSubsystem.WantedState.RELEASE), superStructure)
    )
    ;

    //TODO: check processor superstate, check the onTrue onFalse
    //idk - processor
    funniOperator.button(14).onTrue(
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.ALGAE_PROCESSOR), superStructure)
    )
    .onFalse(
      new InstantCommand(()->suctionSubsystem.SetWantedState(frc.robot.subsystems.SuctionSubsystem.WantedState.RELEASE), superStructure)
    );

    //x - climber up, really just leds, this is FINAL, already climbed, stage
    funniOperator.button(3).onTrue(
      new InstantCommand(()->ledSubsystem.setPattern(BlinkinPattern.LIME, 10), ledSubsystem)
    );
    
    //left stick - climber down, preparing to climb
    funniOperator.button(9).onTrue(
      new InstantCommand(()->ledSubsystem.setPattern(BlinkinPattern.GRAY, 10), ledSubsystem)
    );

    //y - l2, also l1 but wtv
    funniOperator.button(4).onTrue(
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.MOVE_TO_L2), superStructure)
    );

    //right stick - intake (+ pickup)
    funniOperator.button(10).onTrue(
      new SequentialCommandGroup(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.CORAL_GROUND_INTAKE),superStructure),
        trueWaitUntil(intakeSubsystem.getBeamBreakTrigger()),
        pickupCoralSequence().until(this::getHasCoral),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_PLACE), superStructure)
      )
    );

    //left bumper - l3
    funniOperator.button(5).onTrue(
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.MOVE_TO_L3), superStructure)
    );
    
    //right bumper - l4
    funniOperator.button(6).onTrue(
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.MOVE_TO_L4), superStructure)
    );

    //idk - score + drop algae
    funniOperator.button(12).onTrue(
      new ConditionalCommand(
        new InstantCommand(()->suctionSubsystem.SetWantedState(frc.robot.subsystems.SuctionSubsystem.WantedState.RELEASE)), 
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
        },superStructure), 
        this::getHasAlgae
      )
    );

    //idk - home
    funniOperator.button(17).onTrue(
      new ConditionalCommand(
        new SequentialCommandGroup(
          new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.HOME),superStructure),
          new InstantCommand(()->enableTeleopDriving()),
          new WaitCommand(5)
      ), 
        new SequentialCommandGroup(
          new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.STOW_ALGAE),superStructure),
          new InstantCommand(()->enableTeleopDriving()),
          new WaitCommand(5)
      ), 
      ()->!getHasAlgae())
      );
  }

  public Command pickupCoralSequence(){
    return new SequentialCommandGroup(
      new SequentialCommandGroup(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_RECEIVE),superStructure),
        waitUntil(armSubsystem::getOnTarget, elevatorSubsystem::getOnTarget),
        new RepeatCommand(
          new SequentialCommandGroup(
            new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.CORAL_GROUND_RECEIVE),superStructure),
            new WaitCommand(2.0),
            new InstantCommand(()->suctionSubsystem.SetWantedState(frc.robot.subsystems.SuctionSubsystem.WantedState.RELEASE)),
            new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_RECEIVE),superStructure),
            new WaitCommand(0.2)
          )
        )
      )
      .until(suctionSubsystem::getCoralSuctionGood),
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_PLACE),superStructure),
      waitUntil(()->elevatorSubsystem.getOnTarget(ElevatorConstants.postIntake, 3), ()->armSubsystem.getOnTarget(ArmConstants.intakeAngle, 5)),
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
      waitUntil(elevatorSubsystem::getOnTarget).withTimeout(0.5),// armSubsystem::getOnTarget).withTimeout(0.5),
      new InstantCommand(()->ledSubsystem.blink(BlinkinPattern.RED_ORANGE, 2)),
      new InstantCommand(()->hasCoral=false)
    );
  }


  // public Command pidToReefPosition(Supplier<Pose2d> offsetPose, Supplier<Pose2d> scoringPose){
  //   BooleanSupplier shouldNotGoToOffset = ()->{
  //     Pose2d scoringToRobot = swerveSubsystem.getPose().relativeTo(scoringPose.get());
  //     boolean closeEnough = scoringToRobot.getTranslation().getNorm()<Inches.of(30).in(Meters);
  //     boolean angleInTolerance = Math.abs(scoringToRobot.getRotation().getDegrees()) < 20.0;
  //     boolean canSeeTag = Math.abs(scoringToRobot.getTranslation().getAngle().getDegrees()) < 30.0;
      
  //     return (closeEnough || canSeeTag) && angleInTolerance;
  //   };
  //   return new SequentialCommandGroup(
  //     new ConditionalCommand(
  //       new InstantCommand(),
  //       new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, offsetPose.get()), swerveSubsystem)
  //       .andThen(waitUntil(swerveSubsystem::getCloseEnough).withTimeout(3.0)),
  //       shouldNotGoToOffset
  //     ),
  //     new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, scoringPose.get()), swerveSubsystem),
  //     waitUntil(()->false)
  //   )
  //   .until(()->!superStructure.getIsAtReefState() && FieldNavigation.getTooCloseToTag(swerveSubsystem.getPose()))
  //   .finallyDo((e)->enableTeleopDriving());
  // }

  public void enableTeleopDriving(){
    swerveSubsystem.setWantedState(
      SwerveSubsystem.WantedState.TELEOP_DRIVE, 
      ()->-driver.getLeftY(),
      ()->-driver.getLeftX(),
      ()->-driver.getRightX()
    );
  }

  public boolean getHasCoral(){
    hasCoral = hasCoral && suctionSubsystem.getCoralSuctionApplied();
    return hasCoral;
  }

  //have to access hasAlgae through this method
  public boolean getHasAlgae(){
    hasAlgae = hasAlgae && suctionSubsystem.getAlgaeSuctionApplied();
    return hasAlgae;
  }

  public boolean getCoralMode(){
    return coralMode;
  }

  // public boolean getAutomationEnabled(){
  //   return automationEnabled;
  // }
}
