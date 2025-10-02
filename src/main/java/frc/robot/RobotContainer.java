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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SuctionSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.WantedSuperState;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

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

  // The robot's subsystems and commands are defined here...
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  ArmSubsystem armSubsystem = new ArmSubsystem(()->!coralMode);
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  SuctionSubsystem suctionSubsystem = new SuctionSubsystem();
  SuperStructure superStructure = new SuperStructure( intakeSubsystem, elevatorSubsystem, armSubsystem, suctionSubsystem);

  VisionSubsystem visionSubsystem = new VisionSubsystem(swerveSubsystem, superStructure);
  // QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem();
  
  CommandXboxController driver = new CommandXboxController(0);


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
    swerveSubsystem.setWantedState(
      SwerveSubsystem.WantedState.TELEOP_DRIVE, 
      ()->-driver.getLeftY(),
      ()->-driver.getLeftX(),
      ()->-driver.getRightX()
    );

    // driver.start().onTrue(new InstantCommand(()->swerveSubsystem.resetGyro()));
    driver.povRight().onTrue(new InstantCommand(()->coralMode=!coralMode));

    //intake
    driver.leftTrigger()
    .whileTrue(
      new RunCommand(()->swerveSubsystem.setWantedState(
        SwerveSubsystem.WantedState.ASSISTED_TELEOP_DRIVE, 
        ()->{
          Rotation2d poseRotation = FieldNavigation.getNearestReef(swerveSubsystem.getPose()).getRotation();
          double xAssist = 0.2*Math.sin(poseRotation.getRadians());
          xAssist *= isRed.getAsBoolean() ? -1 : 1;

          return xAssist;
        },
        ()->{
          Rotation2d poseRotation = FieldNavigation.getNearestReef(swerveSubsystem.getPose()).getRotation();
          double yAssist = 0.2*Math.cos(poseRotation.getRadians());
          yAssist *= isRed.getAsBoolean() ? -1 : 1;

          return yAssist;
        },
        ()->-driver.getLeftY(),
        ()->-driver.getLeftX(),
        ()->-driver.getRightX(),
        false
      ), swerveSubsystem)

      .until(()->!FieldNavigation.getTooCloseToTag(swerveSubsystem.getPose()))

      .andThen(new RunCommand(()->swerveSubsystem.setWantedState(
        SwerveSubsystem.WantedState.ASSISTED_TELEOP_DRIVE, 
        ()->0,
        ()->{
          Optional<Translation2d> optional = visionSubsystem.getObjectTranslationRelative();
          if(optional.isEmpty()){return 0.0;}
          Translation2d translation = optional.get();
          if(Math.abs(translation.getY())<Inches.of(1).in(Meters)){return 0.0;}
          ChassisSpeeds robotRelative = swerveSubsystem.getRobotRelativeSpeeds();
          
          double xSpeed = robotRelative.vxMetersPerSecond;
          if(xSpeed<0.0){return 0.0;}
          xSpeed=Math.sqrt(xSpeed);
          Rotation2d robotToCoral = translation.getAngle();
          double yAssist = xSpeed * robotToCoral.getTan();
          yAssist*=5.0;

          if(Math.abs(yAssist)<0.05){return 0.0;}
          
          SmartDashboard.putNumber("vision/intaking/yAssist", yAssist);
          return yAssist;
        },
        ()->-driver.getLeftY(),
        ()->-driver.getLeftX(),
        ()->-driver.getRightX(),
        true
      ), swerveSubsystem))

      .finallyDo((e)->swerveSubsystem.setWantedState(
        SwerveSubsystem.WantedState.TELEOP_DRIVE, 
        ()->-driver.getLeftY(),
        ()->-driver.getLeftX(),
        ()->-driver.getRightX()
      ))
    )
    .onTrue(new ConditionalCommand(
      new SequentialCommandGroup(
        waitUntil(()->!FieldNavigation.getTooCloseToTag(swerveSubsystem.getPose())),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.CORAL_GROUND_INTAKE),superStructure),
        waitUntil(intakeSubsystem::hasCoral,()->!driver.rightTrigger().getAsBoolean(),armSubsystem::getOnTarget, elevatorSubsystem::getOnTarget),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_INTAKE),superStructure)
      ),

      new InstantCommand(()->{
        switch(superStructure.getCurrentSuperState()){
          case ALGAE_INTAKE_L2:
            superStructure.SetWantedState(WantedSuperState.PULLOUT_ALGAE_INTAKE_L2);
            break;
          case ALGAE_INTAKE_L3:
            superStructure.SetWantedState(WantedSuperState.PULLOUT_ALGAE_INTAKE_L3);
            break;
        }
      }),

      ()->coralMode
    ))
    ;

      //Move To l1
      driver.a().onTrue(
      new SequentialCommandGroup(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_RECEIVE),superStructure),
        waitUntil(armSubsystem::getOnTarget, elevatorSubsystem::getOnTarget),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.CORAL_GROUND_RECEIVE),superStructure),
        waitUntil(elevatorSubsystem::getOnTarget, suctionSubsystem::getSuctionGood),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_PLACE),superStructure),
        waitUntil(elevatorSubsystem::getOnTarget, armSubsystem::getOnTarget),
        new InstantCommand(()->hasCoral=true)
      )
      .until(()->hasCoral)
      .andThen(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.MOVE_TO_L1),superStructure)
      )
      );
      
      //Move To l2
      driver.b().onTrue(new ConditionalCommand(
      new SequentialCommandGroup(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_RECEIVE),superStructure),
        waitUntil(armSubsystem::getOnTarget, elevatorSubsystem::getOnTarget),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.CORAL_GROUND_RECEIVE),superStructure),
        waitUntil(elevatorSubsystem::getOnTarget, suctionSubsystem::getSuctionGood),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_PLACE),superStructure),
        waitUntil(elevatorSubsystem::getOnTarget, armSubsystem::getOnTarget),
        new InstantCommand(()->hasCoral=true)
      )
      .until(()->hasCoral)
      .andThen(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.MOVE_TO_L2),superStructure)
      ),
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.ALGAE_INTAKE_L2), superStructure),
      ()->coralMode
      ));

    //Move To l3
    driver.x().onTrue(new ConditionalCommand(
      new SequentialCommandGroup(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_RECEIVE),superStructure),
        waitUntil(armSubsystem::getOnTarget, elevatorSubsystem::getOnTarget),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.CORAL_GROUND_RECEIVE),superStructure),
        waitUntil(elevatorSubsystem::getOnTarget, suctionSubsystem::getSuctionGood),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_PLACE),superStructure),
        waitUntil(elevatorSubsystem::getOnTarget, armSubsystem::getOnTarget),
        new InstantCommand(()->hasCoral=true)
      )
      .until(()->hasCoral)
      .andThen(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.MOVE_TO_L3),superStructure)
      ),
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.ALGAE_INTAKE_L3),superStructure),
      ()->coralMode
    ));
    //Move To l4
    driver.y().onTrue(
      new SequentialCommandGroup(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_RECEIVE),superStructure),
        waitUntil(armSubsystem::getOnTarget, elevatorSubsystem::getOnTarget),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.CORAL_GROUND_RECEIVE),superStructure),
        waitUntil(elevatorSubsystem::getOnTarget, suctionSubsystem::getSuctionGood),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_PLACE),superStructure),
        waitUntil(elevatorSubsystem::getOnTarget, armSubsystem::getOnTarget),
        new InstantCommand(()->hasCoral=true)
      )
      .until(()->hasCoral)
      .andThen(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.MOVE_TO_L4),superStructure)
      )
    );

    driver.rightTrigger().onTrue(new InstantCommand(()->{
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
    .alongWith(new InstantCommand(()->hasCoral=false))
    );

    driver.leftBumper().whileTrue(new SequentialCommandGroup(
        //the right to the vision of the apriltag is left when facing at the apriltag.
        new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, FieldNavigation.getOffsetCoralRight(swerveSubsystem.getPose())), swerveSubsystem),
        waitUntil(swerveSubsystem::getOnTarget),
        new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, FieldNavigation.getCoralRight(swerveSubsystem.getPose())), swerveSubsystem),
        waitUntil(swerveSubsystem::getOnTarget)
      )
      .finallyDo((e)->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.TELEOP_DRIVE))
    );
    
    driver.rightBumper().whileTrue(new SequentialCommandGroup(
        //the right to the vision of the apriltag is left when facing at the apriltag.
        new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, FieldNavigation.getOffsetCoralLeft(swerveSubsystem.getPose())), swerveSubsystem),
        waitUntil(swerveSubsystem::getOnTarget),
        new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, FieldNavigation.getCoralLeft(swerveSubsystem.getPose())), swerveSubsystem),
        waitUntil(swerveSubsystem::getOnTarget)
      )
      .finallyDo((e)->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.TELEOP_DRIVE))
    );

    driver.start().whileTrue(new SequentialCommandGroup(
      new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, FieldNavigation.getOffsetCoralMid(swerveSubsystem.getPose()))),
      waitUntil(swerveSubsystem::getOnTarget),
      new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, FieldNavigation.getCoralMid(swerveSubsystem.getPose())), swerveSubsystem),
      waitUntil(swerveSubsystem::getOnTarget)
    )
      .finallyDo((e)->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.TELEOP_DRIVE))
    );

    driver.povUp().onTrue(new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.HOME),superStructure));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new InstantCommand();
  }

  public boolean getCoralMode(){
    return coralMode;
  }
}
