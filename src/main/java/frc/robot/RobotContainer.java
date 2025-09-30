// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SuctionSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SuperStructure.WantedSuperState;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;

/** 
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  ArmSubsystem armSubsystem = new ArmSubsystem();
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

    driver.start().onTrue(new InstantCommand(()->swerveSubsystem.resetGyro()));

    //intake
    driver.leftTrigger().onTrue(new SequentialCommandGroup(
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_RECEIVE)),
      waitUntil(armSubsystem::getOnTarget, elevatorSubsystem::getOnTarget),
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.CORAL_GROUND_INTAKE)),
      waitUntil(()->intakeSubsystem.hasCoral()&&!driver.rightTrigger().getAsBoolean()),
      new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_RECEIVE))
      ));

      //Move To l1
      driver.a().onTrue(
        new SequentialCommandGroup(
          new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_RECEIVE)),
          waitUntil(armSubsystem::getOnTarget, elevatorSubsystem::getOnTarget),
          new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.CORAL_GROUND_RECEIVE)),
          waitUntil(elevatorSubsystem::getOnTarget, suctionSubsystem::getSuctionGood),
          new WaitCommand(0.5),
          new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_PLACE)),
          waitUntil(elevatorSubsystem::getOnTarget, armSubsystem::getOnTarget),
          new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.MOVE_TO_L1))
        )
      );
      
      //Move To l2
      driver.b().onTrue(
        new SequentialCommandGroup(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_RECEIVE)),
        waitUntil(armSubsystem::getOnTarget, elevatorSubsystem::getOnTarget),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.CORAL_GROUND_RECEIVE)),
        waitUntil(elevatorSubsystem::getOnTarget, suctionSubsystem::getSuctionGood),
        new WaitCommand(0.5),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_PLACE)),
        waitUntil(elevatorSubsystem::getOnTarget, armSubsystem::getOnTarget),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.MOVE_TO_L2))
      )
    );
    //Move To l3
    driver.x().onTrue(
      new SequentialCommandGroup(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_RECEIVE)),
        waitUntil(armSubsystem::getOnTarget, elevatorSubsystem::getOnTarget),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.CORAL_GROUND_RECEIVE)),
        waitUntil(elevatorSubsystem::getOnTarget, suctionSubsystem::getSuctionGood),
        new WaitCommand(0.5),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_PLACE)),
        waitUntil(elevatorSubsystem::getOnTarget, armSubsystem::getOnTarget),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.MOVE_TO_L3))
      )
    );
    //Move To l4
    driver.y().onTrue(
      new SequentialCommandGroup(
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_RECEIVE)),
        waitUntil(armSubsystem::getOnTarget, elevatorSubsystem::getOnTarget),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.CORAL_GROUND_RECEIVE)),
        waitUntil(elevatorSubsystem::getOnTarget, suctionSubsystem::getSuctionGood),
        new WaitCommand(0.5),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.PREPARE_TO_PLACE)),
        waitUntil(elevatorSubsystem::getOnTarget, armSubsystem::getOnTarget),
        new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.MOVE_TO_L4))
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
    }));

    driver.leftBumper().whileTrue(new SequentialCommandGroup(
        //the right to the vision of the apriltag is left when facing at the apriltag.
        new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, FieldNavigation.getOffsetCoralRight(swerveSubsystem.getPose()))),
        waitUntil(swerveSubsystem::getOnTarget),
        new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, FieldNavigation.getCoralRight(swerveSubsystem.getPose()))),
        waitUntil(swerveSubsystem::getOnTarget)
      )
      .finallyDo((e)->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.TELEOP_DRIVE))
    );
    
    driver.rightBumper().whileTrue(new SequentialCommandGroup(
        //the right to the vision of the apriltag is left when facing at the apriltag.
        new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, FieldNavigation.getOffsetCoralLeft(swerveSubsystem.getPose()))),
        waitUntil(swerveSubsystem::getOnTarget),
        new InstantCommand(()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, FieldNavigation.getCoralLeft(swerveSubsystem.getPose()))),
        waitUntil(swerveSubsystem::getOnTarget)
      )
      .finallyDo((e)->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.TELEOP_DRIVE))
    );

    driver.povUp().onTrue(new InstantCommand(()->superStructure.SetWantedState(WantedSuperState.HOME)));
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
}
