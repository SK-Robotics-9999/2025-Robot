// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SuctionSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SwerveSubsystem;


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
  SuperStructure superStructure = new SuperStructure(swerveSubsystem, intakeSubsystem, elevatorSubsystem, armSubsystem, suctionSubsystem);

  CommandXboxController driver = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setWantedState(
      SwerveSubsystem.WantedState.TELEOP_DRIVE, 
      ()->-driver.getLeftY(),
      ()->-driver.getLeftX(),
      ()->-driver.getRightX()
    );
    // Configure the trigger bindings
    configureBindings();

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
    // swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommandAllianceManaged(
    //   ()->-driver.getLeftY(),
    //   ()->-driver.getLeftX(),
    //   ()->-driver.getRightX()
    // ));

    // driver.start().onTrue(new InstantCommand(()->swerveSubsystem.resetGyro()));

    // driver.b().whileTrue(elevatorSubsystem.setElevatorPositionTrap(()->40));
    // driver.y().whileTrue(elevatorSubsystem.setElevatorPositionTrap(()->20));
    // driver.a().whileTrue(elevatorSubsystem.setElevatorPositionTrap(()->0));
    // driver.x().whileTrue(elevatorSubsystem.getSysIDRoutine());
    // driver.povLeft().whileTrue(armSubsystem.setArmAngleTrap(()->90));
    // driver.povRight().whileTrue(armSubsystem.setArmAngleTrap(()->-90));

    // driver.povUp()
    //   .whileTrue(new InstantCommand(()->intakeSubsystem.setIntaking(true)))
    //   .whileFalse(new InstantCommand(()->intakeSubsystem.setIntaking(false)))
    // ;

    
    // driver.a().onTrue(new InstantCommand(()-> superStructure.SetWantedState(SuperStructure.WantedSuperState.HOME)));
    // driver.b().onTrue(new InstantCommand(()-> superStructure.SetWantedState(SuperStructure.WantedSuperState.PREPARE_TO_INTAKE)));
    // driver.x().onTrue(new InstantCommand(()-> superStructure.SetWantedState(SuperStructure.WantedSuperState.CORAL_GROUND_INTAKE)));
    // driver.y().onTrue(new InstantCommand(()-> superStructure.SetWantedState(SuperStructure.WantedSuperState.CORAL_GROUND_RECIEVE)));
    // driver.povDown().onTrue(new InstantCommand(()-> superStructure.SetWantedState(SuperStructure.WantedSuperState.PREPARE_TO_PLACE)));
    // driver.povUp().onTrue(new InstantCommand(()-> superStructure.SetWantedState(SuperStructure.WantedSuperState.MOVE_TO_L4)));
    // driver.povLeft().onTrue(new InstantCommand(()-> superStructure.SetWantedState(SuperStructure.WantedSuperState.MOVE_TO_L2)));
    // driver.povRight().onTrue(new InstantCommand(()-> superStructure.SetWantedState(SuperStructure.WantedSuperState.MOVE_TO_L1)));
    // driver.rightBumper().onTrue(new InstantCommand(()-> superStructure.SetWantedState(SuperStructure.WantedSuperState.PLACE_L4)));
    // driver.leftBumper().onTrue(new InstantCommand(()-> superStructure.SetWantedState(SuperStructure.WantedSuperState.PLACE_L2)));
    // driver.leftTrigger().onTrue(new InstantCommand(()-> superStructure.SetWantedState(SuperStructure.WantedSuperState.PLACE_L1)));
    
    driver.rightTrigger().whileTrue( //is it actually a whileTrue
      new StartEndCommand(
        ()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.DRIVE_TO_POINT, new Pose2d()), 
        ()->swerveSubsystem.SetWantedState(SwerveSubsystem.WantedState.TELEOP_DRIVE)
      )
    );
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
