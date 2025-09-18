// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SuperStructure extends SubsystemBase {
  /** Creates a new SuperStructure. */ 
  private SwerveSubsystem swerve;
  private IntakeSubsystem intake;
  private ElevatorSubsystem elevator;
  private ArmSubsystem arm;
  private SuctionSubsystem suck;



  public enum WantedSuperState{
    HOME,
    IDLE,
    CORAL_GROUND_INTAKE


  }

  public enum CurrentSuperState{
    HOME,
    IDLE,
    CORAL_GROUND_INTAKE
  }

  private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
  private CurrentSuperState currentSuperState = CurrentSuperState.IDLE;
  private CurrentSuperState previousSuperState;




  
  public SuperStructure(SwerveSubsystem swerve, IntakeSubsystem intake, ElevatorSubsystem elevator, ArmSubsystem arm, SuctionSubsystem suck) {
    this.swerve = swerve;
    this.intake = intake;
    this.elevator = elevator;
    this.arm = arm;
    this.suck = suck;


  }

  @Override
  public void periodic() {


    // This method will be called once per scheduler run
  }


  private CurrentSuperState handleStateTransitions(){
    previousSuperState = currentSuperState;
    switch (wantedSuperState){
      case IDLE:
        currentSuperState = currentSuperState.IDLE;
        break;
      case HOME:
        currentSuperState = currentSuperState.HOME;
        break;
      case CORAL_GROUND_INTAKE:
        currentSuperState = currentSuperState.CORAL_GROUND_INTAKE;



    }
    return currentSuperState;
}

  private void ApplyStates(){
    switch(currentSuperState){
      case IDLE:
        //Everything should stay the same
      case HOME:
        Home();
    }
  }

  private void Home(){
    //Add all the stuff for other subsystems states
    arm.SetWantedState(ArmSubsystem.WantedState.HOME);
  }

  private void CoralGroundIntake(){
    //Add all the stuff for other subsystem states
    arm.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, Constants.ArmConstants.intakeAngle);
  }
}
