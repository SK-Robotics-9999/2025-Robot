// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

public class SuperStructure extends SubsystemBase {
  /** Creates a new SuperStructure. */ 
  private SwerveSubsystem swerveSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private ArmSubsystem armSubsystem;
  private SuctionSubsystem suctionSubsystem;



  public enum WantedSuperState{
    HOME,
    IDLE,
    PREPARE_TO_INTAKE,
    CORAL_GROUND_INTAKE,
    CORAL_GROUND_RECIEVE


  }

  public enum CurrentSuperState{
    HOME,
    IDLE,
    PREPARE_TO_INTAKE,
    CORAL_GROUND_INTAKE
  }

  private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
  private CurrentSuperState currentSuperState = CurrentSuperState.IDLE;
  private CurrentSuperState previousSuperState;
  
  //booleans
  private boolean armNeedsToWait = false;
  private boolean elevatorNeedsToWait = false;




  
  public SuperStructure(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, SuctionSubsystem suck) {
    this.swerveSubsystem = swerveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.suctionSubsystem = suck; //heheheha


  }

  @Override
  public void periodic() {

    currentSuperState = handleStateTransitions();
    ApplyStates();

    // This method will be called once per scheduler run
  }


  private CurrentSuperState handleStateTransitions(){
    previousSuperState = currentSuperState;

    switch (wantedSuperState){
      case IDLE:
        return currentSuperState = CurrentSuperState.IDLE;
      case HOME:
        return CurrentSuperState.HOME;
      case PREPARE_TO_INTAKE:
        return CurrentSuperState.PREPARE_TO_INTAKE;
      case CORAL_GROUND_INTAKE:
        if (previousSuperState==CurrentSuperState.PREPARE_TO_INTAKE){
          return CurrentSuperState.CORAL_GROUND_INTAKE;
        }
        else{
          return CurrentSuperState.IDLE;
        }


    }
    return CurrentSuperState.IDLE;
}

  private void ApplyStates(){
    if (elevatorSubsystem.getArmCanDown()){
      armNeedsToWait = false;
    }
    else{
      armNeedsToWait=true;
    }

    if (armSubsystem.getArmIsSafe()){
      elevatorNeedsToWait=false;
    }
    else{
      elevatorNeedsToWait=true;
    }

    switch(currentSuperState){
      case IDLE:
        //Everything should stay the same
        break;
      case HOME:
        home();
        break;
      case PREPARE_TO_INTAKE:
        prepareToIntake();
        break;
      case CORAL_GROUND_INTAKE:
        coralGroundIntake();
        break;
    }
  }

  private void home(){
    //Add all the stuff for other subsystems states
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.HOME);
    if(!elevatorNeedsToWait){
      elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.HOME);
    }
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.HOME);
  }

  private void prepareToIntake(){
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.preIntake);
    if (!armNeedsToWait){
      armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.intakeAngle);
    }
  }

  private void coralGroundReceive(){
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.IDLE);
    // elevator.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, Constants.ElevatorConstants.Intake);
  }

  private void coralGroundIntake(){
    //Add all the stuff for other subsystem states
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.IDLE);
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.IDLE);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.INTAKE);
  }


  public void SetWantedState(SuperStructure.WantedSuperState wantedSuperState){
    this.wantedSuperState = wantedSuperState;
  }

}
