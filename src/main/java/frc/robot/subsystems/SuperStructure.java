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
    CORAL_GROUND_RECIEVE,
    PREPARE_TO_PLACE,
    MOVE_TO_L4,
    MOVE_TO_L3,
    MOVE_TO_L2,
    MOVE_TO_L1


  }

  public enum CurrentSuperState{
    HOME,
    IDLE,
    PREPARE_TO_INTAKE,
    CORAL_GROUND_INTAKE,
    CORAL_GROUND_RECIEVE,
    PREPARE_TO_PLACE,
    MOVE_TO_L4,
    MOVE_TO_L3,
    MOVE_TO_L2,
    MOVE_TO_L1
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
      case CORAL_GROUND_RECIEVE:
        return CurrentSuperState.CORAL_GROUND_RECIEVE;
      case PREPARE_TO_PLACE:
        return CurrentSuperState.PREPARE_TO_PLACE;
      case MOVE_TO_L4:
        return CurrentSuperState.MOVE_TO_L4;
      case MOVE_TO_L3:
        return CurrentSuperState.MOVE_TO_L3;
      case MOVE_TO_L2:
        return CurrentSuperState.MOVE_TO_L2;
      case MOVE_TO_L1:
        return CurrentSuperState.MOVE_TO_L1;

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
      case CORAL_GROUND_RECIEVE:
        coralGroundReceive();
        break;
      case PREPARE_TO_PLACE:
        prepareToPlace();
        break;
      case MOVE_TO_L4:
        moveToLevel(ArmConstants.moveL4, ElevatorConstants.l4);
        break;
      case MOVE_TO_L3:
        moveToLevel(ArmConstants.moveL3, ElevatorConstants.l3);
        break;
      case MOVE_TO_L2:
        moveToLevel(ArmConstants.moveL2, ElevatorConstants.l2);
        break;
      case MOVE_TO_L1:
        moveToLevel(ArmConstants.moveL1, ElevatorConstants.l1);
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
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.HOME);
  }

  
  private void coralGroundIntake(){
    //Add all the stuff for other subsystem states
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.IDLE);
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.IDLE);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.INTAKE);
  }
  
  private void coralGroundReceive(){
    if (!armNeedsToWait){
      armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.intakeAngle);
    }
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.intake);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.POST_INTAKE);

  }

  private void prepareToPlace(){
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.intakeAngle);
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.postIntake);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.IDLE);
  }

  private void moveToLevel(double armAngle, double elevatorHeight){
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, armAngle);
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, elevatorHeight);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.IDLE);
  }
  
  public void SetWantedState(SuperStructure.WantedSuperState wantedSuperState){
    this.wantedSuperState = wantedSuperState;
  }

}
