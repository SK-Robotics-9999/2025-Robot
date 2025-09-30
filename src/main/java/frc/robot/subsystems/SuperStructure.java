// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.FieldNavigation;

public class SuperStructure extends SubsystemBase {
  /** Creates a new SuperStructure. */ 
  private IntakeSubsystem intakeSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private ArmSubsystem armSubsystem;
  private SuctionSubsystem suctionSubsystem;
  private boolean leftReef = false;




  public enum WantedSuperState{
    HOME,
    IDLE,
    PREPARE_TO_RECEIVE,
    CORAL_GROUND_INTAKE,
    CORAL_GROUND_RECEIVE,
    PREPARE_TO_PLACE,
    MOVE_TO_L4,
    MOVE_TO_L3,
    MOVE_TO_L2,
    MOVE_TO_L1,
    MOVE_TO_BARGE,
    PLACE_L4,
    PLACE_L3,
    PLACE_L2,
    PLACE_L1,
    PLACE_BARGE,
  }

  public enum CurrentSuperState{
    HOME,
    IDLE,
    PREPARE_TO_INTAKE,
    CORAL_GROUND_INTAKE,
    CORAL_GROUND_RECEIVE,
    PREPARE_TO_PLACE,
    MOVE_TO_L4,
    MOVE_TO_L3,
    MOVE_TO_L2,
    MOVE_TO_L1,
    MOVE_TO_BARGE,
    PLACE_L4,
    PLACE_L3,
    PLACE_L2,
    PLACE_L1,
    PLACE_BARGE,
  }

  private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
  private CurrentSuperState currentSuperState = CurrentSuperState.IDLE;
  private CurrentSuperState previousSuperState;

  private Pose2d currentPose;
  
  //booleans
  private boolean armNeedsToWait = false;
  private boolean elevatorNeedsToWait = false;





  
  public SuperStructure(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, SuctionSubsystem suck) {
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
      case PREPARE_TO_RECEIVE:
        return CurrentSuperState.PREPARE_TO_INTAKE;
      case CORAL_GROUND_INTAKE:
        // if (previousSuperState==CurrentSuperState.PREPARE_TO_INTAKE){
          return CurrentSuperState.CORAL_GROUND_INTAKE;
        // }
        // else{
        //   return CurrentSuperState.IDLE;
        // }
      case CORAL_GROUND_RECEIVE:
        return CurrentSuperState.CORAL_GROUND_RECEIVE;
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
      case MOVE_TO_BARGE:
        return CurrentSuperState.MOVE_TO_BARGE;
      case PLACE_L4:
        if(previousSuperState==CurrentSuperState.MOVE_TO_L4){
          return CurrentSuperState.PLACE_L4;
        }
        else{
          return CurrentSuperState.IDLE;
        }
      case PLACE_L3:
        if(previousSuperState==CurrentSuperState.MOVE_TO_L3){
          return CurrentSuperState.PLACE_L3;
        }
        else{
          return CurrentSuperState.IDLE;
        }
      case PLACE_L2:
        if(previousSuperState==CurrentSuperState.MOVE_TO_L2){
          return CurrentSuperState.PLACE_L2;
        }
        else{
          return CurrentSuperState.IDLE;
        }
      case PLACE_L1:
        if(previousSuperState==CurrentSuperState.MOVE_TO_L1){
          return CurrentSuperState.PLACE_L1;
        }
        else{
          return CurrentSuperState.IDLE;
        }      
        case PLACE_BARGE:
          if(previousSuperState==CurrentSuperState.MOVE_TO_BARGE){
            return CurrentSuperState.PLACE_BARGE;
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
      case CORAL_GROUND_RECEIVE:
        coralGroundReceive();
        break;
      case PREPARE_TO_PLACE:
        prepareToPlace();
        break;
      case MOVE_TO_L4:
        moveToL4();
        break;
      case MOVE_TO_L3:
        moveToL3();
        break;
      case MOVE_TO_L2:
        moveToL2();
        break;
      case MOVE_TO_L1:
        moveToL1();
        break;
      case MOVE_TO_BARGE:
        moveToBarge();
        break;
      case PLACE_L4:
        placeAtL4();
        break;
      case PLACE_L3:
        placeAtL3();
        break;
      case PLACE_L2:
        placeAtL2();
        break;
      case PLACE_L1:
        placeAtL1();
        break;
      case PLACE_BARGE:
        placeAtBarge();
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
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.HOME);
  }

  private void prepareToIntake(){
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.preIntake);
    if (!armNeedsToWait){
      armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.intakeAngle);
    }
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.HOME);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE);
  }

  
  private void coralGroundIntake(){
    //Add all the stuff for other subsystem states
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.IDLE);
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.IDLE);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.INTAKE);
  }
  
  private void coralGroundReceive(){
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.intake);
    if (!armNeedsToWait){
      armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.intakeAngle);
    }
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.POST_INTAKE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE);
  }

  private void prepareToPlace(){
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.intakeAngle);
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.postIntake);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.POST_INTAKE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE);
  }

  private void moveToL4(){
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.moveL4);
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.moveL4);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.IDLE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE);
  }

  private void moveToL3(){
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.moveL3);
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.moveL3);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.IDLE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE);
  }

  private void moveToL2(){
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.moveL2);
    //if() need safety
    if(armSubsystem.getAngle()>ArmConstants.safeL2){
      elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.moveL2);
    }
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.IDLE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE);
  }

  private void moveToL1(){
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.moveL1);
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.moveL1);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.IDLE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE);
  }

  private void moveToBarge(){
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.barge);
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.barge);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.IDLE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE);
  }

  private void placeAtL4(){
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.scoreL4);
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.scoreL4);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.IDLE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.RELEASE);

    //release pressure
  }

  private void placeAtL3(){
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.scoreL3);
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.scoreL3);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.IDLE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.RELEASE);
  }

  private void placeAtL2(){
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.scoreL2);
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.scoreL2);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.IDLE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.RELEASE);
  }
  
  private void placeAtL1(){
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.scoreL1);
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.scoreL1);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.IDLE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.RELEASE);
  }

  private void placeAtBarge(){
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.barge);
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.barge);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.IDLE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.RELEASE);
  }
  
  public CurrentSuperState getCurrentSuperState(){
    return currentSuperState;
  }

  public void SetWantedState(SuperStructure.WantedSuperState wantedSuperState){
    this.wantedSuperState = wantedSuperState;
  }

  public boolean isIntaking(){
    return currentSuperState==CurrentSuperState.CORAL_GROUND_INTAKE;
  }

}
