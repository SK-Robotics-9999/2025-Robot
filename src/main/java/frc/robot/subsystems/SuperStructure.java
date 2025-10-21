// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    PREPARE_TO_INTAKE,
    PREPARE_TO_RECEIVE,
    CORAL_GROUND_INTAKE,
    CORAL_GROUND_INTAKE_WITH_ALGAE,
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
    ALGAE_INTAKE_L3,
    ALGAE_INTAKE_L2,
    PULLOUT_ALGAE_INTAKE_L3,
    PULLOUT_ALGAE_INTAKE_L2,
    STOW_ALGAE,
    RELEASE_ALGAE_INTAKE,
    EJECT,
    START_AUTO
  }

  public enum CurrentSuperState{
    HOME,
    IDLE,
    PREPARE_TO_INTAKE,
    PREPARE_TO_RECEIVE,
    CORAL_GROUND_INTAKE,
    CORAL_GROUND_INTAKE_WITH_ALGAE,
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
    ALGAE_INTAKE_L3,
    ALGAE_INTAKE_L2,
    PULLOUT_ALGAE_INTAKE_L3,
    PULLOUT_ALGAE_INTAKE_L2,
    STOW_ALGAE,
    RELEASE_ALGAE_INTAKE,
    EJECT,
    START_AUTO
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

    SmartDashboard.putString("superStructure/SystemState", currentSuperState.toString());

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
      case PREPARE_TO_RECEIVE:
        return CurrentSuperState.PREPARE_TO_RECEIVE;
      case CORAL_GROUND_INTAKE:
        return CurrentSuperState.CORAL_GROUND_INTAKE;
      case CORAL_GROUND_INTAKE_WITH_ALGAE:
        return CurrentSuperState.CORAL_GROUND_INTAKE_WITH_ALGAE;
      case CORAL_GROUND_RECEIVE:
        return CurrentSuperState.CORAL_GROUND_RECEIVE;
      case PREPARE_TO_PLACE:
        return CurrentSuperState.PREPARE_TO_PLACE;
      case EJECT:
        return CurrentSuperState.EJECT;
      case MOVE_TO_L4:
        if(previousSuperState==CurrentSuperState.PREPARE_TO_PLACE || isAReefState(previousSuperState)){
          return CurrentSuperState.MOVE_TO_L4;
        }
        return CurrentSuperState.IDLE;
      case MOVE_TO_L3:
        if(previousSuperState==CurrentSuperState.PREPARE_TO_PLACE || isAReefState(previousSuperState)){
          return CurrentSuperState.MOVE_TO_L3;
        }
        return CurrentSuperState.IDLE;
      case MOVE_TO_L2:
        if(previousSuperState==CurrentSuperState.PREPARE_TO_PLACE || isAReefState(previousSuperState)){
          return CurrentSuperState.MOVE_TO_L2;
        }
        return CurrentSuperState.IDLE;
      case MOVE_TO_L1:
        if(previousSuperState==CurrentSuperState.PREPARE_TO_PLACE || isAReefState(previousSuperState)){
          return CurrentSuperState.MOVE_TO_L1;
        }
        return CurrentSuperState.IDLE;
      case MOVE_TO_BARGE:
        return CurrentSuperState.MOVE_TO_BARGE;
      case ALGAE_INTAKE_L2:
        return CurrentSuperState.ALGAE_INTAKE_L2;
      case ALGAE_INTAKE_L3:
        return CurrentSuperState.ALGAE_INTAKE_L3;
      case START_AUTO:
        return CurrentSuperState.START_AUTO;
      case PLACE_L4:
        if(previousSuperState==CurrentSuperState.MOVE_TO_L4 || previousSuperState==CurrentSuperState.PLACE_L4){
          return CurrentSuperState.PLACE_L4;
        }
        return CurrentSuperState.IDLE;
      case PLACE_L3:
        if(previousSuperState==CurrentSuperState.MOVE_TO_L3 || previousSuperState==CurrentSuperState.PLACE_L3){
          return CurrentSuperState.PLACE_L3;
        }
        return CurrentSuperState.IDLE;
      case PLACE_L2:
        if(previousSuperState==CurrentSuperState.MOVE_TO_L2 || previousSuperState==CurrentSuperState.PLACE_L2){
          return CurrentSuperState.PLACE_L2;
        }
        return CurrentSuperState.IDLE;
      case PLACE_L1:
        if(previousSuperState==CurrentSuperState.MOVE_TO_L1 || previousSuperState==CurrentSuperState.PLACE_L1){
          return CurrentSuperState.PLACE_L1;
        }
        return CurrentSuperState.IDLE;
      case PULLOUT_ALGAE_INTAKE_L2:
        if(previousSuperState==CurrentSuperState.ALGAE_INTAKE_L2 || isAReefState(previousSuperState)){
          return CurrentSuperState.PULLOUT_ALGAE_INTAKE_L2;
        }
        return CurrentSuperState.IDLE;
      case PULLOUT_ALGAE_INTAKE_L3:
        if(previousSuperState==CurrentSuperState.ALGAE_INTAKE_L3 || isAReefState(previousSuperState)){
          return CurrentSuperState.PULLOUT_ALGAE_INTAKE_L3;
        }
        return CurrentSuperState.IDLE;
      case STOW_ALGAE:
        if(previousSuperState==CurrentSuperState.PULLOUT_ALGAE_INTAKE_L2 || previousSuperState==CurrentSuperState.PULLOUT_ALGAE_INTAKE_L3 || previousSuperState==CurrentSuperState.STOW_ALGAE){
          return CurrentSuperState.STOW_ALGAE;
        }
        return CurrentSuperState.IDLE;
      case RELEASE_ALGAE_INTAKE:
        return CurrentSuperState.RELEASE_ALGAE_INTAKE;
        
    }
    return CurrentSuperState.IDLE;
}

  private void ApplyStates(){
    armNeedsToWait = !elevatorSubsystem.getArmCanDown();

    elevatorNeedsToWait = !armSubsystem.getArmIsSafe();

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
      case CORAL_GROUND_INTAKE_WITH_ALGAE:
        coralGroundIntakeWithAlgae();
        break;
      case CORAL_GROUND_RECEIVE:
        coralGroundReceive();
        break;
      case PREPARE_TO_PLACE:
        prepareToPlace();
        break;
      case PREPARE_TO_RECEIVE:
        prepareToReceive();
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
      case ALGAE_INTAKE_L2:
        algaeIntakeL2();
        break;
      case ALGAE_INTAKE_L3:
        algaeIntakeL3();
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
      case PULLOUT_ALGAE_INTAKE_L2:
        pulloutAlgaeIntakeL2();
        break;
      case PULLOUT_ALGAE_INTAKE_L3:
        pulloutAlgaeIntakeL3();
        break;
      case STOW_ALGAE:
        stowAlgae();
        break;
      case RELEASE_ALGAE_INTAKE:
        releaseAlgaeIntake();
        break;
      case EJECT:
        eject();
        break;
      case START_AUTO:
        startAuto();
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
  }
  
  
  private void coralGroundIntake(){
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.preIntake);
    if (!armNeedsToWait){
      armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.intakeAngle);
    }
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.INTAKE);
  }

  private void coralGroundIntakeWithAlgae(){
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.HOME);
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.HOME);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE_ALGAE);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.INTAKE);
  }
  
  private void coralGroundReceive(){
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.intake);
    if (!armNeedsToWait){
      armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.intakeAngle);
    }
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.POST_INTAKE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE_CORAL);
  }

  private void prepareToPlace(){
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.intakeAngle);
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.postIntake);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.POST_INTAKE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE_CORAL);
  }

  private void prepareToReceive(){
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.preIntake);
    if (!armNeedsToWait){
      armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.intakeAngle);
    }
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.POST_INTAKE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE_CORAL);
  }

  private void eject(){
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.EJECT);
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.IDLE);
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.IDLE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.IDLE);
  }

  private void moveToL4(){
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.moveL4);
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.moveL4);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.HOME);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE_CORAL);
  }

  private void moveToL3(){
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.moveL3);
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.moveL3);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.HOME);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE_CORAL);
  }

  private void moveToL2(){
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.moveL2);
    //if() need safety
    if(armSubsystem.getAngle()>ArmConstants.safeL2){
      elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.moveL2);
    }
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.HOME);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE_CORAL);
  }

  private void moveToL1(){
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.moveL1);
    if(armSubsystem.getAngle()>ArmConstants.safeL1){
      elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.moveL1);
    }
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.HOME);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE_CORAL);
  }

  private void moveToBarge(){
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.barge);
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.barge);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.HOME);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE_ALGAE);
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

  private void algaeIntakeL3(){
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.algael3);
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.algaeReefIntake);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.IDLE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE_ALGAE);
  }
  
  private void algaeIntakeL2(){
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.algael2);
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.algaeReefIntake);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.IDLE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE_ALGAE);
  }

  private void pulloutAlgaeIntakeL3(){
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.pulloutAlgael3);
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.pulloutAlgaeReefIntake);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.IDLE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE_ALGAE);
  }
  
  private void pulloutAlgaeIntakeL2(){
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.pulloutAlgael2);
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmConstants.pulloutAlgaeReefIntake);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.IDLE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE_ALGAE);
  }

  private void stowAlgae(){
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.HOME);
    if(!elevatorNeedsToWait){
      elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.HOME);
    }
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.HOME);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE_ALGAE);
  }
  
  private void releaseAlgaeIntake(){
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.IDLE);
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.IDLE);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.IDLE);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.RELEASE);
  }

  private void startAuto(){
    elevatorSubsystem.SetWantedState(ElevatorSubsystem.WantedState.HOME);
    armSubsystem.SetWantedState(ArmSubsystem.WantedState.BRAKE);
    intakeSubsystem.SetWantedState(IntakeSubsystem.WantedState.HOME);
    suctionSubsystem.SetWantedState(SuctionSubsystem.WantedState.INTAKE_CORAL);
  }
  
  public CurrentSuperState getCurrentSuperState(){
    return currentSuperState;
  }

  public boolean getIsAtReefState(){
    return currentSuperState == CurrentSuperState.ALGAE_INTAKE_L2 
    || currentSuperState == CurrentSuperState.ALGAE_INTAKE_L3
    || currentSuperState == CurrentSuperState.PULLOUT_ALGAE_INTAKE_L2
    || currentSuperState == CurrentSuperState.PULLOUT_ALGAE_INTAKE_L3
    || currentSuperState == CurrentSuperState.STOW_ALGAE //Just to make stuff work i guess
    || currentSuperState == CurrentSuperState.MOVE_TO_L1
    || currentSuperState == CurrentSuperState.MOVE_TO_L2
    || currentSuperState == CurrentSuperState.MOVE_TO_L3
    || currentSuperState == CurrentSuperState.MOVE_TO_L4
    || currentSuperState == CurrentSuperState.PLACE_L1
    || currentSuperState == CurrentSuperState.PLACE_L2
    || currentSuperState == CurrentSuperState.PLACE_L3
    || currentSuperState == CurrentSuperState.PLACE_L4
    ;
  }

  public void SetWantedState(SuperStructure.WantedSuperState wantedSuperState){
    this.wantedSuperState = wantedSuperState;
  }

  public boolean isIntaking(){
    return currentSuperState==CurrentSuperState.CORAL_GROUND_INTAKE;
  }

  public boolean isAReefState(CurrentSuperState superState){
    return superState == CurrentSuperState.MOVE_TO_L1 ||
      superState == CurrentSuperState.MOVE_TO_L1 ||
      superState == CurrentSuperState.MOVE_TO_L2 ||
      superState == CurrentSuperState.MOVE_TO_L3 ||
      superState == CurrentSuperState.MOVE_TO_L4 ||
      superState == CurrentSuperState.ALGAE_INTAKE_L2 ||
      superState == CurrentSuperState.ALGAE_INTAKE_L3 ||
      superState == CurrentSuperState.PULLOUT_ALGAE_INTAKE_L2 ||
      superState == CurrentSuperState.PULLOUT_ALGAE_INTAKE_L3
    ;
  }

}
