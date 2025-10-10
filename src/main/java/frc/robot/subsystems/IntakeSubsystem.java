// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.MotorConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  SparkFlex pivotMotor = new SparkFlex(MotorConstants.kIntakePivotID, MotorType.kBrushless);
  SparkMax rollerMotor = new SparkMax(MotorConstants.kIntakeRollersID, MotorType.kBrushless);
  SparkMax passthroughLeft = new SparkMax(MotorConstants.kFeederLeftID, MotorType.kBrushless);
  SparkMax passthroughRight = new SparkMax(MotorConstants.kFeederRightID, MotorType.kBrushless);
  
  DigitalInput beambreak = new DigitalInput(1);




  private final double absConversionFactor = (18.0/40.0) * 360.0;
  
  private boolean intaking = false;

  public double pidTargetIntake = 155.0; // close to absConversionFactor
  public double pidTargetSafe = pidTargetIntake-11.0; //out of reach from pickup

  public enum WantedState{
    HOME,
    IDLE,
    INTAKE,
    POST_INTAKE,
    EJECT
  }

  public enum SystemState{
    HOMING,
    IDLING,
    INTAKING,
    POST_INTAKING,
    EJECTING
  }

  private WantedState wantedState = WantedState.HOME;
  private WantedState previousWantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  public IntakeSubsystem() {
    pivotMotor.configure(pivotConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rollerMotor.configure(rollerConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    passthroughLeft.configure(passthroughConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    passthroughRight.configure(passthroughConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // new Trigger(()->intaking)
    //   .onTrue(
    //     new RunCommand(this::passStuff)
    //     .until(()->!beambreak.get()).finallyDo((e)->{stopPassthrough();})
    //   );
  }

  public SystemState handleStateTransitions(){
    switch (wantedState){
      case HOME:
        if(previousWantedState != WantedState.HOME){
            return SystemState.HOMING;
        }
        return SystemState.IDLING;
      case IDLE:
        return SystemState.IDLING;
      case INTAKE:
        return SystemState.INTAKING;
      case POST_INTAKE:
        return SystemState.POST_INTAKING;
      case EJECT:
        return SystemState.EJECTING;
    }
    return SystemState.IDLING;
  }

  public void ApplyStates(){
    switch (systemState) {
      case HOMING:
        intaking = false;
        PidToZero();
        stopPassthrough();
        case IDLING:
        //nothing
        break;
      case INTAKING:
        intaking = true;
        intakeStuff();
        break;
      case POST_INTAKING:
        PidToSafe();
        stopPassthrough();
        break;
      case EJECTING:
        intaking = false;
        ejectStuff();
        stopPassthrough();
        break;
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("/intake/pivotMotorABS", pivotMotor.getAbsoluteEncoder().getPosition());
    // SmartDashboard.putNumber("/intake/pivotMotor", getAngle());
    // SmartDashboard.putBoolean("/intake/intaking", getIsIntaking());
    SmartDashboard.putNumber("/intake/current", pivotMotor.getOutputCurrent());
    SmartDashboard.putBoolean("/intake/beambreak", !beambreak.get());
    // SmartDashboard.putBoolean("/intake/intaking", intaking);
    // SmartDashboard.putNumber("/intake/passthroughVelocity", passthroughLeft.getEncoder().getVelocity());

    systemState = handleStateTransitions();
    ApplyStates();

    previousWantedState = wantedState;
  }

  public SparkBaseConfig rollerConfig(){
    SparkBaseConfig rollerConf = new SparkMaxConfig()
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake)
    ;
    return rollerConf;
  }

  public SparkBaseConfig pivotConfig(){
    SparkBaseConfig pivotConf = new SparkMaxConfig()
      .inverted(true)
      .smartCurrentLimit(80)
      .closedLoopRampRate(0.2)
      .idleMode(IdleMode.kBrake)
    ;

    pivotConf.absoluteEncoder
      .positionConversionFactor(absConversionFactor)
      .velocityConversionFactor(absConversionFactor/60.0)
    ;

    // pivotConf.softLimit
      // .forwardSoftLimit(absConversionFactor).forwardSoftLimitEnabled(true)
      // .reverseSoftLimit(95).reverseSoftLimitEnabled(true)
    // ;

    pivotConf.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(1/60.0)
      .p(1/180.0, ClosedLoopSlot.kSlot1)
      .positionWrappingEnabled(true)
      .positionWrappingMaxInput(absConversionFactor)
      .positionWrappingMinInput(0)
      .maxOutput(0.2)
    ;

    return pivotConf;
  }

  SparkBaseConfig passthroughConfig (){
    SparkBaseConfig config = new SparkMaxConfig();
    config.encoder
      .velocityConversionFactor(1/60.0)
    ;

    return config;
  }

  public double getAngle(){
    double angle = pivotMotor.getAbsoluteEncoder().getPosition();
    if(angle>absConversionFactor/2.0){
      angle-=absConversionFactor;
    }

    angle+=90; //approximately
    return angle;
  }
  
  public boolean getIsIntaking(){
    return intaking;
  }

  public void setIntaking(boolean isIntaking){
    intaking = isIntaking;
  }

  // pid to 157
  public void PidToZero(){
    stopRollers();
    if(pivotMotor.getAbsoluteEncoder().getPosition()<140){
      pivotMotor.getClosedLoopController()
        .setReference(
          pidTargetIntake,
          ControlType.kPosition,
          ClosedLoopSlot.kSlot0
        );
        
    }
    else{
      pivotMotor.getClosedLoopController()
        .setReference(
          pidTargetIntake,
          ControlType.kPosition,
          ClosedLoopSlot.kSlot1
        );

    }
    }
    public void PidToSafe(){
      stopRollers();
      if(pivotMotor.getAbsoluteEncoder().getPosition()<135){
        pivotMotor.getClosedLoopController()
          .setReference(
            pidTargetSafe,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0
          );
          
      }
      else{
        pivotMotor.getClosedLoopController()
          .setReference(
            pidTargetSafe,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot1
          );
  
      }
      }

  public void intakeStuff(){
    if(pivotMotor.getAbsoluteEncoder().getPosition()>130.0){
      pivotMotor.setVoltage(-1.0);
      // stopRollers();
      rollStuff();
      stopPassthrough();
    }
    else{
      pivotMotor.setVoltage(0);
      rollStuff();
      passStuff();
    }
  }

  public void ejectStuff(){
    pivotMotor.setVoltage(pivotMotor.getAbsoluteEncoder().getPosition()>130.0 ? -0.5 : 0.0);
    rollerMotor.setVoltage(8.5);
  }

  public void rollStuff(){
    rollerMotor.setVoltage(-8.5);
  }
  
  public void stopRollers(){
    rollerMotor.setVoltage(0.0);
  }

  public void passStuff(){
    passthroughLeft.setVoltage(2.4);
    passthroughRight.setVoltage(-2.4);
  }
  
  public void stopPassthrough(){
    passthroughLeft.setVoltage(0);
    passthroughRight.setVoltage(0);

  }

  public void SetWantedState(WantedState wantedState){
    this.wantedState = wantedState;
  }

  public boolean hasCoral(){
    return !beambreak.get();
  }

  public void zeroIntake(){
    pidTargetIntake = pivotMotor.getAbsoluteEncoder().getPosition()-7.0; //cant use getAngle method, i messed it up
    pidTargetIntake%=162;
    pidTargetSafe = pidTargetIntake-11.0; //cant use getAngle method, i messed it up
    pidTargetSafe%=162;

  }


}
