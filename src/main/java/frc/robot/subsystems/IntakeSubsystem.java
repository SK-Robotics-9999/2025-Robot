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
  SparkMax passthroughLeft = new SparkMax(15, MotorType.kBrushless);
  SparkMax passthroughRight = new SparkMax(16, MotorType.kBrushless);
  
  DigitalInput beambreak = new DigitalInput(1);


  private final double absConversionFactor = (18.0/40.0) * 360.0;
  
  private boolean intaking = false;

  public double pidTargetIntake = 157.0; // close to absConversionFactor

  public IntakeSubsystem() {
    pivotMotor.configure(pivotConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rollerMotor.configure(rollerConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    new Trigger(()->intaking)
      .onTrue(
        new RunCommand(this::passStuff)
        .until(()->!beambreak.get()).finallyDo((e)->{stopPassthrough();})
      );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("/intake/pivotMotorABS", pivotMotor.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("/intake/pivotMotor", getAngle());
    SmartDashboard.putBoolean("/intake/intaking", getIsIntaking());
    SmartDashboard.putNumber("/intake/current", pivotMotor.getOutputCurrent());
    SmartDashboard.putBoolean("/intake/beambreak", !beambreak.get());

    if(intaking){
      IntakeStuff();
    }
    else{
      PidToZero();
      // pivotMotor.setVoltage(11.0);
    }
  }

  public SparkBaseConfig rollerConfig(){
    SparkBaseConfig rollerConf = new SparkMaxConfig()
      .smartCurrentLimit(40);
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
    rollerMotor.setVoltage(0);
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

  public void IntakeStuff(){
    if(pivotMotor.getAbsoluteEncoder().getPosition()>130.0){
      pivotMotor.setVoltage(-0.5);
      rollerMotor.setVoltage(0);
    }
    else{
      pivotMotor.setVoltage(0);
      rollerMotor.setVoltage(-8.5);
    }
  }

  public void passStuff(){
    passthroughLeft.setVoltage(2.4);
    passthroughRight.setVoltage(-2.4);
  }
  
  public void stopPassthrough(){
    passthroughLeft.setVoltage(0);
    passthroughRight.setVoltage(0);

  }


}
