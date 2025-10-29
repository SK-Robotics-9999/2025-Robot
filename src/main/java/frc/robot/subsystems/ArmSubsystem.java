// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.MotorConstants;

public class ArmSubsystem extends SubsystemBase {

  SparkMax armMotor = new SparkMax(MotorConstants.kArmID, MotorType.kBrushless);

  // multiply motor rotations to get final, * 360 for rotation to degrees, / 60.926 to get motor rotations to output
  private final static double armConversionFactor = 360.0 / ( (60.0/11.0)*(60.0/34.0)*(114.0/18.0)  ); 
  private final static double absConversionFactor = 360.0;

  private double targetAngle = 90.0;

  private final double maxVelocity = 450.0; //degrees per second, i hope
  private final double maxAccel = 2000.0; //already maxes out, beyond maxing out, cant exactly follow profile, but pushes it to be faster
  private final double maxAlgaeAccel = 375.0;
  private final TrapezoidProfile trapProfile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(maxVelocity, maxAccel)
  );
  private final TrapezoidProfile trapAlgaeProfile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(maxVelocity, maxAlgaeAccel)
  );

  BooleanSupplier isAlgae = ()->false;

  private boolean holdArm = false;

  ArmFeedforward armFF = new ArmFeedforward(0.03, 0.12, 0.0264);//i have no idea if the kv is good, but hope it works...

  private TrapezoidProfile.State trapState = new TrapezoidProfile.State();
  private TrapezoidProfile.State trapGoal = new TrapezoidProfile.State(90.0, 0.0);

  private final double kUpperLimit = 200.0;
  private final double kLowerLimit = -95.0;

  SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(Volts.of(0.05).per(Second), Volts.of(1), Seconds.of(15)), 
    new SysIdRoutine.Mechanism(this::setVoltage, this::logMotors, this)
    );


  public enum WantedState{
    HOME,
    IDLE,
    MOVE_TO_POSITION,
    BRAKE
  }

  public enum SystemState{
    HOMING,
    IDLING,
    MOVING_TO_POSITION,
    BRAKING
  }

  private WantedState wantedState = WantedState.IDLE;
  private WantedState previousWantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(BooleanSupplier isAlgae) {
    armMotor.configure(getArmConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    syncEncoders();

    new Trigger(DriverStation::isEnabled)
    .onTrue(
      new InstantCommand(this::syncEncoders)
    );

    this.isAlgae=isAlgae;
  }
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // SmartDashboard.putNumber("/arm/absValue", armMotor.getAbsoluteEncoder().getPosition());
    // SmartDashboard.putNumber("/arm/relativeValue", armMotor.getEncoder().getPosition());
    // SmartDashboard.putNumber("/arm/relativeVelo", armMotor.getEncoder().getVelocity());

    // SmartDashboard.putNumber("/arm/voltage", armMotor.getAppliedOutput()*armMotor.getBusVoltage());
    // SmartDashboard.putNumber("/arm/current", armMotor.getOutputCurrent()); 

    // SmartDashboard.putNumber("/arm/targetAngle", targetAngle);
    // SmartDashboard.putNumber("/arm/trapGoalAngle", trapGoal.position);
    // SmartDashboard.putNumber("/arm/trapStateAngle", trapState.position);
    // SmartDashboard.putBoolean("/arm/onTarget", getOnTarget());

    systemState = handleStateTransitions();

    ApplyStates();
    previousWantedState = this.wantedState;

    if(!holdArm){
      setArmAngleTrap();
    }
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
      case MOVE_TO_POSITION:
        return SystemState.MOVING_TO_POSITION;
      case BRAKE:
        return SystemState.BRAKING;
    }
    return SystemState.IDLING;
  }

  public void ApplyStates(){
    holdArm = false;
    switch (systemState) {
      case HOMING:
        targetAngle = 90.0;        
        if(trapGoal.position!=targetAngle){
          trapState = new TrapezoidProfile.State(armMotor.getEncoder().getPosition(), armMotor.getEncoder().getVelocity());
          trapGoal.position = targetAngle;
        }
        break;
      case IDLING:
        //targetAngle stays same
        break;
      case MOVING_TO_POSITION:
        if(trapGoal.position!=targetAngle){
          trapState = new TrapezoidProfile.State(armMotor.getEncoder().getPosition(), armMotor.getEncoder().getVelocity());
          trapGoal.position = targetAngle;
        }  
        break;
      case BRAKING:
        holdArm = true;
        break;

    }
  }

  private SparkBaseConfig getArmConfig() {
    SparkBaseConfig armConf = new SparkMaxConfig()
      .inverted(false)
      .smartCurrentLimit(40) //don't need that much but wtv
      .closedLoopRampRate(0.02)
      .idleMode(IdleMode.kBrake)
    ;

    armConf.encoder
      .positionConversionFactor(armConversionFactor)
      .velocityConversionFactor(armConversionFactor/60.0) //rpm to rps
      // .uvwAverageDepth(4)
      // .uvwMeasurementPeriod(2)
    ;

    armConf.absoluteEncoder
      .positionConversionFactor(absConversionFactor)
      .velocityConversionFactor(absConversionFactor/60.0)
      .inverted(false)
    ;

    armConf.softLimit
      .forwardSoftLimit(kUpperLimit).forwardSoftLimitEnabled(true)
      .reverseSoftLimit(kLowerLimit).reverseSoftLimitEnabled(true)
    ;

    armConf.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(1/60.0)
      .positionWrappingEnabled(false)
    ;

    return armConf;
  }


  public void syncEncoders(){
    double absValue = armMotor.getAbsoluteEncoder().getPosition();
    if(absValue>255 && absValue<360){
      absValue-=360;
    }

    armMotor.getEncoder().setPosition(absValue);
  }

  public void setVoltage(Voltage volts){
    armMotor.setVoltage(volts);
  }

  public double getAngle(){
    return armMotor.getEncoder().getPosition();
  }

  public void logMotors(SysIdRoutineLog log){
    log.motor("armMotor")
      .voltage(Volts.of(armMotor.getAppliedOutput()*armMotor.getBusVoltage()))
      .angularPosition(Degrees.of(getAngle()))
      .angularVelocity(DegreesPerSecond.of(armMotor.getEncoder().getVelocity()))
    ;
  }

  //intended to be put in a run command, ff only calculates once
  public void setPIDtoAngle(DoubleSupplier setpoint){
    double ff = armFF.calculate(Radians.convertFrom(armMotor.getEncoder().getPosition(), Degrees), 0.0);
        armMotor.getClosedLoopController()
          .setReference(
            setpoint.getAsDouble(),
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            ff,
            ArbFFUnits.kVoltage
          )
        ;
  }


  public Command stop(){
    return new InstantCommand(()->armMotor.set(0));
  }

  public Command hold(){
    return new RunCommand(()->{
        double ff = armFF.calculate(Math.toRadians(getAngle()), 0.0);
        armMotor.setVoltage(ff);
      },
      this
    );
    
  }

  // public Command setArmAngleTrap(DoubleSupplier setpoint){
  //   return startRun(
  //     ()->{
  //       trapState = new TrapezoidProfile.State(armMotor.getEncoder().getPosition(), armMotor.getEncoder().getVelocity());
  //       trapGoal = new TrapezoidProfile.State(setpoint.getAsDouble(), 0.0);
  //     }, 
  //     ()->{
  //       trapState = trapProfile.calculate(0.02, trapState, trapGoal); //assumes we follow along the trap well, in theory we should be returning our actual but its ok

  //       double ff = armFF.calculate(Radians.convertFrom(armMotor.getEncoder().getPosition(), Degrees), RadiansPerSecond.convertFrom(trapState.velocity, DegreesPerSecond));
  //       armMotor.getClosedLoopController()
  //         .setReference(
  //           trapState.position,
  //           ControlType.kPosition,
  //           ClosedLoopSlot.kSlot0,
  //           ff,
  //           ArbFFUnits.kVoltage
  //         )
  //       ;
  //     }
  //   );
  // }

  public void setArmAngleTrap(){
    TrapezoidProfile profile = isAlgae.getAsBoolean() ? trapAlgaeProfile : trapProfile;
    trapState = profile.calculate(0.02, trapState, trapGoal); //assumes we follow along the trap well, in theory we should be returning our actual but its ok

    double ff = armFF.calculate(Radians.convertFrom(armMotor.getEncoder().getPosition(), Degrees), RadiansPerSecond.convertFrom(trapState.velocity, DegreesPerSecond));
    armMotor.getClosedLoopController()
      .setReference(
        trapState.position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        ff,
        ArbFFUnits.kVoltage
      )
    ;
  }


  public Command getSysIDRoutine(){
    return Commands.sequence(
      routine.quasistatic(Direction.kForward).withTimeout(15),
      hold().withTimeout(1),
      routine.quasistatic(Direction.kReverse).withTimeout(10),
      hold().withTimeout(1),
      routine.dynamic(Direction.kForward).withTimeout(5),
      hold().withTimeout(1),
      routine.dynamic(Direction.kReverse).withTimeout(5)
    );
  }

  public boolean getOnTarget(){
    return MathUtil.isNear(trapGoal.position, getAngle(), 4);
  }

  public boolean getOnTarget(double position){
    return MathUtil.isNear(position, getAngle(), 4);
  }
  
  public boolean getOnTarget(double position, double tolerance){
    SmartDashboard.putBoolean("Arm/isTolerance", MathUtil.isNear(position, getAngle(), tolerance));
    return MathUtil.isNear(position, getAngle(), tolerance);
  }

  public boolean getArmIsSafe(){
    return getAngle()>ArmConstants.lowestAtZeroElevator;
  } 

  public double getTarget(){
    return trapGoal.position;
  }

  public void SetWantedState(WantedState wantedState){
    this.wantedState = wantedState;
  }
  public void SetWantedState(WantedState wantedState, double targetAngle){
    this.wantedState = wantedState;
    this.targetAngle = targetAngle;
  }


}
