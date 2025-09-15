// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Constants.MotorConstants;

public class ArmSubsystem extends SubsystemBase {

  SparkMax armMotor = new SparkMax(MotorConstants.kArmID, MotorType.kBrushless);

  // multiply motor rotations to get final, * 360 for rotation to degrees, / 60.926 to get motor rotations to output
  private final static double armConversionFactor = 360.0 / ( (60.0/11.0)*(60.0/34.0)*(114.0/18.0)  ); 
  private final static double absConversionFactor = 360.0;

  public int pidTarget = 90;

  private final double maxVelocity = 30.0; //degrees per second, i hope
  private final double maxAccel = 15.0;
  private final TrapezoidProfile trapProfile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(maxVelocity, maxAccel)
  );

  ArmFeedforward armFF = new ArmFeedforward(0.03, 0.13, 0.0);

  private TrapezoidProfile.State trapState = new TrapezoidProfile.State();
  private TrapezoidProfile.State trapGoal = new TrapezoidProfile.State();

  private final double kUpperLimit = 200.0;
  private final double kLowerLimit = -95.0;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armMotor.configure(getArmConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    syncEncoders();

    new Trigger(DriverStation::isEnabled)
    .onTrue(
      new InstantCommand(this::syncEncoders)
    );

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    SmartDashboard.putNumber("/arm/absValue", armMotor.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("/arm/relativeValue", armMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("/arm/pidTarget", pidTarget);

    setPIDtoAngle(()->pidTarget);
  }

  private SparkBaseConfig getArmConfig() {
    SparkBaseConfig armConf = new SparkMaxConfig()
      .inverted(false)
      .smartCurrentLimit(10)
      .closedLoopRampRate(0.2)
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
      //.velocityFF(12.0 / ( (1/0.77) *360)) //12 volts (approx), how many volts per degree, at 12 volts it can do 1 rotation in 0.77 seconds, dps calculated, pid should carry
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

  //intended to be put in a run command, ff only calculates once
  public void setPIDtoAngle(DoubleSupplier setpoint){
    SmartDashboard.putNumber("/arm/setpoint", setpoint.getAsDouble());
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

  public Command setArmAngleTrap(DoubleSupplier setpoint){
    return startRun(
      ()->{
        trapState = new TrapezoidProfile.State(armMotor.getEncoder().getPosition(), armMotor.getEncoder().getVelocity());
        trapGoal = new TrapezoidProfile.State(setpoint.getAsDouble(), 0.0);
      }, 
      ()->{
        trapState = trapProfile.calculate(0.02, trapState, trapGoal); //assumes we follow along the trap well, in theory we should be returning our actual but its ok

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
    );
  }

}
