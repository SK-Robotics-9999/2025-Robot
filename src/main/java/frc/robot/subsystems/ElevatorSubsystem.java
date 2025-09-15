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
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.MotorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    DigitalInput bottomLimitSwitch = new DigitalInput(3);
    boolean hasReset = false;

    public static int pidTarget = 0;
    
    SparkMax elevatorMotorfront = new SparkMax(MotorConstants.kElevatorFrontID, MotorType.kBrushless);
    SparkMax elevatorMotorback = new SparkMax(MotorConstants.kElevatorBackID, MotorType.kBrushless);
    
    
    public final static double elevatorConversionFactor = 1.18110236; //inches per revolution of motor
    
    private final double maxVelocity = 10.0; //inches per second
    private final double maxAccel = 10.0; //inches per second squared
    private final TrapezoidProfile trapProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(maxVelocity, maxAccel)
      );
      
      ElevatorFeedforward ElevatorFF = new ElevatorFeedforward(0.0, 0.0, 0.0);
      
      private TrapezoidProfile.State trapState = new TrapezoidProfile.State();
      private TrapezoidProfile.State trapGoal = new TrapezoidProfile.State();
      
      private final double kUpperLimit = 53.0; //inches
      private final double kLowerLimit = 0; //inches
      
      /** Creates a new ElevatorSubsystem. */
      public ElevatorSubsystem() {
        elevatorMotorfront.configure(getEConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotorback.configure(getEConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotorback.configure(getFollowConfig(), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
        elevatorMotorfront.getEncoder().setPosition(0.0);

  }

  public void periodic() {
    SmartDashboard.putNumber("/elevator/height", elevatorMotorfront.getEncoder().getPosition());
    SmartDashboard.putBoolean("/elevator/limitSwitchEnabled", !bottomLimitSwitch.get());
    SmartDashboard.putBoolean("/elevator/hasReset", hasReset);
    SmartDashboard.putNumber("/elevator/pidTarget", pidTarget);
    SmartDashboard.putBoolean("/elevator/motorsInverted", elevatorMotorback.configAccessor.getFollowerModeInverted());

    // setPIDtoPosition(()->pidTarget);

    if (!hasReset && !bottomLimitSwitch.get()){
      elevatorMotorfront.getEncoder().setPosition(0.0);
      elevatorMotorback.getEncoder().setPosition(0.0);
      hasReset = true;
      SparkBaseConfig config = new SparkMaxConfig();
      config.softLimit.reverseSoftLimitEnabled(true);
      elevatorMotorfront.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      elevatorMotorback.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
  }
    
  private SparkBaseConfig getEConfig() {
    SparkBaseConfig econfig = new SparkMaxConfig()
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(10)
    ;

    econfig.encoder
        .positionConversionFactor(elevatorConversionFactor)
        .velocityConversionFactor(elevatorConversionFactor / 60.0)
        // .uvwAverageDepth(4)
        // .uvwMeasurementPeriod(2)
    ;
    econfig.softLimit
        .forwardSoftLimit(kUpperLimit).forwardSoftLimitEnabled(true)
        .reverseSoftLimit(kLowerLimit).reverseSoftLimitEnabled(false)
    ;

    econfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.05)
        .i(0.0)
        .d(0.0)
    ;

    return econfig;
  }

  private SparkBaseConfig getFollowConfig() {
    SparkBaseConfig econfig = new SparkMaxConfig()
        .follow(elevatorMotorfront, true)
    ;

    return econfig;
  }

    //intended to be put in a run command, ff only calculates once
    public void setPIDtoPosition(DoubleSupplier setpoint){
      double ff = ElevatorFF.calculate(0.0, 0.0); // just get the kg essentially
        elevatorMotorfront.getClosedLoopController()
          .setReference(
            setpoint.getAsDouble(),
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            ff,
            ArbFFUnits.kVoltage
          )
        ;
      }

    public Command setElevatorPositionTrap(DoubleSupplier setpoint){
        return startRun(
            ()->{
                trapState = new TrapezoidProfile.State(elevatorMotorfront.getEncoder().getPosition(), elevatorMotorfront.getEncoder().getVelocity());
                trapGoal = new TrapezoidProfile.State(
                    MathUtil.clamp(setpoint.getAsDouble(), kLowerLimit, kUpperLimit),
                    0.0
                );
            },
            ()->{
                trapState = trapProfile.calculate(0.02, trapState, trapGoal);
                
                double ff = ElevatorFF.calculate(trapState.velocity, 0.0);
                elevatorMotorfront.getClosedLoopController()
                 .setReference(
                   trapState.position,
                  ControlType.kPosition,             //this is wrong //fix
                  ClosedLoopSlot.kSlot0,
                  ff,
                  ArbFFUnits.kVoltage
              )
            ;
            }
        );
    }
}
