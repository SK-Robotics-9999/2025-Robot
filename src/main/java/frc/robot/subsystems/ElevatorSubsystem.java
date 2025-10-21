package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  DigitalInput bottomLimitSwitch = new DigitalInput(3);
  boolean hasReset = false;

  SparkMax elevatorMotorFront = new SparkMax(MotorConstants.kElevatorFrontID, MotorType.kBrushless);
  SparkMax elevatorMotorBack = new SparkMax(MotorConstants.kElevatorBackID, MotorType.kBrushless);
  
  
  public final static double elevatorConversionFactor = 1.18110236; //inches per revolution of motor
  public final static double stage1To2Height = 24.5;//don't want to convert rn
  
  private final double maxVelocity = 90.0; //inches per second
  private final double maxVelocityDown = 60.0; //inches per second
  private final double maxAccel = 350.0; //inches per second squared
  private final double maxAccelDown = 250.0; //inches per second squared
  private final TrapezoidProfile trapProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAccel));
  private final TrapezoidProfile trapDownProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocityDown, maxAccelDown));
    
  ElevatorFeedforward elevatorFF = new ElevatorFeedforward(0.08, 0.22, 0.0);
  final ElevatorFeedforward lowElevatorFF = new ElevatorFeedforward(0.08, 0.22, 0.121);
  final ElevatorFeedforward highElevatorFF = new ElevatorFeedforward(0.09, 0.31, 0.121); //LOW range kv
  
  private TrapezoidProfile.State trapState = new TrapezoidProfile.State();
  private TrapezoidProfile.State trapGoal = new TrapezoidProfile.State();
  
  private final double kUpperLimit = 53.0; //inches
  private final double kLowerLimit = 0; //inches

  private double targetPosition = 0.0;


  SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(Volts.of(0.15).per(Second), Volts.of(1.5), Seconds.of(10)), 
    new SysIdRoutine.Mechanism(this::setVoltage, this::logMotors, this)
  );

  public static enum WantedState{
    HOME,
    IDLE,
    MOVE_TO_POSITION
  }

  public static enum SystemState{
    HOMING,
    IDLING,
    MOVING_TO_POSITION
  }

  private WantedState wantedState = WantedState.IDLE;
  private WantedState previousWantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

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
    }
    return SystemState.IDLING;
  }

  public void ApplyStates(){
    switch (systemState) {
      case HOMING:
        targetPosition = 0.0;
        if(trapGoal.position!=targetPosition){
          trapState = new TrapezoidProfile.State(elevatorMotorFront.getEncoder().getPosition(), elevatorMotorFront.getEncoder().getVelocity());
        trapGoal.position = targetPosition;
        }
        break;
      case IDLING:
        //stay wherever we currently are, target position persists
        break;
      case MOVING_TO_POSITION:
        if(trapGoal.position!=targetPosition){
        trapState = new TrapezoidProfile.State(elevatorMotorFront.getEncoder().getPosition(), elevatorMotorFront.getEncoder().getVelocity());
        trapGoal.position = targetPosition;
        }
        break;
    }
  }
  
      
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorMotorFront.configure(getEConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorMotorBack.configure(getEConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorMotorBack.configure(getFollowConfig(), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
    elevatorMotorFront.getEncoder().setPosition(0.0);

    elevatorFF = lowElevatorFF;
  }

  public void periodic() {
    // SmartDashboard.putNumber("/elevator/height", elevatorMotorFront.getEncoder().getPosition());
    // SmartDashboard.putNumber("/elevator/vel", elevatorMotorFront.getEncoder().getVelocity());
    // SmartDashboard.putBoolean("/elevator/limitSwitchEnabled", !bottomLimitSwitch.get());
    // SmartDashboard.putBoolean("/elevator/hasReset", hasReset);
    // SmartDashboard.putBoolean("/elevator/motorsInverted", elevatorMotorBack.configAccessor.getFollowerModeInverted());
    // SmartDashboard.putNumber("/elevator/target", trapGoal.position);
    // SmartDashboard.putNumber("/elevator/trapStateInches", trapState.position);

    // SmartDashboard.putNumber("/elevator/voltage", elevatorMotorFront.getAppliedOutput()*elevatorMotorFront.getBusVoltage());
    // SmartDashboard.putNumber("/elevator/voltageBack", -elevatorMotorBack.getAppliedOutput()*elevatorMotorBack.getBusVoltage());
    // SmartDashboard.putNumber("/elevator/current", elevatorMotorFront.getOutputCurrent());


    //for some reason doesn't work in a trigger, whatever, not important rn i hope
    if (!hasReset && !bottomLimitSwitch.get()){
      elevatorMotorFront.getEncoder().setPosition(0.0);
      elevatorMotorBack.getEncoder().setPosition(0.0);
      hasReset = true;
      SparkBaseConfig config = new SparkMaxConfig();
      config.softLimit.reverseSoftLimitEnabled(true);
      elevatorMotorFront.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      elevatorMotorBack.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    if (getHeight()>stage1To2Height){
      elevatorFF=highElevatorFF;
    }
    else{
      elevatorFF=lowElevatorFF;
    }


    systemState = handleStateTransitions();
    ApplyStates();

    previousWantedState = this.wantedState;

    SetElevatorPositionTrap();
  }
    
  private SparkBaseConfig getEConfig() {
    SparkBaseConfig econfig = new SparkMaxConfig()
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40)
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
      .p(0.1)
      .i(0.0)
      .d(0.0)
    ;

    return econfig;
  }

  private SparkBaseConfig getFollowConfig() {
    SparkBaseConfig econfig = new SparkMaxConfig()
      .follow(elevatorMotorFront, true)
    ;

    return econfig;
  }

  /**
   * @return inches
   */
  public double getHeight(){
    return elevatorMotorFront.getEncoder().getPosition();
  }

  /**
   * @return inches/sec
   */
  public double getVelocity(){
    return elevatorMotorFront.getEncoder().getVelocity();
  }

  private void setVoltage(Voltage volts){
    elevatorMotorFront.setVoltage(volts);
  }

  private void logMotors(SysIdRoutineLog log){
    log.motor("armMotor")
      .voltage(Volts.of(elevatorMotorFront.getAppliedOutput()*elevatorMotorFront.getBusVoltage()))
      .linearPosition(Inches.of(getHeight()))
      .linearVelocity(InchesPerSecond.of(elevatorMotorFront.getEncoder().getVelocity()))
    ;
  }

  public Command hold(){
    return new RunCommand(()->elevatorMotorFront.setVoltage(elevatorFF.calculate(0.0)), 
      this
    );
  }

  //intended to be put in a run command, ff only calculates once
  public void setPIDtoPosition(DoubleSupplier setpoint){
    // setpoint =  ()->MathUtil.clamp(setpoint.getAsDouble(), kLowerLimit, kUpperLimit);
    double ff = elevatorFF.calculate(0.0, 0.0); // just get the kg essentially
    elevatorMotorFront.getClosedLoopController()
      .setReference(
        setpoint.getAsDouble(),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        ff,
        ArbFFUnits.kVoltage
      )
    ;
  }


  // public Command setElevatorPositionTrap(DoubleSupplier setpoint){
  //   return startRun(
  //     ()->{
  //         trapState = new TrapezoidProfile.State(elevatorMotorFront.getEncoder().getPosition(), elevatorMotorFront.getEncoder().getVelocity());
  //         trapGoal = new TrapezoidProfile.State(
  //           MathUtil.clamp(setpoint.getAsDouble(), kLowerLimit, kUpperLimit),
  //           0.0
  //         );
  //     },
  //     ()->{
  //       trapState = trapProfile.calculate(0.02, trapState, trapGoal);
        
  //       double ff = elevatorFF.calculate(trapState.velocity);
  //       elevatorMotorFront.getClosedLoopController()
  //         .setReference(
  //           trapState.position,
  //           ControlType.kPosition,             //this is wrong //fix
  //           ClosedLoopSlot.kSlot0,
  //           ff,
  //           ArbFFUnits.kVoltage
  //         )
  //       ;
  //     }
  //   );
  // }

  public void SetElevatorPositionTrap(){
    TrapezoidProfile tempProfile = trapGoal.position>getHeight() || ElevatorConstants.intake==trapGoal.position ? trapProfile : trapDownProfile;
    trapState = tempProfile.calculate(0.02, trapState, trapGoal);
    double ff = elevatorFF.calculate(trapState.velocity);
    elevatorMotorFront.getClosedLoopController()
      .setReference(
        trapState.position,
         ControlType.kPosition,
         ClosedLoopSlot.kSlot0,
            ff,
            ArbFFUnits.kVoltage
      );
  }



  public Command getSysIDRoutine(){
    return Commands.sequence(
      routine.quasistatic(Direction.kForward).withTimeout(10),
      hold().withTimeout(1),
      routine.quasistatic(Direction.kReverse).withTimeout(5),
      hold().withTimeout(1),
      routine.dynamic(Direction.kForward).withTimeout(5),
      hold().withTimeout(1),
      routine.dynamic(Direction.kReverse).withTimeout(5)
    );
  }

  public boolean getOnTarget(){
    return MathUtil.isNear(trapGoal.position, getHeight(), 1.0);
  }

  public boolean getOnTarget(double position){
    return MathUtil.isNear(position, getHeight(), 1.0);
  }

  public boolean getOnTarget(double position, double tolerance){
    return MathUtil.isNear(position, getHeight(), tolerance);
  }

  //doesn't work
  public boolean getArmCanDown(){
    //intake height, not prepare to intake height, as it is simply the minimum possible height to rotate arm 360
    return getHeight()>ElevatorConstants.intake-6.0 && trapGoal.position>ElevatorConstants.intake;
  }

  public void SetWantedState(WantedState wantedState){
    this.wantedState = wantedState;
  }
  public void SetWantedState(WantedState wantedState, double position){
    this.wantedState = wantedState;
    targetPosition = position;
  }
}
