    package frc.robot.subsystems;

    import java.util.function.DoubleSupplier;

    import com.revrobotics.spark.SparkBase.PersistMode;
    import com.revrobotics.spark.SparkBase.ResetMode;
    import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
    import com.revrobotics.spark.config.SparkBaseConfig;
    import com.revrobotics.spark.config.SparkMaxConfig;

    import edu.wpi.first.math.MathUtil;
    import edu.wpi.first.math.controller.PIDController;
    import edu.wpi.first.wpilibj.AnalogInput;
    import edu.wpi.first.wpilibj.DigitalInput;
    import edu.wpi.first.wpilibj.DigitalOutput;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;
    import frc.robot.Constants.MotorConstants;

    public class SuctionSubsystem extends SubsystemBase {
        AnalogInput vacuumSensor = new AnalogInput(0);  
        DigitalOutput solenoid = new DigitalOutput(9);

        SparkFlex suctionMotor = new SparkFlex(MotorConstants.kSuctionID, MotorType.kBrushless);

        private final double targetPressureCoral = 40.0;
        private final double targetPressureAlgae = 50.0;

        PIDController pid = new PIDController(12.0/10.0, 0, 0);



        public static enum WantedState{
            HOME,
            IDLE,
            INTAKE_CORAL,
            INTAKE_ALGAE,
            RELEASE
          }
        
          public static enum SystemState{
            HOMING,
            IDLING,
            INTAKING_CORAL,
            INTAKING_ALGAE,
            RELEASE
          }

        private WantedState wantedState = WantedState.HOME;
        private WantedState previousWantedState = WantedState.IDLE;
        private SystemState systemState = SystemState.IDLING;

        public SuctionSubsystem() {
            suctionMotor.configure(getSuctionConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        public void periodic(){
            SmartDashboard.putNumber("/suction/pressure", getPressure());
            SmartDashboard.putBoolean("/suction/solenoidReleased", getSolenoidReleased());

            systemState = handleStateTransitions();
            ApplyStates();
            previousWantedState = wantedState;
        }

        private SparkBaseConfig getSuctionConfig(){
            SparkBaseConfig sucConf = new SparkMaxConfig()
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(40)
            .closedLoopRampRate(0.1)
            ;
            return sucConf;
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
              case INTAKE_CORAL:
                return SystemState.INTAKING_CORAL;
              case INTAKE_ALGAE:
                return SystemState.INTAKING_ALGAE;
              case RELEASE:
                return SystemState.RELEASE;
            }
            return SystemState.IDLING;
          }

        public void ApplyStates(){
            switch (systemState) {
                case HOMING:
                    releaseSolenoid(false);
                    stop();
                    break;
                case IDLING:
                //nothing
                    break;
                case INTAKING_CORAL:
                    releaseSolenoid(false);
                    pidToSuctionCoral();
                    break;
                case INTAKING_ALGAE:
                    releaseSolenoid(false);
                    pidToSuctionAlgae();
                    break;
                case RELEASE:
                    releaseSolenoid(true); //TODO: make this run only once on change
                    stop();
                    break;
            }
          }


        //scales 0.2-4.6, -115 to 0 kPascals
        public double getPressure(){

            double voltageRatio = vacuumSensor.getVoltage()/5.0; //technically supply voltage, idt it matter significantly but we will see
            double pressure = -(voltageRatio-0.92)/0.007652; //technically negative pressure, but positive is more understandable
        
            return pressure;
        }

        public void pidToSuctionCoral(){
            double output = pid.calculate(getPressure(), targetPressureCoral);
            output = MathUtil.clamp(output, 0, 12.0);
            suctionMotor.setVoltage(output);
            
        }

        public void pidToSuctionAlgae(){
            double output = pid.calculate(getPressure(), targetPressureAlgae);
            output = MathUtil.clamp(output, 0, 12.0);
            suctionMotor.setVoltage(output);
            
        }

        public void stop(){
            suctionMotor.set(0);
        }

        public void releaseSolenoid(boolean release){
            solenoid.set(release);
        }

        public boolean getSolenoidReleased(){
            return solenoid.get();
        }

        //for coral
        public boolean getCoralSuctionGood(){
            return getPressure()>35.0;
        }

        public boolean getAlgaeSuctionGood(){
            return getPressure()>35.0; //should probably be higher
        }



        public void SetWantedState(WantedState wantedState){
            this.wantedState = wantedState;
        }
        
    }
