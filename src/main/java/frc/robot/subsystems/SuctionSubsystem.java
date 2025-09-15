    package frc.robot.subsystems;

    import java.util.function.DoubleSupplier;

    import com.revrobotics.spark.SparkBase.PersistMode;
    import com.revrobotics.spark.SparkBase.ResetMode;
    import com.revrobotics.spark.SparkLowLevel.MotorType;
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

        SparkMax suctionMotor = new SparkMax(MotorConstants.kSuctionID, MotorType.kBrushless);

        PIDController pid = new PIDController(0.01, 0, 0);

        public SuctionSubsystem() {
            suctionMotor.configure(getSuctionConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        public void periodic(){
            SmartDashboard.putNumber("/suction/pressure", getPressure());
            SmartDashboard.putBoolean("/suction/solenoidReleased", getSolenoidReleased());
        }

        private SparkBaseConfig getSuctionConfig(){
            SparkBaseConfig sucConf = new SparkMaxConfig()
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(10)
            .closedLoopRampRate(0.2)
            ;
            return sucConf;
        }
        

        //scales 0.2-4.6, -115 to 0 kPascals
        public double getPressure(){
            double voltageRatio = vacuumSensor.getVoltage()/5.0; //technically supply voltage, idt it matter significantly but we will see
            double pressure = -(voltageRatio-0.92)/0.007652; //technically negative pressure, but positive is more understandable
        
            return pressure;
        }

        public void setPIDtopressure(DoubleSupplier setpoint){
            double error = setpoint.getAsDouble() - getPressure();
            double output = pid.calculate(error);
            output = MathUtil.clamp(output, 0, 1);
            suctionMotor.set(output);
            
        }

        public void releaseSolenoid(boolean release){
            solenoid.set(release);
        }

        public boolean getSolenoidReleased(){
            return solenoid.get();
        }
        
    }
