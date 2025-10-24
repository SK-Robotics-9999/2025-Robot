// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LEDSubsystem extends SubsystemBase {
  Spark leds = new Spark(0);
  BooleanSupplier algaeMode = ()->false;

  public static enum BlinkinPattern{
    HOT_PINK(+0.57),
    DARK_RED(+0.59),
    RED(+0.61),
    RED_ORANGE(+0.63),
    ORANGE(+0.65),
    GOLD(+0.67),
    YELLOW(+0.69),
    LAWN_GREEN(+0.71),
    LIME(+0.73),
    DARK_GREEN(+0.75),
    GREEN(+0.77),
    BLUE_GREEN(+0.79),
    AQUA(+0.81),
    SKY_BLUE(+0.83),
    DARK_BLUE(+0.85),
    BLUE(+0.87),
    BLUE_VIOLET(+0.89),
    VIOLET(+0.91),
    WHITE(+0.93),
    GRAY(+0.95),
    DARK_GRAY(+0.97),
    BLACK(+0.99);

    public final double value;
    private BlinkinPattern(double value) {
      this.value = value;
    }
  }

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(BooleanSupplier algaeMode) {
    this.algaeMode=algaeMode;

    //could technically improve, do onChange
    setDefaultCommand(new RunCommand(()->{
      if(algaeMode.getAsBoolean()){
        setPattern(BlinkinPattern.AQUA);
      }
      else{
        setPattern(BlinkinPattern.YELLOW);
      }
    }, this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPattern(BlinkinPattern pattern){
    leds.set(pattern.value);
  }

  public void setPattern(BlinkinPattern pattern, double seconds){
    Commands.startRun(()->setPattern(pattern), ()->{}, this).withTimeout(seconds).schedule();
  }

  public void blink(BlinkinPattern pattern, double seconds){
    final double interval = 0.08; //seconds

    Commands.repeatingSequence(
      Commands.runOnce(()-> setPattern(pattern), this),
      new WaitCommand(interval),
      Commands.runOnce(()-> setPattern(BlinkinPattern.BLACK), this),
      new WaitCommand(interval)
    ).withTimeout(seconds).schedule();
  }
}