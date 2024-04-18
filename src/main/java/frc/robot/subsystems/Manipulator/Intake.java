package frc.robot.subsystems.Manipulator;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Outtake;

public class Intake extends SubsystemBase {
  private final TalonFX intakeMotor;
  private final LinearFilter currentFilter = LinearFilter.movingAverage(10);
  private double filteredCurrent;
  private double intakeVolts = 3;


  /** Create a new Gripper subsystem. */
 public Intake() {
    intakeMotor = new TalonFX(16);

  }

  public void setVoltage(double outputVoltage){
   intakeMotor.setVoltage(outputVoltage);
  }
  public Command stop() {
    return runOnce(() -> setVoltage(0)); 
  }
  public Command reverseIntake() {
    return runOnce(() -> setVoltage(2));
  }

  public Command intake() {
    return runOnce(() -> setVoltage(-intakeVolts));
  }



  public double getCurrent(TalonFX intakeMotor) {
    return (intakeMotor.getSupplyCurrent()).getValueAsDouble();
  }
  public void periodic() {
    filteredCurrent = currentFilter.calculate(getCurrent(intakeMotor));
  }

} 