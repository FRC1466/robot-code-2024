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

public class EndEffector extends SubsystemBase {
  private final TalonFX shooterMotorLeft;
  private final TalonFX shooterMotorRight;
  private final LinearFilter currentFilter = LinearFilter.movingAverage(10);
  private double filteredCurrentLeft;
  public double filteredCurrentRight;



  /** Create a new Gripper subsystem. */
 public EndEffector() {
    shooterMotorLeft = new TalonFX(Outtake.SHOOTER_ID_LEFT);
    shooterMotorRight = new TalonFX(Outtake.SHOOTER_ID_RIGHT);

  }

  public void setVoltage(double outputVoltage){
   shooterMotorLeft.setVoltage(outputVoltage);
   shooterMotorRight.setVoltage(outputVoltage);
  }
  public Command stop() {
    return runOnce(() -> setVoltage(0)); 
  }
  public Command drop() {
    return runOnce(() -> setVoltage(1));
  }

  public Command shoot() {
    return runOnce(() -> setVoltage(-8));
  }

  public Command intake() {
    return runOnce(() -> setVoltage(-3));
  }

  public Command ampShoot() {
    return runOnce (() -> setVoltage(-2));
  }

  public double getCurrent(TalonFX shooterMotor) {
    return (shooterMotor.getSupplyCurrent()).getValueAsDouble();
  }
  public void periodic() {
    filteredCurrentLeft = currentFilter.calculate(getCurrent(shooterMotorLeft));
    filteredCurrentRight = currentFilter.calculate(getCurrent(shooterMotorRight));
  }

} 