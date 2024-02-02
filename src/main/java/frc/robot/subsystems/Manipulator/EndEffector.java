/*package frc.robot.subsystems.Manipulator;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
  private final TalonFX gripperMotor;
  private final LinearFilter currentFilter = LinearFilter.movingAverage(10);
  private double filteredCurrent;

  /** Create a new Gripper subsystem. */
 /*public EndEffector() {
    gripperMotor = new TalonFX(Intake.motorID, "rio");
    gripperMotor.enableVoltageCompensation(12.0);
    gripperMotor.setSmartCurrentLimit(25);
    gripperMotor.burnFlash();
  }

} */