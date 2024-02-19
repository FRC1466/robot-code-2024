package frc.robot.subsystems.Manipulator;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DragonheadConstants;
import webblib.math.ArmPIDController;

public class Dragonhead extends SubsystemBase{

  private TalonFX armMotorRight, armMotorLeft;
  private DutyCycleEncoder absoluteArmEncoder;
  private double peakOutput;
  private ArmPIDController armPID;
  private Rotation2d localSetpoint;
  private DoubleSupplier overrideFeedforward = () -> 0.0;
  private boolean disabled = false;
  private Rotation2d storedPosRad = Rotation2d.fromRadians(DragonheadConstants.restRadians);
  private boolean storedInPerimeter = false;

  /** Create a new VirtualFourBar subsystem. */
  public Dragonhead() {
    armMotorRight = new TalonFX(DragonheadConstants.rightArmPort);
    armMotorLeft = new TalonFX(DragonheadConstants.leftArmPort);
    
    peakOutput = DragonheadConstants.dragonPosition.peakOutput;
    absoluteArmEncoder = new DutyCycleEncoder(DragonheadConstants.dutyCyclePort);
    absoluteArmEncoder.setDutyCycleRange(0, 1);
    absoluteArmEncoder.setDistancePerRotation(1.0);

    armPID =
        new ArmPIDController(
            DragonheadConstants.dragonPosition.P,DragonheadConstants.dragonPosition.I, DragonheadConstants.dragonPosition.D);
    armPID.setAvoidanceRange(
        Rotation2d.fromRadians(DragonheadConstants.restRadians),
        Rotation2d.fromRadians(DragonheadConstants.maxRadians));
    armPID.setTolerance(0.15);

  /*   if (Robot.isSimulation()) {
      sim = new VirtualFourBarSimulation(absoluteArmEncoder);
      SmartDashboard.putData("Arm Sim", sim.getMech2d());
    }*/

  setGoal(Rotation2d.fromRadians(DragonheadConstants.restRadians));
  setDefaultCommand(hold());
  //armMotor.setNeuMode(NeutralModeValue.Brake);
  }
  public Command setArmP(double p){
    return runOnce(() ->armPID.setP(p));
  }
    public Command setPeakOutput(double output){
    return runOnce(() -> peakOutput = output);
  }
  @Override
  public void simulationPeriodic() {
    //sim.update(armMotor.get());
  }

  /** Configure arm motor. */

  /**
   * Gets absolute position of the arm as a Rotation2d. Assumes the arm being level within frame is
   * the 0 point on the x axis. Assumes CCW+.
   *
   * @return current angle of arm
   */
  private Rotation2d getShiftedAbsoluteDistance() {
    var initialPosition =
        absoluteArmEncoder.getAbsolutePosition() / DragonheadConstants.dutyCycleResolution;
     return Rotation2d.fromRotations(initialPosition)
        .minus(Rotation2d.fromRotations(DragonheadConstants.absolutePositionOffset));
  }

  /**
   * Gets position of arm in radians. Assumes the arm being level within frame is the 0 point on the
   * x axis. Assumes CCW+.
   *
   * @return position in rad.
   */
  public Rotation2d getPosition() {
    return DragonheadConstants.encoderInverted
        ? getShiftedAbsoluteDistance().unaryMinus()
        : getShiftedAbsoluteDistance();
  }

  /**
   * Set arm with PID.
   *
   * @param setpoint setpoint in radians.
   */
  public void setGoal(Rotation2d setpoint) {
    localSetpoint = setpoint;
    armPID.setSetpoint(setpoint);
    SmartDashboard.putNumber("Arm PID Setpoint", setpoint.getRadians());
  }

  public void setArmHold() {
    var motorOutput =
        MathUtil.clamp(
            armPID.calculate(getPosition(), localSetpoint),
            -peakOutput,
            peakOutput);
    var feedforward = getPosition().getCos() * DragonheadConstants.gravityFF;
    setMotor(motorOutput + feedforward + overrideFeedforward.getAsDouble());

    SmartDashboard.putNumber("Arm PID Output", motorOutput);
    SmartDashboard.putNumber("Arm Feedforward", feedforward);
    SmartDashboard.putNumber("Arm Feedforward Override", overrideFeedforward.getAsDouble());
  }

  public void setMotor(double percent) {
    armMotorRight.set(percent);
    armMotorLeft.set(-percent);
  }

  public void setFeedforward(DoubleSupplier ff) {
    System.out.println("ff: " + ff.toString());
    overrideFeedforward = ff;
  }

  public Command podium() {
    return runOnce(() -> setGoal(Rotation2d.fromRadians(DragonheadConstants.podiumRadians)))
       .andThen(holdUntilSetpoint());
  }

  public Command amp() {
    return runOnce(() -> setGoal(Rotation2d.fromRadians(DragonheadConstants.ampRadians)))
      .andThen(holdUntilSetpoint());
  }


  public Command store() {
    return runOnce(() -> setGoal(storedPosRad))
       .andThen(holdUntilSetpoint());
  }

  public void setStoreSetpoint() {
    if (storedInPerimeter) {
      storedPosRad = Rotation2d.fromRadians(DragonheadConstants.restRadians);
    } else {
      storedPosRad = Rotation2d.fromRadians(DragonheadConstants.restRadians);
    }
    System.out.println("Override changed.");
    SmartDashboard.putBoolean("In Frame Perimeter", storedInPerimeter);
    storedInPerimeter = !storedInPerimeter;
  }


  public Command hold() {
    return Commands.run(this::setArmHold, this);
  }

  public Command holdUntilSetpoint() {
    return hold()
        .raceWith(Commands.waitSeconds(0.3).andThen(Commands.waitUntil(this::isAtSetpoint)));
  }

  public Command toggleDisable() {
    return runOnce(
        () -> {
          disabled = !disabled;
        });
  }

  /**
   * If the arm is at setpoint.
   *
   * @return if arm is at setpoint.
   */
  public boolean isAtSetpoint() {
    SmartDashboard.putBoolean("Arm PID at setpoint", armPID.atSetpoint());
    return armPID.atSetpoint();
  }

  public Supplier<Translation3d> getCOM() {
    var rest = new Translation2d(Constants.ARM_LENGTH / 2, 0);
    var rotatedRest = rest.rotateBy(getPosition());
    return () ->
        Constants.INITIAL_ARM_MOUNT.plus(
            new Translation3d(-rotatedRest.getX(), 0, rotatedRest.getY()));
  }


  @Override
  public void periodic() {
   setArmHold();

    SmartDashboard.putData(absoluteArmEncoder);
    SmartDashboard.putNumber("Arm Raw Absolute Encoder", absoluteArmEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Arm Processed Absolute Encoder", getPosition().getRadians());
    SmartDashboard.putNumber("Get Shifted Absolute Position", getShiftedAbsoluteDistance().getRadians());
    SmartDashboard.putNumber("Arm PID error", armPID.getPositionError());
    SmartDashboard.putString("Arm COM", getCOM().get().toString());
    SmartDashboard.putBoolean("Arm Disabled", disabled);
}

}

