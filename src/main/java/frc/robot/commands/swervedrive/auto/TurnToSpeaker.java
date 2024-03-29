package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;


public class TurnToSpeaker extends Command {
  private final SwerveSubsystem drive;
  private final PIDController controller;
  private final CommandJoystick driverController;
  private static double kP = 0;
  private static double kI = 0;
  private static double kD = 0;
  private static double minVelocity = 0;
  private static double toleranceDegrees = 0;
  private DriverStation.Alliance alliance = null;

  /** Creates a new TurnToAngle. Turns to the specified rotation. */
  public TurnToSpeaker(SwerveSubsystem drive, CommandJoystick driverController) {
    addRequirements(drive);
    this.drive = drive;
    this.driverController = driverController;


        kP = 0.1;
        kI = 0.0;
        kD = 0.001;
        minVelocity = 0.01;
        toleranceDegrees = 1.5;

    controller = new PIDController(kP, kI, kD, 0.02);
    controller.setTolerance(toleranceDegrees);
    controller.enableContinuousInput(-180, 180);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset();

    if (DriverStation.getAlliance().isPresent()) {
      this.alliance = DriverStation.getAlliance().get();
    }

    if (alliance == DriverStation.Alliance.Red) {
      controller.setSetpoint(
          (new Rotation2d(
                 drive.getPose().getX()-16.28,
                  drive.getPose().getY()-5.55))
              .getDegrees());
    } else {
      controller.setSetpoint(
          (new Rotation2d(drive.getPose().getX() , drive.getPose().getY()-5.55)
              .getDegrees()));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Update output speeds

    double angularSpeed = MathUtil.clamp(controller.calculate(drive.getGyroYawDegrees()), -1, 1);

    if (Math.abs(angularSpeed) < minVelocity) {
      angularSpeed = Math.copySign(minVelocity, angularSpeed);
    }
    drive.setAngularSpeed(angularSpeed);
    drive.turnAndDrive( () -> MathUtil.applyDeadband(driverController.getY(), Constants.OperatorConstants.LEFT_Y_DEADBAND),
    () -> MathUtil.applyDeadband(driverController.getX(), Constants.OperatorConstants.LEFT_X_DEADBAND));
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}