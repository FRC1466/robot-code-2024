// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DragonheadConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Outtake;
import frc.robot.commands.AutoMap;
import frc.robot.commands.SuperStructure;
import frc.robot.subsystems.Manipulator.BlinkinLights;
import frc.robot.subsystems.Manipulator.Dragonhead;
import frc.robot.subsystems.Manipulator.EndEffector;
import frc.robot.subsystems.Manipulator.Indexer;
import frc.robot.subsystems.Manipulator.Intake;
import frc.robot.subsystems.Manipulator.BlinkinLights.BlinkinLedMode;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

 
  // The robot's subsystems and commands are defined here...
  private double absoluteDistanceFromSpeaker;
  private double podiumRadians;
  private boolean autoControl = false;
  private SendableChooser<Command> chooser = new SendableChooser<>();
   // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
                                                                        
  private final EndEffector outTake = new EndEffector();
  private final Intake intake = new Intake();   
  private final Indexer index = new Indexer();  
  public final BlinkinLights lights = new BlinkinLights();
  CommandJoystick buttonBox = new CommandJoystick(5);
  public final Dragonhead Fafnir = new Dragonhead();
  private Command driveFieldOrientedAnglularVelocity;
  

  /*private final SuperStructure superstructure = new SuperStructure(outTake, Fafnir, intake, index);
  private final AutoMap autoMap = new AutoMap(superstructure, outTake, Fafnir);
*/
                                               
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController = new CommandJoystick(1);


  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
   
    // Configure the trigger bindings
   
    NamedCommands.registerCommand("Intake", index.launch().alongWith(intake.intake()).alongWith(Commands.waitSeconds(1.5)).andThen(intake.stop()).andThen(index.stop()));
    NamedCommands.registerCommand("Shoot", outTake.shoot().alongWith(Commands.waitSeconds(.3).andThen(index.outtake())));
    NamedCommands.registerCommand("Raise Arm", Fafnir.podium());
    NamedCommands.registerCommand("Lower Arm", Fafnir.store());
    NamedCommands.registerCommand("StopAll", outTake.stop().alongWith(index.stop()).alongWith(intake.stop()));
    configureBindings();
     PortForwarder.add(5800, "photonvision.local", 5800);

    initializeChooser();

    
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot


              Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getZ()*1.15, .1)).withName("Default Drive Command");
        if(autoControl){
                    driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getZ()*.05,.1)).withName("Default Drive Command");

        }
        
      



    drivebase.setDefaultCommand(
        driveFieldOrientedAnglularVelocity);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
public Command intakeNote(){
  return intake.intake().alongWith(index.outtake());
}

/* funny voltage set
public Command setVoltage(){
  return runOnce(()-> outTake.setShooterVolts(-(driverController.getRawAxis(3)+1)*4));
}*/

public void stopAll(){
 intake.setVoltage(0);
 index.setRollers(0);
 outTake.setVoltage(0);
}

public boolean beamBreak(){
  return index.getIndexerBeamBreak();
}

public void initializeChooser(){
  chooser.addOption("Taxi", new PathPlannerAuto("Taxi"));

  chooser.addOption(
        "4 Piece Auto",
       new PathPlannerAuto("4 Piece Auto SMR"));
  chooser.addOption("Taxi", new PathPlannerAuto("Taxi"));
  chooser.addOption("2 Piece Top",new PathPlannerAuto("2 piece auto - top"));
  chooser.addOption("2 Piece Center", new PathPlannerAuto("2 piece auto - center"));
  chooser.addOption("Shoot and leave( GOOD LUCK HENRY!! BREAK IT RIDGE!!!)", new PathPlannerAuto("Shoot and back"));
  SmartDashboard.putData("CHOOSE", chooser);
}
public void resetPID(){
  Fafnir.setArmP(Constants.DragonheadConstants.dragonPosition.P);
  Fafnir.setPeakOutput(Constants.DragonheadConstants.dragonPosition.peakOutput);
}



public Command backPID(){
  return runOnce(() -> resetPID());
}



  
  private void configureBindings()
  {
    
    // Creating trigger for beambreak
    Trigger indexBeamBreak = new Trigger(() -> index.getIndexerBeamBreak());
    // Reset Gyro
    driverController.povDown().onTrue(Commands.runOnce(drivebase::zeroGyro));
    // Shoot Command - Runs up shooter falcons, then feeds them the note.
    driverController.button(1).whileTrue(outTake.shoot().alongWith(Commands.waitSeconds(0.5).andThen(index.outtake()))).onFalse(index.stop().alongWith(outTake.stop()).alongWith(intake.stop()));
    // Raise to Vision Angle
    driverController.button(2).whileTrue(Fafnir.visionAngle()).onFalse(Fafnir.store());
    // Intake Command - run intake and indexer motors while the beam break is unbroken
    driverController.button(3).and(indexBeamBreak).whileTrue(intake.intake().alongWith(index.launch())).onFalse(intake.stop().alongWith(index.stop()));
     // Amp Command
    driverController.button(4).onTrue(Fafnir.amp()).onFalse(outTake.ampShoot().andThen(index.outtake()).andThen(Commands.waitSeconds(.5)).andThen(outTake.stop()).andThen(index.stop()).andThen(Fafnir.setArmP(.2)).andThen(Fafnir.setPeakOutput(.2)).andThen(Fafnir.store()).andThen(Fafnir.setArmP(DragonheadConstants.dragonPosition.P)).andThen(Fafnir.setPeakOutput(DragonheadConstants.dragonPosition.peakOutput)));
    // Raise to Podium - raises the arm to the podium position and then places it back in the stored position
    driverController.button(8).onTrue(Fafnir.podium()).onFalse(Fafnir.store());

    //climb

    buttonBox.button(2).onTrue(runOnce(()-> lights.setMode(BlinkinLedMode.FIXED_RAINBOW_PARTY)));
    buttonBox.button(1).onTrue(runOnce(()-> lights.setMode(BlinkinLedMode.TWO_CHASE)));
    buttonBox.button(3).onTrue(runOnce(()-> lights.setMode(BlinkinLedMode.SOLID_WHITE)));

    driverController.button(6).onTrue(Fafnir.amp()).onFalse(Fafnir.setArmP(.4).andThen(Fafnir.setPeakOutput(.5)).andThen(Fafnir.store()).alongWith(Commands.waitSeconds(.5)).andThen(Fafnir.setPeakOutput(.8).andThen(Fafnir.setArmP(.7)).andThen(Fafnir.store())));

    driverController.button(7).onTrue(outTake.drop().alongWith(index.drop()).alongWith(intake.reverseIntake())).onFalse(outTake.stop().alongWith(index.stop()).alongWith(intake.stop()));
   // driverController.button(8).whileTrue(intake.intake().alongWith(index.outtake())).onFalse(intake.stop().alongWith(index.stop()));

    //make an amp shoot command eventually
   driverController.button(5).onTrue(Fafnir.store().andThen(Fafnir.setArmP(DragonheadConstants.dragonPosition.P)).andThen(Fafnir.setPeakOutput(DragonheadConstants.dragonPosition.peakOutput)));
//Amp

    
 
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }




  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */




  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public Command getAuto(){
    return chooser.getSelected();
  }

}
