// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.commands.AutoMap;
import frc.robot.commands.SuperStructure;
import frc.robot.subsystems.Manipulator.Dragonhead;
import frc.robot.subsystems.Manipulator.EndEffector;
import frc.robot.subsystems.Manipulator.Indexer;
import frc.robot.subsystems.Manipulator.Intake;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
                                                                        
  private final EndEffector outTake = new EndEffector();
  private final Intake intake = new Intake();   
  private final Indexer index = new Indexer();  
  private final Dragonhead Fafnir = new Dragonhead();  

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
    NamedCommands.registerCommand("Intake", index.launch().alongWith(intake.intake()).alongWith(Commands.waitSeconds(2)).andThen(intake.stop()).andThen(index.stop()));
    NamedCommands.registerCommand("Shoot", outTake.shoot().alongWith(Commands.waitSeconds(.5).andThen(index.outtake())));
    
    NamedCommands.registerCommand("StopAll", outTake.stop().alongWith(index.stop()).alongWith(intake.stop()));
    configureBindings();

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
public void stopAll(){
 intake.setVoltage(0);
 index.setRollers(0);
 outTake.setVoltage(0);
}
public boolean beamBreak(){
  return index.getIndexerBeamBreak();
}
  
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    Trigger indexBeamBreak = new Trigger(() -> index.getIndexerBeamBreak());
    driverController.povDown().onTrue(Commands.runOnce(drivebase::zeroGyro));
    
    driverController.button(1).whileTrue(outTake.shoot().alongWith(Commands.waitSeconds(0.5).andThen(index.outtake()))).onFalse(index.stop().alongWith(outTake.stop()).alongWith(intake.stop()));
    
    //driverController.button(1).onTrue(outTake.shoot()).onFalse(outTake.stop());`
    //intake
    // 
    driverController.button(3).and(indexBeamBreak).whileTrue(intake.intake().alongWith(index.outtake())).onFalse(intake.stop().alongWith(index.stop()));


    //climb
    driverController.button(6).onTrue(Fafnir.amp()).onFalse(Fafnir.setArmP(.25).andThen(Fafnir.setPeakOutput(.9)).andThen(Fafnir.podium()));
    //.andThen(Fafnir.podium()));
    driverController.button(7).onTrue(outTake.drop().alongWith(index.drop()).alongWith(intake.reverseIntake())).onFalse(outTake.stop().alongWith(index.stop()).alongWith(intake.stop()));
    driverController.button(8).whileTrue(intake.intake().alongWith(index.outtake())).onFalse(intake.stop().alongWith(index.stop()));
    //driverController.button(9).onTrue(index.drop()).onFalse(index.stop());
    //driverController.button(10).onTrue(index.stop());
    //driverController.button(13).onTrue(intake.stop());
    //driverController.button(12).onTrue(intake.intake()).onFalse(intake.stop());
    //driverController.button(11).onTrue(intake.reverseIntake()).onFalse(intake.stop());
   // driverController.button(3).onTrue(Fafnir.podium()).onFalse(Fafnir.setArmP(.2).andThen(Fafnir.store()).andThen(Fafnir.setArmP(DragonheadConstants.dragonPosition.P)));
    //make an amp shoot command eventually
    driverController.button(4).onTrue(Fafnir.amp()).onFalse(outTake.shoot().andThen(index.outtake()).andThen(Commands.waitSeconds(.25)).andThen(outTake.stop()).andThen(index.stop()).andThen(Fafnir.setArmP(.2)).andThen(Fafnir.store()).andThen(Fafnir.setArmP(DragonheadConstants.dragonPosition.P)).andThen(Fafnir.setPeakOutput(DragonheadConstants.dragonPosition.peakOutput)));

    
 
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

 public Command getAutonomousCommand(){

  return new PathPlannerAuto("4 Piece Auto");
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

}
