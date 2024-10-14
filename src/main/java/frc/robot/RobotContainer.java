// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.commands.swervedrive.auto.TurnToSpeaker;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
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
  CommandJoystick driverController = new CommandJoystick(1);
 // CommandJoystick buttonBox = new CommandJoystick(2);

                                                                        
  private final EndEffector outTake = new EndEffector();
  private final Intake intake = new Intake();   
  private final Indexer index = new Indexer();  
  public final BlinkinLights lights = new BlinkinLights();
 
  public final Dragonhead Fafnir = new Dragonhead();
  private Command driveFieldOrientedAnglularVelocity;
  

  /*private final SuperStructure superstructure = new SuperStructure(outTake, Fafnir, intake, index);
  private final AutoMap autoMap = new AutoMap(superstructure, outTake, Fafnir);
*/
                                               
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed



  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
   
    // Configure the trigger bindings
   
    NamedCommands.registerCommand("Intake", index.launch().alongWith(intake.intake()).alongWith(Commands.waitSeconds(2.5)).andThen(intake.stop()).andThen(index.stop()));
    NamedCommands.registerCommand("Outtake", outTake.drop().alongWith(index.drop()).alongWith(intake.reverseIntake()).alongWith(Commands.waitSeconds(.4)).andThen(outTake.stop().alongWith(index.stop()).alongWith(intake.stop())));
    NamedCommands.registerCommand("Shoot", outTake.autoShoot().alongWith(Commands.waitSeconds(.6)).andThen(index.outtake()));
    NamedCommands.registerCommand("Raise Arm To Vision",Fafnir.visionAngle());
    
    // Fafnir.visionAngle());
  
    NamedCommands.registerCommand("Shoot and Raise", outTake.shoot().andThen(Fafnir.visionAngle()).alongWith(Commands.waitSeconds(.3)).andThen(index.outtake()).alongWith(Commands.waitSeconds(.4)).andThen(Fafnir.store()));
    NamedCommands.registerCommand("Raise Arm to Podium", Fafnir.podium());
    NamedCommands.registerCommand("Lower Arm", Fafnir.store());
    NamedCommands.registerCommand("StopAll", outTake.stop().alongWith(index.stop()).alongWith(intake.stop()));
    configureBindings();
     PortForwarder.add(1183, "photonvision.local", 1184);

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

   /*  AbsoluteFieldDrive drive = new AbsoluteFieldDrive(drivebase,       
        () -> MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getZ()*1.15, .1));
*/
        


      
        
      



    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
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
public void checkCommandDSandFMSWork(){
  if(DriverStation.isFMSAttached() || DriverStation.isDSAttached()){
    SmartDashboard.putBoolean("Driver Station or FMS are Connected", true);
  }
  else{
    SmartDashboard.putBoolean("Driver Station or FMS are Connected", false);
  }
}
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

 /*  chooser.addOption(
        "4 Piece Auto",
       new PathPlannerAuto("4 Piece Auto SMR"));*/
  chooser.addOption("Amp Test", new PathPlannerAuto("amp test"));
  chooser.addOption("Taxi", new PathPlannerAuto("Taxi"));
  chooser.addOption("5 Piece Auto", new PathPlannerAuto("Copy of 5 piece Auto"));
  chooser.addOption("2 Piece Top",new PathPlannerAuto("2 piece auto - top"));
  chooser.addOption("3 Piece Auto Far", new PathPlannerAuto("3 piece far bot"));
  chooser.addOption("2 Piece Center", new PathPlannerAuto("2 piece auto"));
  chooser.addOption("3 Piece Auto with Amp", new PathPlannerAuto("3 Piece Auto with Amp"));
  chooser.addOption("4 Piece Auto", new PathPlannerAuto("4 Piece Auto"));
  chooser.addOption("Test for 5 Piece -Do Not Run", new PathPlannerAuto("Test for 5 Piece -Do Not Run"));
  chooser.addOption("Shoot and leave( GOOD LUCK HENRY!! BREAK IT RIDGE!!!)", new PathPlannerAuto("Shoot and back"));
  chooser.addOption("Just Shoot(Manifest 4)", new PathPlannerAuto("just shoot"));
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

   

    driverController.button(1).whileTrue(outTake.shoot().alongWith(Commands.waitSeconds(2.8)).andThen(index.outtake())).onFalse(index.stop().alongWith(outTake.stop()).alongWith(intake.stop()));
    
    // Raise to Vision Angle
    driverController.button(2).whileTrue(Fafnir.visionAngle()).onFalse(Fafnir.store());
    // Intake Command - run intake and indexer motors while the beam break is unbroken
    driverController.button(3).and(indexBeamBreak).whileTrue(Fafnir.setPeakOutput(Constants.DragonheadConstants.dragonPosition.peakOutput).andThen(Fafnir.setArmP(Constants.DragonheadConstants.dragonPosition.P)).andThen(intake.intake()).alongWith(index.launch())).onFalse(intake.stop().alongWith(index.stop()));
     // Amp Command
    driverController.button(1).whileTrue(Fafnir.visionAngle()).onFalse(Fafnir.store());
   driverController.button(4).onTrue(Fafnir.setPeakOutput(Constants.DragonheadConstants.dragonPosition.peakOutput).andThen(Fafnir.setArmP(Constants.DragonheadConstants.dragonPosition.P)).andThen(Fafnir.amp())).onFalse(outTake.ampShoot().andThen(index.outtake()).andThen(Commands.waitSeconds(.5)).andThen(outTake.stop()).andThen(index.stop()).andThen(Fafnir.setArmP(.2)).andThen(Fafnir.setPeakOutput(.2)).andThen(Fafnir.store()).andThen(Fafnir.setPeakOutput(DragonheadConstants.dragonPosition.peakOutput)));
    // Raise to Podium - raises the arm to the podium position and then places it back in the stored position
    driverController.button(8).onTrue(Fafnir.podium()).onFalse(outTake.shoot().alongWith(Commands.waitSeconds(0.8)).andThen(index.outtake()).alongWith(Commands.waitSeconds(1)).andThen(index.stop()).andThen(outTake.stop()).andThen(Fafnir.store()));
  //.alongWith(intake.stop()));

    //climb
    /*driverController.button(14).whileTrue(
    driveFieldOrientedAnglularVelocity = drivebase.turnAndDrive(        
        () -> MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND))).whileFalse(
          driveFieldOrientedAnglularVelocity = drivebase.driveCommand(        
        () -> MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getZ()*.85,.1)));*/
driverController.button(14).whileFalse(
          driveFieldOrientedAnglularVelocity = drivebase.driveCommand(        
        () -> MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getZ()*.85,.1)));

   /*  driverController
    .button(7)
    .whileTrue(
        new TeleopDrive(
            drivebase,
            () ->
                MathUtil.applyDeadband(
                    -driverController.getY() * InputLimits.reduced, InputLimits.vxDeadband),
            () ->
                MathUtil.applyDeadband(
                    -driverController.getX() * InputLimits.reduced, InputLimits.vyDeadband),
            () ->
                MathUtil.applyDeadband(
                    -driverController.getZ() * InputLimits.reduced, InputLimits.angDeadband),
            driverController.button(6).negate(),
            false,
            arm.getCOM()));*/




    driverController.button(6).onTrue(Fafnir.setPeakOutput(Constants.DragonheadConstants.dragonPosition.peakOutput).andThen(Fafnir.setArmP(Constants.DragonheadConstants.dragonPosition.P)).andThen(Fafnir.amp())).onFalse(Fafnir.setArmP(.4).andThen(Fafnir.setPeakOutput(.5)).andThen(Fafnir.store()).alongWith(Commands.waitSeconds(.5)).andThen(Fafnir.setPeakOutput(.8).andThen(Fafnir.setArmP(.7)).andThen(Fafnir.store())));

    driverController.button(7).onTrue(outTake.drop().alongWith(index.drop()).alongWith(intake.reverseIntake())).onFalse(outTake.stop().alongWith(index.stop()).alongWith(intake.stop()));
   // driverController.button(8).whileTrue(intake.intake().alongWith(index.outtake())).onFalse(intake.stop().alongWith(index.stop()));
    //Index Test
    driverController.button(15).onTrue(index.outtake()).onFalse(index.stop());
    //make an amp shoot command eventually
   driverController.button(5).onTrue(Fafnir.store().andThen(Fafnir.setArmP(DragonheadConstants.dragonPosition.P)).andThen(Fafnir.setPeakOutput(DragonheadConstants.dragonPosition.peakOutput)));
//Amp
/* Button Box Commented Out 
    buttonBox.button(1).onTrue(Fafnir.raiseAngle());
    buttonBox.button(2).onTrue(Fafnir.lowerAngle());
    buttonBox.button(4).onTrue(Fafnir.raiseAngleHalf());
    buttonBox.button(5).onTrue(Fafnir.lowerAngleHalf());
*/
 
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
