// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Manipulator.Dragonhead;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;
import webblib.util.Gains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double ARM_LENGTH = 0.58;
  public static final Translation3d INITIAL_ARM_MOUNT = new Translation3d(0.3, 0, 0.7);

  public static final class AutonConstants
  {

    public static final PIDConstants TranslationPID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants angleAutoPID   = new PIDConstants(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class IndexerConstants{

    public static final int IndexerBeamBreak = 0;
  }
  public static final class DragonheadConstants{
      public static final int rightArmPort = 19, leftArmPort = 20,
      dutyCyclePort = 1;
      public static final Gains dragonPosition = new Gains(.33,0.002,0.002,0,0,0.38);
      public static final double restRadians = Math.PI/48;
      public static final double ampRadians = 7*Math.PI/12;
      public static final double maxRadians = 2*Math.PI/3;
      public static final double podiumRadians = Math.PI/9;
      public static final double gravityFF = 0.02;
      public static final double absolutePositionOffset = -1.653;
      public static final boolean encoderInverted = false;
      public static final double dutyCycleResolution = 1.0;
      public static final class DragonConfig {
      public static final CurrentLimitsConfigs supplyCurrent;

      public static final TalonFXConfiguration motorConfig;

      static {
        supplyCurrent = new CurrentLimitsConfigs();
    
    

        motorConfig = new TalonFXConfiguration();
        motorConfig.CurrentLimits = supplyCurrent;

    
      }
      
    }
      
  }
  
  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  public static class Outtake{
    public static final int SHOOTER_ID_LEFT = 14;
    public static final int SHOOTER_ID_RIGHT = 15;
  }
}

