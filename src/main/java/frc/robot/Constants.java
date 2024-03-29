// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
      public static final Translation3d cameraTranslation = new Translation3d(0.28, 0.0, 0.23);
    public static final Rotation3d cameraRotation = new Rotation3d(0, Math.toRadians(-15), 0);
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
  public static final class VisionConstants{
      public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(.2, .2, .3);
      public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(.1, .1, .2);
  }

  public static final class PoseEstimator {
    /** THANK YOU IRON PANTHERS */
    public static final double NOISY_DISTANCE_METERS = 2.5;

    /**
     * The number to multiply by the smallest of the distance minus the above constant, clamped
     * above 1 to be the numerator of the fraction.
     */
    public static final double DISTANCE_WEIGHT = 7;

    /**
     * The number to multiply by the number of tags beyond the first to get the denominator of the
     * deviations matrix.
     */
    public static final double TAG_PRESENCE_WEIGHT = 10;

    /** The amount to shift the pose ambiguity by before multiplying it. */
    public static final double POSE_AMBIGUITY_SHIFTER = .2;

    /** The amount to multiply the pose ambiguity by if there is only one tag. */
    public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
  }
  public static final class DragonheadConstants{
      public static final int rightArmPort = 19, leftArmPort = 20,
      dutyCyclePort = 1;
      public static final Gains dragonPosition = new Gains(.50,0.0035,0.006,0,0,0.7);
      public static final double restRadians = .0;
      public static final double ampRadians = (7*Math.PI/12)+.3;
      public static final double maxRadians = (7*Math.PI/12)+.7;
      public static final double podiumRadians = .485;//not podium, just
      public static final double gravityFF = 0.02;
      public static final double absolutePositionOffset = -.1565;
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

