// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/**
 * The CommonConstants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class CommonConstants {

  /**
   * CommonConstants for controllers
   * <br>
   * {@link HIDConstants#PRIMARY_CONTROLLER_PORT} is for the driver,
   * <br> <br>
   * {@link HIDConstants#SECONDARY_CONTROLLER_PORT} is for the operator
   */
  public static class HIDConstants {

  public static final int PRIMARY_CONTROLLER_PORT = 0;
  public static final int SECONDARY_CONTROLLER_PORT = 1;
  public static final double CONTROLLER_DEADBAND = 0.6;

  public static final CommandXboxController DRIVER_CONTROLLER = new CommandXboxController(
    PRIMARY_CONTROLLER_PORT);
  public static final CommandXboxController OPERATOR_CONTROLLER = new CommandXboxController(
    SECONDARY_CONTROLLER_PORT);

  }

  public static class SmartDashboardConstants {
    public static final String SMARTDASHBOARD_AUTO_MODE = "Auto Mode";
  }

  public static class LogConstants {
    public static final String POSE_LOG_ENTRY = "/Pose";
    public static final String ACTUAL_SWERVE_STATE_LOG_ENTRY = "/ActualSwerveState";
    public static final String DESIRED_SWERVE_STATE_LOG_ENTRY = "/DesiredSwerveState";
    public static final String ROTATE_ERROR_LOG_ENTRY = "/RotateError";
    public static final String MAX_LINEAR_VELOCITY_LOG_ENTRY = "/MaxLinearVelocity";

  }

}