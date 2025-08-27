// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.common.swerve;

import java.time.Duration;
import java.time.Instant;
import java.util.concurrent.ThreadLocalRandom;

import org.lasarobotics.drive.swerve.AdvancedSwerveKinematics.ControlCentricity;
import org.lasarobotics.hardware.IMU;
import org.lasarobotics.hardware.LoggableHardware;
import org.lasarobotics.hardware.PurpleManager;
import org.lasarobotics.hardware.kauailabs.NavX2InputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearAcceleration;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

/** NavX2 */
@SuppressWarnings("unused")
public class RAWRNavX2 extends LoggableHardware implements IMU {
  /** NavX2 ID */
  public static class ID {
    public final String name;

    /**
     * NavX2 ID
     * @param name Device name for logging
     */
    public ID(String name) {
      this.name = name;
    }
  }

  /**
   * NavX sensor inputs
   */
  @AutoLog
  public static class NavX2Inputs {
    public boolean isConnected = false;
    public MutAngle rollAngle = Units.Radians.zero().mutableCopy();
    public MutAngle pitchAngle = Units.Radians.zero().mutableCopy();
    public MutAngle yawAngle = Units.Radians.zero().mutableCopy();
    public MutLinearAcceleration accelerationX = Units.MetersPerSecondPerSecond.zero().mutableCopy();
    public MutLinearAcceleration accelerationY = Units.MetersPerSecondPerSecond.zero().mutableCopy();
    public MutLinearAcceleration accelerationZ = Units.MetersPerSecondPerSecond.zero().mutableCopy();
    public MutLinearVelocity velocityX = Units.MetersPerSecond.zero().mutableCopy();
    public MutLinearVelocity velocityY = Units.MetersPerSecond.zero().mutableCopy();
    public MutLinearVelocity velocityZ = Units.MetersPerSecond.zero().mutableCopy();
    public MutAngularVelocity yawRate = Units.RadiansPerSecond.zero().mutableCopy();
    public Rotation2d rotation2d = Rotation2d.kZero;
  }

  private static final AngularVelocity NAVX2_YAW_DRIFT_RATE = Units.DegreesPerSecond.of(0.5 / 60);

  private final AHRS navx;
  private ChassisSpeeds previousSpeeds;
  private Instant lastUpdateTime;

  private final SimDouble simPitch;
  private final SimDouble simRoll;
  private final SimDouble simYaw;
  private final SimDouble simAccelX;
  private final SimDouble simAccelY;

  private final String name;
  private final NavX2InputsAutoLogged inputs;

  private final boolean fieldCentricVelocities;

  /**
   * Create a NavX2 object with built-in logging
   * @param id NavX2 ID
   */
  public RAWRNavX2(ID id) {
    this.name = id.name;
    this.navx = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k200Hz);
    this.inputs = new NavX2InputsAutoLogged();
    this.fieldCentricVelocities = false;

    SimDeviceSim simNavX2 = new SimDeviceSim("navX-Sensor", navx.getPort());
    this.simPitch = simNavX2.getDouble("Pitch");
    this.simRoll = simNavX2.getDouble("Roll");
    this.simYaw = simNavX2.getDouble("Yaw");
    this.simAccelX = simNavX2.getDouble("LinearWorldAccelX");
    this.simAccelY = simNavX2.getDouble("LinearWorldAccelY");
      SimDouble simAccelZ = simNavX2.getDouble("LinearWorldAccelZ");
    this.previousSpeeds = new ChassisSpeeds();
    this.lastUpdateTime = Instant.now();

    // Update inputs on init
    updateInputs();

    // Register device with manager
    PurpleManager.add(this);
  }

  /**
   * Get NavX port number
   * @return Port number
   */
  int getPort() {
    return navx.getPort();
  }

  /**
   * Update NavX input readings
   */
  @Override
  public void updateInputs() {
    synchronized (inputs) {
      inputs.isConnected = navx.isConnected();
      inputs.rollAngle.mut_replace(navx.getRoll(), Units.Degrees);
      inputs.pitchAngle.mut_replace(navx.getPitch(), Units.Degrees);
      inputs.yawAngle.mut_replace(navx.getAngle(), Units.Degrees);
      inputs.accelerationX.mut_replace(navx.getWorldLinearAccelX(), Units.Gs);
      inputs.accelerationY.mut_replace(navx.getWorldLinearAccelY(), Units.Gs);
      inputs.accelerationZ.mut_replace(navx.getWorldLinearAccelZ(), Units.Gs);
      inputs.velocityX.mut_replace(fieldCentricVelocities ? navx.getVelocityX() : navx.getRobotCentricVelocityX(), Units.MetersPerSecond);
      inputs.velocityY.mut_replace(fieldCentricVelocities ? navx.getVelocityY() : navx.getRobotCentricVelocityY(), Units.MetersPerSecond);
      inputs.velocityZ.mut_replace(fieldCentricVelocities ? navx.getVelocityZ() : navx.getRobotCentricVelocityZ(), Units.MetersPerSecond);
      inputs.yawRate.mut_replace(navx.getRate(), Units.DegreesPerSecond);
      inputs.rotation2d = Rotation2d.fromRadians(inputs.yawAngle.times(-1).in(Units.Radians));
    }
  }

  /**
   * Call this method periodically
   */
  @Override
  protected void periodic() {
    synchronized (inputs) { Logger.processInputs(name, inputs); }
  }

  @Override
  public Frequency getUpdateRate() {
    return Units.Hertz.of(1.0 / NavXUpdateRate.k200Hz.getValue());
  }

  /**
   * Get latest sensor input data
   * @return Latest NavX data
   */
  @Override
  public NavX2InputsAutoLogged getInputs() {
    synchronized (inputs) { return inputs; }
  }

  /**
   * Call this to configure swapable axes for X/Y or to invert an axis. Currently, this will also swap/invert
   * the robot centic values.
   * @param swapAxes Will swap X/Y Axis
   * @param invertX Will invert X
   * @param invertY Will invert Y
   * @param invertZ Will invert Z
   */
  public void configureVelocity(boolean swapAxes, boolean invertX, boolean invertY, boolean invertZ) {
    navx.configureVelocity(swapAxes, invertX, invertY, invertZ);
  }

  /**
   * Zeros the displacement integration variables.   Invoke this at the moment when
   * integration begins.
   */
  public void resetDisplacement() {
    navx.resetDisplacement();
  }

  /**
   * Returns true if the sensor is currently performing automatic
   * gyro/accelerometer calibration. Automatic calibration occurs when the
   * sensor is initially powered on, during which time the sensor should be
   * held still, with the Z-axis pointing up (perpendicular to the earth).
   * <p>
   * NOTE: During this automatic calibration, the yaw, pitch and roll values
   * returned may not be accurate.
   * <p>
   * Once calibration is complete, the sensor will automatically remove an
   * internal yaw offset value from all reported values.
   * @return Returns true if the sensor is currently automatically calibrating the gyro
   */
  public boolean isCalibrating() {
    return navx.isCalibrating();
  }

  @Override
  public boolean isConnected() {
    synchronized (inputs) { return inputs.isConnected; }
  }

  @Override
  public void reset() {
    navx.reset();
  }

  @Override
  public Angle getRoll() {
    synchronized (inputs) { return inputs.rollAngle; }
  }

  @Override
  public Angle getPitch() {
    synchronized (inputs) { return inputs.pitchAngle; }
  }

  @Override
  public Angle getYaw() {
    synchronized (inputs) { return inputs.yawAngle; }
  }

  @Override
  public AngularVelocity getYawRate() {
    synchronized (inputs) { return inputs.yawRate; }
  }

  @Override
  public Rotation2d getRotation2d() {
    synchronized (inputs) { return inputs.rotation2d; }
  }

  @Override
  public LinearVelocity getVelocityX() {
    synchronized (inputs) { return inputs.velocityX; }
  }

  @Override
  public LinearVelocity getVelocityY() {
    synchronized (inputs) { return inputs.velocityY; }
  }

  @Override
  public void updateSim(Rotation2d orientation, ChassisSpeeds desiredSpeeds, ControlCentricity controlCentricity) {
    var currentTime = Instant.now();
    double randomNoise = ThreadLocalRandom.current().nextDouble(0.9, 1.0);
    double dt = Duration.between(currentTime, lastUpdateTime).toMillis() / 1000.0;

    if (controlCentricity.equals(ControlCentricity.FIELD_CENTRIC))
      desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, orientation);

    inputs.velocityX.mut_replace(desiredSpeeds.vxMetersPerSecond, Units.MetersPerSecond);
    inputs.velocityY.mut_replace(desiredSpeeds.vyMetersPerSecond, Units.MetersPerSecond);

    int yawDriftDirection = ThreadLocalRandom.current().nextDouble(1.0) < 0.5 ? -1 : +1;
    double angle = simYaw.get() + Math.toDegrees(desiredSpeeds.omegaRadiansPerSecond * randomNoise) * dt
                   + (NAVX2_YAW_DRIFT_RATE.in(Units.DegreesPerSecond) * dt * yawDriftDirection);
    simYaw.set(angle);

    simAccelX.set((desiredSpeeds.vxMetersPerSecond - previousSpeeds.vxMetersPerSecond) / dt);
    simAccelY.set((desiredSpeeds.vyMetersPerSecond - previousSpeeds.vyMetersPerSecond) / dt);

    previousSpeeds = desiredSpeeds;
    lastUpdateTime = currentTime;
  }

  /**
   * Closes the NavX
   */
  @Override
  public void close() {
    PurpleManager.remove(this);
    navx.close();
  }
}