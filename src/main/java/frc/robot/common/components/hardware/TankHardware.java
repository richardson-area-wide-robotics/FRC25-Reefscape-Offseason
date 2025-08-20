package frc.robot.common.components.hardware;

import com.revrobotics.spark.SparkBase;
import frc.robot.common.swerve.RAWRNavX2;


public record TankHardware(RAWRNavX2 navx, SparkBase lMotor, SparkBase rMotor) {

    public void stop() {
        lMotor.set(0.0);
        rMotor.set(0.0);
    }

    public void close() {
        navx.close();
        lMotor.close();
        rMotor.close();
    }
}