package frc.robot.common.subsystems.drive;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.components.EasyMotor;
import lombok.AccessLevel;
import lombok.Getter;
import lombok.NoArgsConstructor;

import java.util.function.DoubleSupplier;

/**
 * Drive Subsystem for Tank Drive bots with 2 motors for each tread
 * This is unfinished
 *
 * @author Hudson Strub
 * @since 2025
 */
@NoArgsConstructor(access = AccessLevel.PUBLIC)
public class TankDriveSubsystem extends SubsystemBase {
    @Getter SparkMax leftMotor;
    @Getter SparkMax rightMotor;

    public TankDriveSubsystem(int leftMotorId, int rightMotorId) {
        leftMotor = EasyMotor.createEasySparkMax(leftMotorId, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast);
        rightMotor = EasyMotor.createEasySparkMax(rightMotorId, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast);

    }

    /**
     * Control both motors of the tank drive
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        leftMotor.set(leftSpeed);
        rightMotor.set(rightSpeed);
    }

    /**
     * Like that one Wii game 
     */
    public void arcadeDrive(double forward, double rotation) {
        double leftSpeed = forward + rotation;
        double rightSpeed = forward - rotation;
        tankDrive(leftSpeed, rightSpeed);
    }

    public Command arcadeDriveCommand(DoubleSupplier forward, DoubleSupplier rotation) {
        return Commands.run(
                () -> arcadeDrive(-forward.getAsDouble(), rotation.getAsDouble()), // negative if forward stick is inverted
                this
        );
    }
}
