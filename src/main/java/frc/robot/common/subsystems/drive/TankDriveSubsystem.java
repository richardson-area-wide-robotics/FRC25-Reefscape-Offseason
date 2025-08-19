package frc.robot.common.subsystems.drive;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.common.components.EasyMotor;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;

/**
 * Drive Subsystem for Tank Drive bots with 2 motors for each tread
 * This is unfinished, and is for the T-shirt bot in offseason
 *
 * @author Hudson Strub
 * @since 2025
 */
@NoArgsConstructor(access = AccessLevel.PUBLIC)
public class TankDriveSubsystem {
    SparkMax leftMotor;
    SparkMax rightMotor;

    public TankDriveSubsystem(int leftMotorId, int rightMotorId) {
        System.out.println("LEft MOTRER:" +leftMotorId);
        leftMotor = EasyMotor.createEasySparkMax(leftMotorId, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast);
        rightMotor = EasyMotor.createEasySparkMax(rightMotorId, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast);

    }

    public void setMotorSpeed(double speed) {
         leftMotor.set(speed);
    }

    public SparkMax getLeftMotor() {
        return leftMotor;
    }
}
