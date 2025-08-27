package frc.robot.common.components.hardware;


import frc.robot.common.gryo.RAWRNavX2;
import frc.robot.common.swerve.RAWRSwerveModule;


/**
 * Drive hardware for a robot with swerve drive
 *
 * @author PurpleLib
 * @author Alan Trinh
 * @author Hudson Strub
 *
 * @since 2025
 */
public record SwerveHardware(RAWRNavX2 navx, RAWRSwerveModule lFrontModule, RAWRSwerveModule rFrontModule,
                             RAWRSwerveModule lRearModule, RAWRSwerveModule rRearModule) {
    public void lock() {
        lFrontModule.lock();
        rFrontModule.lock();
        lRearModule.lock();
        rRearModule.lock();
    }

    public void stop() {
        lFrontModule.stop();
        rFrontModule.stop();
        lRearModule.stop();
        rRearModule.stop();
    }

    public void toggleTractionControl() {
        lFrontModule.toggleTractionControl();
        rFrontModule.toggleTractionControl();
        lRearModule.toggleTractionControl();
        rRearModule.toggleTractionControl();
    }

    public void enableTractionControl() {
        lFrontModule.enableTractionControl();
        rFrontModule.enableTractionControl();
        lRearModule.enableTractionControl();
        rRearModule.enableTractionControl();
    }

    public void disableTractionControl() {
        lFrontModule.disableTractionControl();
        rFrontModule.disableTractionControl();
        lRearModule.disableTractionControl();
        rRearModule.disableTractionControl();
    }

    public void close() {
        navx.close();
        lFrontModule.close();
        rFrontModule.close();
        lRearModule.close();
        rRearModule.close();
    }
}