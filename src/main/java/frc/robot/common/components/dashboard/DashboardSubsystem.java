package frc.robot.common.components.dashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.components.NamedAutoRegistry;

/**
 * A SubsystemBase that automatically registers its instance
 * for @DashboardVariable tracking.
 *
 * @author Hudson Strub
 * @since 2025 Offseason
 */
public abstract class DashboardSubsystem extends SubsystemBase {

    public DashboardSubsystem() {
        super();
        NamedAutoRegistry.register(this);
        DashboardAutoUpdater.register(this);
    }

    public DashboardSubsystem(String name) {
        super(name);
        DashboardAutoUpdater.register(this);
    }
}
