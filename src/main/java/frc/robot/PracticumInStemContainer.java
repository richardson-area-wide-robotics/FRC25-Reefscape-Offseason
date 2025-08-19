package frc.robot;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;
import frc.robot.common.interfaces.IRobotContainer;
import frc.robot.common.subsystems.drive.TankDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.HIDConstants;
import frc.robot.common.annotations.Robot;
import frc.robot.common.components.RobotUtils;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
@Robot(team = 9991)
public class PracticumInStemContainer implements IRobotContainer {

  public static final TankDriveSubsystem DRIVE_SUBSYSTEM = new TankDriveSubsystem(1, 2);
    
    public static IRobotContainer createContainer(){
      //TODO Binds and Tank Drive go here
      configureBindings();

      return new PracticumInStemContainer();
    }

    private static void configureBindings() {
      RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.a(), Commands.run(() -> DRIVE_SUBSYSTEM.setMotorSpeed(0.1)), Commands.run(() -> DRIVE_SUBSYSTEM.setMotorSpeed(0)));
    }
    
    
    @Override    
    public Command getAutonomousCommand() {
        return null;
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }
    
}
