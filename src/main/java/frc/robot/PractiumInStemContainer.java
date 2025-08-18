package frc.robot;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;
import frc.robot.common.interfaces.IRobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.annotations.Robot;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
@Robot(team = 1745) //Note: This class is also the defualt so it will be loaded on 8874
public class PracticumInStemContainer implements IRobotContainer {@Override
    public Command getAutonomousCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAutonomousCommand'");
    }

    @Override
    public void simulationPeriodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'simulationPeriodic'");
    }

    @Override
    public void disabledPeriodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'disabledPeriodic'");
    }

    @Override
    public void autonomousPeriodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'autonomousPeriodic'");
    }

    @Override
    public void teleopPeriodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'teleopPeriodic'");
    }
    
}
