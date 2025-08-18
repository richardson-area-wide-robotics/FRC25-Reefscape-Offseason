package frc.robot;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;
import frc.robot.common.interfaces.IRobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.annotations.Robot;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
@Robot(team = 9991)
public class PracticumInStemContainer implements IRobotContainer {
    
    public static IRobotContainer createContainer(){
      //TODO Binds and Tank Drive go here


      return new PracticumInStemContainer();
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
