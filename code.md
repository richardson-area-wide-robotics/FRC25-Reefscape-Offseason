# Code Structure Flowchart

```mermaid
flowchart TD
    subgraph frc_robot[frc.robot]
        frc_robot_BuildConstants[BuildConstants]
        frc_robot_CommonConstants[CommonConstants]
        frc_robot_Main[Main «NoArgsConstructor»]
        frc_robot_Robot[Robot]
    end
    subgraph frc_robot_common_annotations[frc.robot.common.annotations]
        frc_robot_common_annotations_Robot[@Robot «Retention»]
    end
    subgraph frc_robot_common_components[frc.robot.common.components]
        frc_robot_common_components_EasyBreakBeam[EasyBreakBeam «Getter»]
        frc_robot_common_components_EasyMotor[EasyMotor «UtilityClass»]
        frc_robot_common_components_RobotContainerRegistry[RobotContainerRegistry «UtilityClass»]
        frc_robot_common_components_RobotUtils[RobotUtils «UtilityClass»]
    end
    subgraph frc_robot_common_interfaces[frc.robot.common.interfaces]
        frc_robot_common_interfaces_IRobotContainer[IRobotContainer]
    end
    subgraph frc_robot_common[frc.robot.common]
        frc_robot_common_LocalADStarAK[LocalADStarAK]
    end
    subgraph frc_robot_common_subsystems_drive[frc.robot.common.subsystems.drive]
        frc_robot_common_subsystems_drive_SwerveDriveSubsystem[SwerveDriveSubsystem]
        frc_robot_common_subsystems_drive_TankDriveSubsystem[TankDriveSubsystem «Getter»]
    end
    subgraph frc_robot_common_subsystems[frc.robot.common.subsystems]
        frc_robot_common_subsystems_SingleMotorSubsystem[SingleMotorSubsystem]
    end
    subgraph frc_robot_common_swerve[frc.robot.common.swerve]
        frc_robot_common_swerve_RAWRNavX2[RAWRNavX2 «SuppressWarnings»]
        frc_robot_common_swerve_RAWRSwerveModule[RAWRSwerveModule]
    end
    subgraph frc_robot_pearce[frc.robot.pearce]
        frc_robot_pearce_PearceConstants[PearceConstants]
        frc_robot_pearce_RobotContainer[RobotContainer]
    end
    subgraph frc_robot_pearce_subsystems[frc.robot.pearce.subsystems]
        frc_robot_pearce_subsystems_CBSSubsystem[CBSSubsystem]
        frc_robot_pearce_subsystems_DeepClimbSubsystem[DeepClimbSubsystem]
        frc_robot_pearce_subsystems_ElevatorSubsystem[ElevatorSubsystem]
        frc_robot_pearce_subsystems_ScoringSubsystem[ScoringSubsystem]
    end
    subgraph frc_robot_practicum[frc.robot.practicum]
        frc_robot_practicum_PracticumInStemContainer[PracticumInStemContainer «NoArgsConstructor, Robot»]
    end
    frc_robot_common_subsystems_drive_SwerveDriveSubsystem -->|extends| SubsystemBase
    frc_robot_common_subsystems_drive_TankDriveSubsystem -->|extends| SubsystemBase
    frc_robot_common_subsystems_SingleMotorSubsystem -->|extends| SubsystemBase
    frc_robot_common_swerve_RAWRNavX2 -->|extends| LoggableHardware
    frc_robot_common_swerve_RAWRSwerveModule -->|extends| SwerveModule
    frc_robot_pearce_subsystems_CBSSubsystem -->|extends| SubsystemBase
    frc_robot_pearce_subsystems_DeepClimbSubsystem -->|extends| SubsystemBase
    frc_robot_pearce_subsystems_ElevatorSubsystem -->|extends| SubsystemBase
    frc_robot_pearce_subsystems_ScoringSubsystem -->|extends| SubsystemBase
    frc_robot_Robot -->|extends| LoggedRobot
    frc_robot_common_LocalADStarAK -.implements.-> Pathfinder
    frc_robot_common_subsystems_drive_SwerveDriveSubsystem -.implements.-> AutoCloseable
    frc_robot_common_subsystems_drive_TankDriveSubsystem -.implements.-> AutoCloseable
    frc_robot_common_swerve_RAWRNavX2 -.implements.-> IMU
    frc_robot_common_swerve_RAWRSwerveModule -.implements.-> Sendable
    frc_robot_pearce_RobotContainer -.implements.-> frc_robot_common_interfaces_IRobotContainer
    frc_robot_practicum_PracticumInStemContainer -.implements.-> frc_robot_common_interfaces_IRobotContainer
    frc_robot_pearce_subsystems_ScoringSubsystem --> frc_robot_common_components_EasyBreakBeam
    frc_robot_Robot --> frc_robot_common_interfaces_IRobotContainer
    frc_robot_practicum_PracticumInStemContainer -.annotated with.-> frc_robot_common_annotations_Robot
    style frc_robot_BuildConstants fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_common_annotations_Robot fill:#fbc02d,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_common_components_EasyBreakBeam fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_common_components_EasyMotor fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_common_components_RobotContainerRegistry fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_common_components_RobotUtils fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_common_interfaces_IRobotContainer fill:#66bb6a,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_common_LocalADStarAK fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_common_subsystems_drive_SwerveDriveSubsystem fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_common_subsystems_drive_TankDriveSubsystem fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_common_subsystems_SingleMotorSubsystem fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_common_swerve_RAWRNavX2 fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_common_swerve_RAWRSwerveModule fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_CommonConstants fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_Main fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_pearce_PearceConstants fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_pearce_RobotContainer fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_pearce_subsystems_CBSSubsystem fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_pearce_subsystems_DeepClimbSubsystem fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_pearce_subsystems_ElevatorSubsystem fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_pearce_subsystems_ScoringSubsystem fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_practicum_PracticumInStemContainer fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
    style frc_robot_Robot fill:#42a5f5,stroke:#333,stroke-width:2px,color:#fff
```
