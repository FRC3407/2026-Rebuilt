package frc.robot;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
public class AllianceRelativeConstants {
    public static Pose2d getHud(){
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            return PathfindingConstants.Red_hub_pose;
        }
        else{
            return PathfindingConstants.Blue_hub_pose;
        }
    }

}