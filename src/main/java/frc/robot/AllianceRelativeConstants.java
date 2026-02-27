package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
public class AllianceRelativeConstants {
    public static <T> T getHud(T red_var, T blue_var){
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            return red_var;
        }
        else{
            return blue_var;
        }
    }

}