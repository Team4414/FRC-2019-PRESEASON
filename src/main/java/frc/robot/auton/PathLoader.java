package frc.robot.auton;

import java.io.File;
import java.util.LinkedHashMap;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

public class PathLoader{

    private static String kPathLocation = "/home/lvuser/autoPaths/";
    private static String kPathSuffix = "_left_detailed.csv";

    public static LinkedHashMap<String, Trajectory> loadPaths(){
        LinkedHashMap<String, Trajectory> paths = new LinkedHashMap<String, Trajectory>();

        paths.put("DriveCurveLeft", getTraj("LeftCurve"));
        paths.put("DriveCurveRight", getTraj("RightCurve"));

    }

    private static Trajectory getTraj(String name){
        return Pathfinder.readFromCSV(
            new File(
                kPathLocation + name + kPathSuffix
            )
        );
    }
}