package frc.robot.subsystems.swerve;

import java.io.File;
import java.io.FileReader;

import edu.wpi.first.wpilibj.Filesystem;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class Swerve {
    public static class Constants {
    }

    private SwerveModule[] modules = new SwerveModule[4];

    public Swerve(SwerveModule[] modules) {
        this.modules = modules;
    }
}
