package frc.robot.common.components;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.json.JSONObject;

public class TeamUtils {

    /**
     * Helper method to get the team number from the wpilib_preferences.json
     * Used by {@link RobotUtils#getTeamNumber} in sim
     *
     * @author Hudson Strub
     * @since 2025 Offseason
     */
    public static int getTeamNumber() {
            try {
                // Look inside the project’s .wpilib folder
                Path projectDir = Paths.get(System.getProperty("user.dir"));
                Path prefPath = projectDir.resolve(".wpilib").resolve("wpilib_preferences.json");

                if (Files.exists(prefPath)) {
                    String content = Files.readString(prefPath);
                    JSONObject json = new JSONObject(content);
                    return json.optInt("teamNumber", 0);
                }
            } catch (IOException e) {
                System.err.println("Error loading team number from file!");
            }
            return 200; // fallback in sim if file missing
        }
    }
