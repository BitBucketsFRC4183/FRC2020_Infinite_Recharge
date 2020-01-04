package frc.robot.utils.roborio;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;



public class RoboRIOFS {
    public static final String ABS_PATH_NAME = "/home/lvuser/"; // path files get stored on in RIO
    public static final File ABS_FILE = new File(ABS_PATH_NAME);
    public static final Path ABS_PATH = Paths.get(ABS_PATH_NAME);



    public static final String MOTOR_DATA_NAME = ABS_PATH_NAME + "motors/";
    public static final File MOTOR_DATA_FILE = new File(MOTOR_DATA_NAME);
    public static final Path MOTOR_DATA_PATH = Paths.get(MOTOR_DATA_NAME);



    public static boolean init() {
        boolean success = true;

        if (!Files.exists(MOTOR_DATA_PATH)) {
            if (!MOTOR_DATA_FILE.mkdir()) {
                success = false;
            }
        }

        return success;
    }
}