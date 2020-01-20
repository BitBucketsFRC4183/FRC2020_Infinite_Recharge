package frc.robot.config;

import frc.robot.config.urations.Cal9000Configuration;
import frc.robot.config.urations.JuniorConfiguration;

public class ConfigChooser {

    public enum ConfigType {
        // The robot we use in the competitions.
        Main,

        // I just need a comment here so that VSCode doesn't ruin my format.
        Junior,

        // The second robot if we build one.
        Cal9000
    }

    //
    // Change this type to pick config
    //
    private static ConfigType configType = ConfigType.Junior;

    public static Config getConfig() {
        switch (configType) {
        case Junior:
            return new JuniorConfiguration();
        case Cal9000:
            return new Cal9000Configuration();
        case Main:
        default:
            return new Config();
        }
    }
}
