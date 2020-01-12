package frc.robot.config;

import frc.robot.config.urations.TestDriveConfiguration;

public class ConfigChooser {

    public enum ConfigType {
        // The robot we use in the competitions.
        Main,

        // I just need a comment here so that VSCode doesn't ruin my format.
        Junior,

        // The second robot if we build one.
        TestDrive
    }

    private static ConfigType configType = ConfigType.Main;

    public static Config getConfig() {
        switch (configType) {
        case Junior:
            //return new JuniorConfiguration();
        case TestDrive:
            return new TestDriveConfiguration();
        case Main:
        default:
            return new Config();
        }
    }
}
