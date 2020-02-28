package frc.robot.config;

import edu.wpi.first.wpilibj.Preferences;

import frc.robot.config.urations.Cal9000Configuration;
import frc.robot.config.urations.FocusedExcellenceConfiguration;
import frc.robot.config.urations.JuniorConfiguration;

public class ConfigChooser {

    public enum ConfigType {
        // The robot we use in the competitions.
        Main (new Config()),

        // I just need a comment here so that VSCode doesn't ruin my format.
        Junior (new JuniorConfiguration()),

        // The second robot if we build one.
        Cal9000 (new Cal9000Configuration()),

        // 2019 Destination: Deep Space robot
        FocusedExellence (new FocusedExcellenceConfiguration());



        private Config config;
        ConfigType(Config config) {
            this.config = config;
        }

        Config getConfig() {
            return config;
        }
    }

    //
    // Change this type to pick config to load in if there's
    // no config or if the robot is to be renamed
    //
    private static ConfigType manualConfigType = ConfigType.Main;
    // configuration type the robot will choose (manualConfigType) if no match found
    // with name of an existing configuration
    private static ConfigType configType = getConfigType();

    // probably would be a good idea to never change this
    private final static String KEY = "NAME";

    private static ConfigType getConfigType() {
        Preferences p = Preferences.getInstance();



        String name = p.getString(KEY, "");
        ConfigType[] types = ConfigType.values();

        for (int i = 0; i < types.length; i++) {
            Config config = types[i].getConfig();

            if (config.name.equals(name)) {
                return types[i]; // return configuration with name that matches that on Rio
            }
        }

        // at this point, no match has been found
        // assume the default specified by manualConfigType
        p.putString(KEY, manualConfigType.getConfig().name);

        return manualConfigType;
    }

    public static Config getConfig() {
        return configType.getConfig();
    }
}
