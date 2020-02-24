package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// to see what's connected to the CAN Bus and what's not
public class CANChecker {
    private final ArrayList<BaseTalon> talons;
    private final String NAME = "CAN Bus";

    private int i = -1; // 1 will get added on first iteration
    private final int UPDATE_PERIOD = 25; // every 25 iterations, called at 50Hz = twice a second



    public CANChecker() {
        talons = new ArrayList<BaseTalon>();
    }

    public void addTalons(BaseTalon... newTalons) {
        for (int i = 0; i < newTalons.length; i++) {
            talons.add(newTalons[i]);
        }
    }

    public void periodic() {
        i++;
        i = i % UPDATE_PERIOD;
        if (i == 0) {
            check();
        }
    }

    public void check() {
        boolean allGood = true;

        for (int i = 0; i < talons.size(); i++) {
            BaseTalon talon = talons.get(i);
            int id = talon.getDeviceID();

            int param = talon.configGetCustomParam(0);
            boolean connected = (param == id);

            SmartDashboard.putBoolean(NAME + "/" + id, connected);
            if (!connected) {
                // in case it hasn't been configured or maybe it was changed
                talon.configSetCustomParam(id, 0); // set the parameter to its ID
                allGood = false;
            }
        }

        SmartDashboard.putBoolean(NAME + "/All good", allGood); // probably want to do something with it
    }
}