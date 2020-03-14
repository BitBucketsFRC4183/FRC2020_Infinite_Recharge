package frc.robot.utils.dashboard;

import java.lang.reflect.Proxy;

public class DashboardFactory {
    /**
     * Construct a new instance of a SmartDashboard proxy
     * @param clazz The class to create
     * @param <T> The type of the interface
     * @return A new concrete class for the interface
     */
    public static <T> T create(Class<T> clazz) {

        T object = (T) Proxy.newProxyInstance(clazz
                        .getClassLoader(), new Class[]{clazz},
                new DashboardHandler(clazz));

        return object;
    }

}
