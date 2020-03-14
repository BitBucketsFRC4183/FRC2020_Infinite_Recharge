package frc.robot.utils.dashboard;

import com.google.common.cache.Cache;
import com.google.common.cache.CacheBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.dashboard.annotations.*;
import org.apache.commons.lang3.StringUtils;

import java.lang.annotation.Annotation;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

/**
 * The DashboardHandler handles a proxy class for an interface. This will proxy any interface calls
 * to SmartDashboard.put() or SmartDashboard.get()
 */
public class DashboardHandler implements InvocationHandler {

    private enum Type {Number, String, Boolean}

    /**
     * A DashboardEntry maps an interface method to dashboard key, type, and default value
     * When a new dashboard is instantiated it will prepopulate all these values to make getting and putting
     * values to the dashboard faster
     */
    private static class DashboardEntry {
        final String dashboardKey;
        final Type type;
        final Object defaultValue;
        final boolean setter;

        DashboardEntry(String dashboardKey, Type type, Object defaultValue, boolean setter) {
            this.dashboardKey = dashboardKey;
            this.type = type;
            this.defaultValue = defaultValue;
            this.setter = setter;
        }
    }


    private final Class clazz;
    private final String name;
    private final Cache<String, Object> getCache;
    private final Map<String, DashboardEntry> entries = new HashMap<>();

    private int expireAfterWriteTime = 0;

    public DashboardHandler(Class clazz) {
        this.clazz = clazz;
        this.name = getNameFromAnnotation();
        this.getCache = CacheBuilder.newBuilder()
                .maximumSize(100)
                .expireAfterWrite(expireAfterWriteTime, TimeUnit.SECONDS)
                .build();
        populateDefaultValues();
    }

    /**
     * Populate the default values for methods at load time so it's faster
     */
    private void populateDefaultValues() {
        for (Method method : this.clazz.getDeclaredMethods()) {
            String key = keyForMethodName(method.getName());
            String dashboardKey = key;
            Object defaultValue = null;
            Type type = Type.String;
            boolean setter = false;

            // check for default value annotations on the method
            Annotation[] annotations = method.getAnnotations();
            for (Annotation annotation : annotations) {
                if (annotation instanceof Default) {
                    defaultValue = ((Default) annotation).value();
                } else if (annotation instanceof DefaultBoolean) {
                    defaultValue = ((DefaultBoolean) annotation).value();
                } else if (annotation instanceof DefaultString) {
                    defaultValue = ((DefaultString) annotation).value();
                } else if (annotation instanceof Key) {
                    // override the key
                    String keyValue = ((Key)annotation).value();
                    if (((Key)annotation).exactPath()) {
                        dashboardKey = keyValue;
                    } else {
                        dashboardKey = name + "/" + keyValue;
                    }
                }
            }

            if (method.getReturnType() == Void.TYPE) {
                // this is a setter
                setter = true;
                if (method.getParameterCount() == 1) {
                    if (method.getParameters()[0].getType().isAssignableFrom(Double.TYPE) ||
                            method.getParameters()[0].getType().isAssignableFrom(Integer.TYPE) ||
                            method.getParameters()[0].getType().isAssignableFrom(Long.TYPE) ||
                            method.getParameters()[0].getType().isAssignableFrom(Number.class)) {
                        // if the first arg is a number, put it in as a double
                        type = Type.Number;
                    } else if (method.getParameters()[0].getType().isAssignableFrom(Boolean.TYPE) ||
                        method.getParameters()[0].getType().isAssignableFrom(Boolean.class)) {
                        // if the first arg is a boolean, put it in as a boolean
                        type = Type.Boolean;
                    }
                }
            } else {
                // if we have a non-void return type, it's a getter
                if (method.getReturnType().isAssignableFrom(Double.TYPE)) {
                    type = Type.Number;
                    if (defaultValue == null) {
                        defaultValue = 0;
                    }
                } else if (method.getReturnType().isAssignableFrom(Boolean.TYPE)) {
                    type = Type.Boolean;
                    if (defaultValue == null) {
                        defaultValue = false;
                    }
                }
            }

            entries.put(key, new DashboardEntry(dashboardKey, type, defaultValue, setter));

            /**
             * Populate any initial getter values based on the default
             */
            boolean populateOnCreate = getPopulateOnCreateFromAnnotation();
            if (populateOnCreate && !setter) {
                switch (type) {
                    case Number:
                        SmartDashboard.putNumber(dashboardKey, (Double) defaultValue);
                        break;
                    case String:
                        SmartDashboard.putString(dashboardKey, (String) defaultValue);
                        break;
                    case Boolean:
                        SmartDashboard.putBoolean(dashboardKey, (Boolean) defaultValue);
                        break;
                }
            }
            
        }
    }

    /**
     * The dashboard name based on the annotation
     *
     * @return A name to use in the SmartDashboard
     */
    String getNameFromAnnotation() {
        Annotation[] annotations = clazz.getAnnotations();
        for (Annotation annotation : annotations) {
            if (annotation instanceof Dashboard) {
                // setup our expireAfterWriteTime for the cache based on the @SmartDashboard annotation
                expireAfterWriteTime = ((Dashboard) annotation).getDelaySeconds();

                // if we have a name, use it for our SmartDashboard key
                String annotationName = ((Dashboard) annotation).value();
                if (annotationName != null && !annotationName.trim().equals("")) {
                    return annotationName;
                }
            }
        }
        // default to the classname without the dashboard, i.e.
        // ShooterDashboard becomes just "Shooter"
        return this.clazz.getSimpleName().replace("Dashboard", "");
    }

    /**
     * Check the @Dashboard annotation to see if we should populate on create
     * @return
     */
    boolean getPopulateOnCreateFromAnnotation() {
        Annotation[] annotations = clazz.getAnnotations();
        for (Annotation annotation : annotations) {
            if (annotation instanceof Dashboard) {
                return ((Dashboard) annotation).populateOnCreate();
            }
        }
        return false;
    }

    /**
     * Build a dashboard key for a method name
     *
     * @param methodName The name of the method to get a key for
     * @return
     */
    String keyForMethodName(String methodName) {
        String entryKey = StringUtils.capitalize(StringUtils.join(StringUtils.splitByCharacterTypeCamelCase(methodName), ' '));
        if (entryKey.startsWith("Put ") || entryKey.startsWith("Get ") || entryKey.startsWith("Set ")) {
            return name + "/" + entryKey.substring(4);
        } else {
            return name + "/" + entryKey;
        }
    }

    /**
     * Invoke any methods on this interface as SmartDashboard methods
     *
     * @param proxy
     * @param method
     * @param args
     * @return
     * @throws Throwable
     */
    @Override
    public Object invoke(Object proxy, Method method, Object[] args) throws Throwable {

        // make sure toString for the proxy works in the debugger.
        if (method.getName().equals("toString")) {
            return proxy.getClass().getName();
        }

        String key = keyForMethodName(method.getName());
        DashboardEntry entry = entries.get(key);
        if (entry == null) {
            System.err.println("Attempting to invoke unknown method in Dashboard interface: " + method.getName());
            return null;
        }

        if (entry.setter) {
            switch (entry.type) {
                case Number:
                    SmartDashboard.putNumber(entry.dashboardKey, (Double) args[0]);
                    break;
                case String:
                    SmartDashboard.putString(entry.dashboardKey, (String) args[0]);
                    break;
                case Boolean:
                    SmartDashboard.putBoolean(entry.dashboardKey, (Boolean) args[0]);
                    break;
            }
            // for setters, we return null
            return null;
        } else {
            switch (entry.type) {

                case Number:
                    return getCache.get(entry.dashboardKey, () -> SmartDashboard.getNumber(entry.dashboardKey, (Double) entry.defaultValue));
                case String:
                    return getCache.get(entry.dashboardKey, () -> SmartDashboard.getString(entry.dashboardKey, (String) entry.defaultValue));
                case Boolean:
                    return getCache.get(entry.dashboardKey, () -> SmartDashboard.getBoolean(entry.dashboardKey, (Boolean) entry.defaultValue));
            }
        }


        throw new IllegalArgumentException("Failed to query SmartDashboard for this value: " + method.getName() + " " + method.getReturnType());
    }
}

// 2020 Recorder Playing Champions