package frc.robot.common.components;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.common.annotations.DashboardVariable;
import org.reflections.Reflections;
import org.reflections.scanners.Scanners;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * Automatically updates SmartDashboard for any static field annotated with @DashboardVariable.
 */
public class DashboardAutoUpdater {

    private record DashboardEntry(Field field, DashboardVariable annotation, String key) {
    }

    private static final List<DashboardEntry> entries = new ArrayList<>();

    static {
        Reflections reflections = new Reflections("frc.robot", Scanners.FieldsAnnotated);
        Set<Field> annotatedFields = reflections.getFieldsAnnotatedWith(DashboardVariable.class);

        for (Field field : annotatedFields) {
            try {
                // Only allow static fields (no instance resolution)
                if (!java.lang.reflect.Modifier.isStatic(field.getModifiers())) {
                    System.err.println("Ignoring non-static field: "
                            + field.getDeclaringClass().getSimpleName()
                            + "." + field.getName());
                    continue;
                }

                field.setAccessible(true);
                DashboardVariable annotation = field.getAnnotation(DashboardVariable.class);
                String key = annotation.name().isEmpty() ? field.getName() : annotation.name();

                System.out.println("Found DashboardVariable: " + key);

                entries.add(new DashboardEntry(field, annotation, key));

                // If persistent, mark it once
                if (annotation.persistent()) {
                    SmartDashboard.setPersistent(key);
                }

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Call this periodically (e.g. in robotPeriodic).
     */
    public static void updateAll() {
        for (DashboardEntry entry : entries) {
            try {
                Object value = entry.field.get(null); // static → no instance needed

                if (value instanceof Number) {
                    SmartDashboard.putNumber(entry.key, ((Number) value).doubleValue());
                } else if (value instanceof Boolean) {
                    SmartDashboard.putBoolean(entry.key, (Boolean) value);
                } else if (value != null) {
                    SmartDashboard.putString(entry.key, value.toString());
                }

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}

