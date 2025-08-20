package frc.robot.common.components;

import frc.robot.practicum.PracticumInStemContainer;
import frc.robot.common.annotations.Robot;
import frc.robot.common.interfaces.IRobotContainer;
import lombok.experimental.UtilityClass;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@UtilityClass
public class RobotContainerRegistry {

    // A map to hold robot containers by team
    private static final Map<Integer, Class<?>> teamContainers = new HashMap<>();

    static {
        // Register the containers for each team
        for (Class<?> clazz : getAllClasses()) {
            if (clazz.isAnnotationPresent(Robot.class)) {
                Robot annotation = clazz.getAnnotation(Robot.class);
                teamContainers.put(annotation.team(), clazz);
            }
        }
    }

    public static IRobotContainer createContainerForTeam(int teamNumber) {
        // Try to get the container for the specific team
        Class<?> containerClass = teamContainers.get(teamNumber);

        // If not found, use the default container
        if (containerClass == null) {
            System.out.println("We cant find a container for " + teamNumber + "! Using default");
            return PracticumInStemContainer.createContainer(); 
        }

        try { //Create the container
            return (IRobotContainer) containerClass.getMethod("createContainer").invoke(null);
        } catch (Exception e) {
            e.printStackTrace();
        }

        return null; 
    }

    private static final String PACKAGE_NAME = "frc.robot";  

    private static List<Class<?>> getAllClasses() {
        List<Class<?>> classes = new ArrayList<>();
        String path = PACKAGE_NAME.replace('.', '/');
    
        try {
            var resources = RobotContainerRegistry.class.getClassLoader().getResources(path);
            while (resources.hasMoreElements()) {
                var url = resources.nextElement();
                var protocol = url.getProtocol();
    
                if ("file".equals(protocol)) {
                    // Running from exploded class files (e.g., in IDE)
                    var dir = new java.io.File(url.toURI());
                    for (var file : dir.listFiles()) {
                        if (file.getName().endsWith(".class")) {
                            String className = PACKAGE_NAME + "." + file.getName().replace(".class", "");
                            classes.add(Class.forName(className));
                        }
                    }
                } else if ("jar".equals(protocol)) {
                    // Running from inside a JAR
                    String jarPath = url.getPath().substring(5, url.getPath().indexOf("!")); // strip "file:" and "!..."
                    try (java.util.jar.JarFile jar = new java.util.jar.JarFile(jarPath)) {
                        var entries = jar.entries();
                        while (entries.hasMoreElements()) {
                            var entry = entries.nextElement();
                            String name = entry.getName();
                            if (name.startsWith(path) && name.endsWith(".class") && !entry.isDirectory()) {
                                String className = name.replace('/', '.').replace(".class", "");
                                classes.add(Class.forName(className));
                            }
                        }
                    }
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        return classes;
    }    
}

