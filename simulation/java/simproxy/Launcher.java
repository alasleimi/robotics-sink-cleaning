package simproxy;

/** Calls the unchanged course main; absent from the hardware classpath. */
public final class Launcher {
  public static void main(String[] args) throws Exception {
    String[] exercises = {
      "Task_01_TeachIn",
      "Task_02_BasicMotions",
      "Task_03_ElbowConfiguration",
      "Task_04_JointAndTaskSpace",
      "Task_05a_JointImpedanceControl",
      "Task_05b_CartesianImpedanceControl",
      "Task_06_BreakConditions",
      "Task_07_SingularPositions",
      "Task_08_Wireloop"
    };
    String selected = args.length == 0 ? "01" : args[0], name = null;
    for (String candidate : exercises)
      if (candidate.startsWith("Task_" + selected + "_")) name = candidate;
    if (name == null)
      throw new IllegalArgumentException(
          "Exercise must be 01, 02, 03, 04, 05a, 05b, 06, 07, or 08");
    System.setProperty("robot.dataDir", System.getProperty("robot.dataDir", "data/simulation/robosuite-sunrise"));
    try {
      if (selected.equals("02"))
        for (int i = 0; i < 3; i++) {
          utility.DataHandler.loadFrame("F" + i);
          utility.DataHandler.loadJointPos("J" + i);
        }
      if (selected.equals("08")) utility.DataHandler.loadFrame("wireloop_start");
      Bridge bridge = new Bridge();
      System.out.println(
          "SIMULATION: iiwa 7 R800 - data in " + System.getProperty("robot.dataDir"));
      try (SimulatorRuntime runtime = new SimulatorRuntime(bridge)) {
        if (!java.awt.GraphicsEnvironment.isHeadless()) TeachingPanel.open(bridge);
        try {
          Class.forName("exercises." + name)
              .getMethod("main", String[].class)
              .invoke(null, (Object) new String[0]);
        } catch (java.lang.reflect.InvocationTargetException e) {
          throw new IllegalStateException(
              "Exercise failed: " + e.getCause().getMessage(), e.getCause());
        }
      }
    } catch (Throwable error) {
      error.printStackTrace();
      if (!java.awt.GraphicsEnvironment.isHeadless())
        javax.swing.JOptionPane.showMessageDialog(
            null,
            error.getMessage(),
            "Simulator exercise stopped",
            javax.swing.JOptionPane.ERROR_MESSAGE);
      System.exit(1);
    }
  }
}
