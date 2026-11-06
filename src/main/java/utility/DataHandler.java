package utility;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.redundancy.IRedundancyCollection;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import java.util.regex.*;
import java.util.stream.Stream;
import robot.SunriseConnector;

/** Teaching files: millimetres/degrees in .frm, radians in .jnt. */
public final class DataHandler {
  public static final String DataPath = System.getProperty("robot.dataDir", "data");
  public static final String SIM_JOINT_SEED = "simulationJointSeed";

  private DataHandler() {}

  private static Path file(String folder, String name, String extension) {
    if (name == null || !name.matches("[A-Za-z0-9_-]+"))
      throw new IllegalArgumentException(
          "Use letters, numbers, underscores or hyphens for a teaching name.");
    return Paths.get(DataPath, folder, name + extension);
  }

  private static String next(String folder, String prefix, String extension) {
    Path dir = Paths.get(DataPath, folder);
    try {
      Files.createDirectories(dir);
      Pattern pattern =
          Pattern.compile(Pattern.quote(prefix) + "(\\d+)" + Pattern.quote(extension));
      int max = -1;
      try (Stream<Path> paths = Files.list(dir)) {
        for (Path path : (Iterable<Path>) paths::iterator) {
          Matcher match = pattern.matcher(path.getFileName().toString());
          if (Files.isRegularFile(path) && match.matches())
            max = Math.max(max, Integer.parseInt(match.group(1)));
        }
      }
      return prefix + Math.addExact(max, 1);
    } catch (IOException e) {
      throw new IllegalStateException("Cannot list teaching files in " + dir, e);
    }
  }

  public static String getNextFrameName() {
    return next("frames", "F", ".frm");
  }

  public static String getNextJointPosName() {
    return next("joint_positions", "J", ".jnt");
  }

  private static void write(Path path, String text) {
    try {
      Files.createDirectories(path.getParent());
      Path temporary = Files.createTempFile(path.getParent(), ".teach-", ".tmp");
      try {
        Files.write(temporary, text.getBytes(StandardCharsets.UTF_8));
        try {
          Files.move(
              temporary, path, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING);
        } catch (AtomicMoveNotSupportedException e) {
          Files.move(temporary, path, StandardCopyOption.REPLACE_EXISTING);
        }
      } finally {
        Files.deleteIfExists(temporary);
      }
      System.out.println("Saved " + path);
    } catch (IOException e) {
      throw new IllegalStateException("Cannot save " + path, e);
    }
  }

  public static void saveFrame(String name, Frame frame) {
    Objects.requireNonNull(frame, "Cannot save a null frame");
    Path destination = file("frames", name, ".frm");
    Frame world = new Frame(frame.transformationFromWorld());
    for (double v :
        new double[] {
          world.getX(),
          world.getY(),
          world.getZ(),
          world.getAlphaRad(),
          world.getBetaRad(),
          world.getGammaRad()
        })
      if (!Double.isFinite(v)) throw new IllegalArgumentException("Cannot save a non-finite pose");
    StringBuilder text =
        new StringBuilder(
            String.format(
                Locale.ROOT,
                "%.9f %.9f %.9f %.9f %.9f %.9f%n",
                world.getX(),
                world.getY(),
                world.getZ(),
                Math.toDegrees(world.getAlphaRad()),
                Math.toDegrees(world.getBetaRad()),
                Math.toDegrees(world.getGammaRad())));
    // Named, versioned metadata avoids the old, undocumented parameter ordering.
    // A plain frame without redundancy is valid and needs no second line.
    for (Map.Entry<String, IRedundancyCollection> entry :
        frame.getRedundancyInformation().entrySet()) {
      if (entry.getValue() instanceof LBRE1Redundancy) {
        LBRE1Redundancy red = (LBRE1Redundancy) entry.getValue();
        if (!Double.isFinite(red.getE1()))
          throw new IllegalArgumentException("Cannot save a non-finite elbow angle");
        String key =
            Base64.getUrlEncoder().encodeToString(entry.getKey().getBytes(StandardCharsets.UTF_8));
        text.append(
            String.format(
                Locale.ROOT,
                "# LBRE1-v1 %s %.12f %d %d%n",
                key,
                red.getE1(),
                red.getStatus(),
                red.getTurn()));
      } else {
        throw new IllegalArgumentException(
            "Unsupported redundancy type: " + entry.getValue().getClass().getName());
      }
    }
    if (frame.hasAdditionalParameter(SIM_JOINT_SEED)) {
      double[] seed = (double[]) frame.getAdditionalParameter(SIM_JOINT_SEED);
      if (seed.length != 7) throw new IllegalArgumentException("Expected seven seed joints");
      text.append("# sim-joints-v1");
      for (double q : seed) {
        if (!Double.isFinite(q))
          throw new IllegalArgumentException("Cannot save a non-finite joint seed");
        text.append(String.format(Locale.ROOT, " %.12f", q));
      }
      text.append('\n');
    }
    write(destination, text.toString());
  }

  public static void saveFrame() {
    saveFrame(SunriseConnector.getControlGui().getFrameName(), SunriseConnector.getTcpFrame());
    SunriseConnector.getControlGui().setNextFrameName(getNextFrameName());
  }

  private static List<String> read(Path path) {
    try {
      return Files.readAllLines(path, StandardCharsets.UTF_8);
    } catch (NoSuchFileException e) {
      throw new IllegalStateException(
          "Missing taught position: "
              + path
              + ". Save it in Exercise 1 first. Existing files are never renumbered automatically.",
          e);
    } catch (IOException e) {
      throw new IllegalStateException("Cannot read " + path, e);
    }
  }

  private static double[] values(String line, int count, Path path) {
    String[] words = line.trim().split("\\s+");
    if (words.length != count)
      throw new IllegalArgumentException("Expected " + count + " values in " + path);
    double[] values = new double[count];
    for (int i = 0; i < count; i++) {
      values[i] = Double.parseDouble(words[i]);
      if (!Double.isFinite(values[i]))
        throw new IllegalArgumentException("Non-finite value in " + path);
    }
    return values;
  }

  public static Frame loadFrame(String name) {
    Path path = file("frames", name, ".frm");
    List<String> lines = read(path);
    if (lines.isEmpty()) throw new IllegalArgumentException("Empty teaching file: " + path);
    double[] v = values(lines.get(0), 6, path);
    Frame result =
        new Frame(
            v[0], v[1], v[2], Math.toRadians(v[3]), Math.toRadians(v[4]), Math.toRadians(v[5]));
    for (String line : lines.subList(1, lines.size())) {
      if (line.startsWith("# LBRE1-v1 ")) {
        String[] p = line.trim().split("\\s+");
        if (p.length != 6)
          throw new IllegalArgumentException("Invalid redundancy metadata in " + path);
        String key = new String(Base64.getUrlDecoder().decode(p[2]), StandardCharsets.UTF_8);
        double e1 = Double.parseDouble(p[3]);
        if (!Double.isFinite(e1))
          throw new IllegalArgumentException("Non-finite elbow metadata in " + path);
        result.setRedundancyInformation(
            key, new LBRE1Redundancy(e1, Integer.parseInt(p[4]), Integer.parseInt(p[5])));
      } else if (line.startsWith("# sim-joints-v1 ")) {
        result.setAdditionalParameter(
            SIM_JOINT_SEED, values(line.substring("# sim-joints-v1 ".length()), 7, path));
      } else if (!line.trim().isEmpty() && !line.startsWith("#")) {
        // Old files saved positional redundancy values but never restored them.
        System.err.println(
            "Legacy unnamed redundancy in "
                + path
                + " cannot be interpreted reliably; pose loaded. Re-teach to preserve elbow"
                + " configuration.");
      }
    }
    return result;
  }

  public static void saveJointPosition(String name, JointPosition joints) {
    Objects.requireNonNull(joints, "Cannot save null joints");
    StringJoiner line = new StringJoiner(" ");
    for (int i = 0; i < 7; i++) {
      if (!Double.isFinite(joints.get(i)))
        throw new IllegalArgumentException("Cannot save a non-finite joint angle");
      line.add(String.format(Locale.ROOT, "%.12f", joints.get(i)));
    }
    write(file("joint_positions", name, ".jnt"), line.toString() + "\n");
  }

  public static void saveJointPosition() {
    saveJointPosition(
        SunriseConnector.getControlGui().getJointPosName(),
        SunriseConnector.getRobot().getCurrentJointPosition());
    SunriseConnector.getControlGui().setNextJointPosName(getNextJointPosName());
  }

  public static JointPosition loadJointPos(String name) {
    Path path = file("joint_positions", name, ".jnt");
    List<String> lines = read(path);
    if (lines.isEmpty()) throw new IllegalArgumentException("Empty teaching file: " + path);
    return new JointPosition(values(lines.get(0), 7, path));
  }
}
