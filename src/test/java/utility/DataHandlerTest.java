package utility;

import static org.junit.Assert.*;

import com.kuka.roboticsAPI.deviceModel.*;
import com.kuka.roboticsAPI.geometricModel.Frame;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.Comparator;
import org.junit.*;

public class DataHandlerTest {
  private static Path folder;

  @BeforeClass
  public static void setup() throws Exception {
    folder = Files.createTempDirectory("teaching-test-");
    System.setProperty("robot.dataDir", folder.toString());
  }

  @Before
  public void clean() throws Exception {
    try (java.util.stream.Stream<Path> paths = Files.walk(folder)) {
      paths
          .sorted(Comparator.reverseOrder())
          .filter(p -> !p.equals(folder))
          .forEach(
              p -> {
                try {
                  Files.delete(p);
                } catch (Exception e) {
                  throw new RuntimeException(e);
                }
              });
    }
  }

  @AfterClass
  public static void cleanup() throws Exception {
    new DataHandlerTest().clean();
    Files.delete(folder);
  }

  @Test
  public void emptyStartsAtZeroAndAdvances() {
    assertEquals("F0", DataHandler.getNextFrameName());
    assertEquals("J0", DataHandler.getNextJointPosName());
    DataHandler.saveFrame("F0", new Frame(400, 10, 350, .2, .3, .4));
    DataHandler.saveJointPosition("J0", new JointPosition(.1, .2, .3, .4, .5, .6, .7));
    assertEquals("F1", DataHandler.getNextFrameName());
    assertEquals("J1", DataHandler.getNextJointPosName());
  }

  @Test
  public void namesWithGapsDoNotOverwriteOrRename() {
    DataHandler.saveFrame("F1", new Frame(100, 0, 0));
    DataHandler.saveFrame("F4", new Frame(400, 0, 0));
    DataHandler.saveFrame("wireloop_start", new Frame(0, 0, 0));
    assertEquals("F5", DataHandler.getNextFrameName());
    assertEquals(100, DataHandler.loadFrame("F1").getX(), 0);
    try {
      DataHandler.loadFrame("F0");
      fail();
    } catch (IllegalStateException e) {
      assertTrue(e.getMessage().contains("Exercise 1"));
    }
  }

  @Test
  public void poseUnitsAndSimulationSeedRoundTrip() {
    Frame frame = new Frame(412.345, -123.456, 789.123, .6, -.3, 2.1);
    double[] q = {.1, .2, .3, -.9, .8, .7, .6};
    frame.setAdditionalParameter(DataHandler.SIM_JOINT_SEED, q);
    DataHandler.saveFrame("F0", frame);
    Frame copy = DataHandler.loadFrame("F0");
    assertEquals(frame.getX(), copy.getX(), 1e-8);
    assertEquals(frame.getY(), copy.getY(), 1e-8);
    assertEquals(frame.getZ(), copy.getZ(), 1e-8);
    assertEquals(frame.getAlphaRad(), copy.getAlphaRad(), 1e-10);
    assertEquals(frame.getBetaRad(), copy.getBetaRad(), 1e-10);
    assertEquals(frame.getGammaRad(), copy.getGammaRad(), 1e-10);
    assertArrayEquals(q, (double[]) copy.getAdditionalParameter(DataHandler.SIM_JOINT_SEED), 1e-11);
  }

  @Test
  public void namedRedundancyRoundTrip() {
    Frame frame = new Frame(400, 0, 350);
    frame.setRedundancyInformation("LBR_iiwa_7_R800_1", new LBRE1Redundancy(.43, 2, 17));
    DataHandler.saveFrame("F0", frame);
    LBRE1Redundancy red =
        (LBRE1Redundancy)
            DataHandler.loadFrame("F0").getRedundancyInformation().get("LBR_iiwa_7_R800_1");
    assertEquals(.43, red.getE1(), 1e-11);
    assertEquals(2, red.getStatus());
    assertEquals(17, red.getTurn());
  }

  @Test
  public void legacyWhitespaceAndDegrees() throws Exception {
    Files.createDirectories(folder.resolve("frames"));
    Files.write(
        folder.resolve("frames/F0.frm"),
        "  400\t20  350 90 0 -180\n".getBytes(StandardCharsets.UTF_8));
    Frame f = DataHandler.loadFrame("F0");
    assertEquals(Math.PI / 2, f.getAlphaRad(), 1e-10);
    assertEquals(-Math.PI, f.getGammaRad(), 1e-10);
  }

  @Test
  public void rejectsMalformedAndMissingFiles() throws Exception {
    try {
      DataHandler.loadJointPos("J0");
      fail();
    } catch (IllegalStateException expected) {
    }
    Files.createDirectories(folder.resolve("frames"));
    Files.write(folder.resolve("frames/F0.frm"), "NaN 0 0 0 0 0".getBytes(StandardCharsets.UTF_8));
    try {
      DataHandler.loadFrame("F0");
      fail();
    } catch (IllegalArgumentException expected) {
    }
    try {
      DataHandler.saveFrame("../F0", new Frame(0, 0, 0));
      fail();
    } catch (IllegalArgumentException expected) {
    }
  }

  @Test
  public void jointsStayInRadians() {
    double[] q = {-1.2, .4, .2, -.6, .2, .7, 1.9};
    DataHandler.saveJointPosition("J0", new JointPosition(q));
    assertArrayEquals(q, DataHandler.loadJointPos("J0").get(), 1e-11);
  }
}
