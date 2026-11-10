package simproxy;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.google.gson.*;
import com.kuka.roboticsAPI.conditionModel.*;
import com.kuka.roboticsAPI.deviceModel.*;
import com.kuka.roboticsAPI.geometricModel.*;
import com.kuka.roboticsAPI.geometricModel.math.*;
import com.kuka.roboticsAPI.motionModel.*;
import com.kuka.roboticsAPI.motionModel.controlModeModel.*;
import java.nio.file.*;
import java.util.concurrent.*;
import robot.SunriseConnector;
import utility.*;

/** Assertions against running physics; deliberately separate from student exercises. */
public final class ContractCheck {
  static void require(boolean value, String message) {
    if (!value) throw new AssertionError(message);
  }

  static void near(double[] a, double[] b, double tolerance, String message) {
    for (int i = 0; i < a.length; i++)
      require(
          Math.abs(a[i] - b[i]) < tolerance, message + " joint " + i + ": " + a[i] + " vs " + b[i]);
  }

  public static void main(String[] args) throws Exception {
    Bridge bridge = new Bridge();
    try (SimulatorRuntime runtime = new SimulatorRuntime(bridge)) {
      SunriseConnector.initialize(new exercises.Task_03_ElbowConfiguration(runtime.context));
      LBR robot = SunriseConnector.getRobot();
      Tool tool = SunriseConnector.getTool();
      if (args.length > 0 && args[0].equals("save")) {
        DataHandler.saveFrame();
        DataHandler.saveJointPosition();
        System.out.println("SAVE PASS");
        return;
      }
      if (args.length > 0 && args[0].equals("wire")) {
        tool.move(ptp(new Frame(400, 200, 430, Math.PI, 0, Math.PI)));
        tool.move(lin(new Frame(400, 200, 350, Math.PI, 0, Math.PI)));
        DataHandler.saveFrame("wireloop_start", SunriseConnector.getTcpFrame());
        System.out.println("WIRE FIXTURE PASS");
        return;
      }
      bridge.call("abort");
      bridge.call("resume");
      SunriseConnector.goFront();
      double[] front = {0, Math.PI / 6, 0, -Math.PI / 3, 0, Math.PI / 4, 0};
      near(front, robot.getCurrentJointPosition().get(), .012, "PTP actual endpoint");
      Thread.sleep(150);
      Frame start = SunriseConnector.getTcpFrame();
      double[] p = Poses.pose(start);
      Frame roundtrip = Poses.frame(p);
      require(
          start.distanceTo(roundtrip) < 1e-6 && start.rotationalDistanceTo(roundtrip) < 1e-6,
          "Pose rotation round trip");
      Frame flange = robot.getCurrentCartesianPosition(robot.getFlange());
      require(
          Math.abs(start.distanceTo(flange) - 202) < 1,
          "TCP/flange offset is 202 mm; got " + start.distanceTo(flange));
      tool.move(linRel(-20, 0, 0, robot.getRootFrame()).setCartVelocity(80));
      Frame relative = SunriseConnector.getTcpFrame();
      require(Math.abs(relative.getX() - start.getX() + 20) < 2, "World-relative LIN x");
      require(Math.abs(relative.getY() - start.getY()) < 2, "World-relative LIN y");
      tool.move(lin(start).setCartVelocity(80));
      require(start.distanceTo(SunriseConnector.getTcpFrame()) < 2, "LIN endpoint");
      double[] beforeTool = Poses.pose(SunriseConnector.getTcpFrame());
      tool.move(linRel(0, 0, -10).setCartVelocity(80));
      Transformation expected =
          Poses.transformation(beforeTool).compose(Transformation.ofTranslation(0, 0, -10));
      require(
          new Frame(expected).distanceTo(SunriseConnector.getTcpFrame()) < 2, "Tool-relative LIN");
      tool.move(
          new MotionBatch(lin(start), lin(start.copy().setX(start.getX() - 10)))
              .setJointVelocityRel(.2));
      require(
          Math.abs(SunriseConnector.getTcpFrame().getX() - start.getX() + 10) < 2,
          "MotionBatch endpoint");
      Frame corner = start.copy().setX(start.getX() - 30);
      Frame exit = corner.copy().setY(corner.getY() + 30);
      IMotionContainer blend = tool.moveAsync(lin(corner).setCartVelocity(80).setBlendingCart(10));
      tool.move(lin(exit).setCartVelocity(80));
      blend.await();
      blend.validate();
      require(exit.distanceTo(SunriseConnector.getTcpFrame()) < 2, "Blended LIN final endpoint");
      boolean movingHandoff = false;
      for (JsonElement event : bridge.call("trace").getAsJsonArray("events")) {
        JsonObject e = event.getAsJsonObject();
        if (e.get("event").getAsString().equals("blendHandoff")
            && e.get("speed").getAsDouble() > .001) movingHandoff = true;
      }
      require(movingHandoff, "Queued blend must pass the corner without stopping");
      IMotionContainer asynchronous =
          robot.moveAsync(
              ptp(new JointPosition(.15, .5, .1, -1, 0, .8, 0)).setJointVelocityRel(.1));
      require(!asynchronous.isFinished(), "Async returns before finish");
      try {
        asynchronous.await(1, TimeUnit.MILLISECONDS);
        throw new AssertionError("Timeout expected");
      } catch (TimeoutException expectedTimeout) {
      }
      bridge.call("pause");
      double[] paused = robot.getCurrentJointPosition().get();
      Thread.sleep(200);
      near(paused, robot.getCurrentJointPosition().get(), .015, "Pause holds joints");
      bridge.call("resume");
      asynchronous.await();
      asynchronous.validate();
      IMotionContainer cancelled =
          robot.moveAsync(ptp(new JointPosition(front)).setJointVelocityRel(.1));
      cancelled.cancel();
      try {cancelled.await(); throw new AssertionError("await must report cancellation");}
      catch(com.kuka.roboticsAPI.executionModel.CancelledException expectedCancellation) {}
      try {cancelled.validate(); throw new AssertionError("validate must report cancellation");}
      catch(com.kuka.roboticsAPI.executionModel.CancelledException expectedCancellation) {}
      require(cancelled.isFinished() && !cancelled.hasError(), "Cancel status");
      SunriseConnector.goFront();
      JointImpedanceControlMode joint = new JointImpedanceControlMode(30, 30, 30, 30, 30, 30, 30);
      joint.setDamping(.7, .7, .7, .7, .7, .7, .7);
      double[] before = robot.getCurrentJointPosition().get();
      IMotionContainer hold = tool.moveAsync(positionHold(joint, -1, TimeUnit.SECONDS));
      bridge.call("force", "force", new double[] {0, 12, 0});
      Thread.sleep(500);
      double[] after = robot.getCurrentJointPosition().get();
      double delta = 0;
      for (int i = 0; i < 7; i++) delta += Math.abs(after[i] - before[i]);
      require(delta > .01, "Physical force deforms compliant joints");
      bridge.call("force", "force", new double[3]);
      hold.cancel();
      try {hold.await(); throw new AssertionError("Cancelled hold must report cancellation");}
      catch(com.kuka.roboticsAPI.executionModel.CancelledException expectedCancellation) {}
      SunriseConnector.goFront();
      ForceCondition force = ForceCondition.createNormalForceCondition(null, CoordinateAxis.Y, 10);
      IMotionContainer broken =
          tool.moveAsync(
              positionHold(
                      new JointImpedanceControlMode(200, 200, 200, 200, 200, 200, 200),
                      -1,
                      TimeUnit.SECONDS)
                  .breakWhen(force));
      Thread.sleep(150);
      require(!broken.isFinished(), "No force: no false break");
      bridge.call("force", "force", new double[] {0, 25, 0});
      broken.await(10, TimeUnit.SECONDS);
      require(broken.hasFired(force), "Force break condition identity");
      bridge.call("force", "force", new double[3]);
      SunriseConnector.goFront();
      FileLogger.startLogging(
          "contract",
          FileLogger.Fields.TIME,
          FileLogger.Fields.JOINT_POSITION,
          FileLogger.Fields.TRANSLATION,
          FileLogger.Fields.ORIENTATION,
          FileLogger.Fields.EXTERNAL_TCP_FORCES_TORQUES);
      Thread.sleep(300);
      FileLogger.stopLogging();
      require(
          Files.readAllLines(Paths.get(DataHandler.DataPath, "logs/contract.log")).size() > 2,
          "Demonstration logger writes samples");
      try {
        robot.setESMState("1");
        throw new AssertionError("Unsupported API accepted");
      } catch (UnsupportedOperationException expectedUnsupported) {
      }
      try {
        new java.net.Socket("198.51.100.1", 30000);
        throw new AssertionError("Non-loopback network allowed");
      } catch (SecurityException expectedBlocked) {
      }
      System.out.println("CONTRACTS PASS");
    }
  }
}
