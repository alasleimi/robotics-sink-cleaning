package simproxy;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import static simproxy.ContractCheck.*;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.deviceModel.*;
import com.kuka.roboticsAPI.geometricModel.*;
import com.kuka.roboticsAPI.geometricModel.math.*;
import com.kuka.roboticsAPI.motionModel.*;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import java.nio.file.*;
import java.util.concurrent.TimeUnit;
import robot.SunriseConnector;
import utility.*;

/** Exercise 3/4 requirements tested without filling in the student's templates. */
public final class ExerciseRequirementsCheck {
  private static LBRE1Redundancy redundancy(Frame frame) {
    return (LBRE1Redundancy) frame.getRedundancyInformation().get(SimulatorRuntime.ROBOT_NAME);
  }

  public static void main(String[] args) throws Exception {
    Bridge bridge = new Bridge();
    try (SimulatorRuntime runtime = new SimulatorRuntime(bridge)) {
      SunriseConnector.initialize(new exercises.Task_03_ElbowConfiguration(runtime.context));
      LBR robot = SunriseConnector.getRobot();
      Tool tool = SunriseConnector.getTool();
      bridge.call("abort");
      bridge.call("resume");
      if (args.length > 0 && args[0].equals("wire-contact")) {
        Frame start = DataHandler.loadFrame("wireloop_start");
        tool.move(ptp(start));
        tool.move(linRel(30,0,0,robot.getRootFrame()).setCartVelocity(20));
        ForceCondition contact = ForceCondition.createNormalForceCondition(null,robot.getRootFrame(),CoordinateAxis.Y,10);
        IMotionContainer probe = tool.move(linRel(0,40,0,robot.getRootFrame()).setCartVelocity(20).breakWhen(contact));
        require(probe.hasFired(contact), "Actual ring/wire collision must fire the force break");
        boolean collision = false;
        for (var item : bridge.call("trace").getAsJsonArray("events")) {
          var event = item.getAsJsonObject();
          if (event.has("contacts") && event.get("contacts").getAsInt() > 0
              && event.getAsJsonArray("fired").size() > 0
              && Math.abs(event.getAsJsonArray("force").get(1).getAsDouble()) > 10) collision = true;
        }
        require(collision, "Wire break records physical contact and measured force at the instant it fires");
        tool.move(lin(new Frame(430,200,350,Math.PI,0,Math.PI)).setCartVelocity(20));
        tool.move(lin(start).setCartVelocity(20));
        System.out.println("WIRE CONTACT PASS");
        return;
      }
      SunriseConnector.goFront();
      Frame first = SunriseConnector.getTcpFrame();
      JointPosition joints = robot.getCurrentJointPosition();
      require(Math.abs(redundancy(first).getE1()-joints.get(2)) < .001, "Measured frame carries E1");
      DataHandler.saveFrame("elbow_first", first);

      // E1 overrides the original frame's saved seed at the same TCP pose.
      Frame second = first.copyWithRedundancy();
      second.setRedundancyInformation(robot, new LBRE1Redundancy().setE1(.4));
      tool.move(ptp(second));
      require(Math.abs(robot.getCurrentJointPosition().get(2)-.4) < .002, "Cartesian PTP enforces E1");
      require(first.distanceTo(SunriseConnector.getTcpFrame()) < 1, "E1 preserves TCP position");
      require(first.rotationalDistanceTo(SunriseConnector.getTcpFrame()) < .003, "E1 preserves TCP orientation");
      tool.move(ptp(DataHandler.loadFrame("elbow_first")));
      near(joints.get(), robot.getCurrentJointPosition().get(), .005, "Exercise 3 taught elbow restored");

      // For LIN, an explicit target E1 is interpolated; endpoint Status is ignored.
      second.setRedundancyInformation(robot, new LBRE1Redundancy(.3, 5, 127));
      tool.move(lin(second));
      require(Math.abs(robot.getCurrentJointPosition().get(2)-.3) < .002, "LIN enforces endpoint E1");
      require(first.distanceTo(SunriseConnector.getTcpFrame()) < 1, "LIN null-space motion holds tool");
      Frame plain = new Frame(SunriseConnector.getTcpFrame().transformationFromWorld());
      plain.setX(plain.getX()-10);
      tool.move(lin(plain));
      require(Math.abs(robot.getCurrentJointPosition().get(2)-.3) < .002, "Untaught LIN retains starting E1");

      Frame fk = robot.getForwardKinematic(robot.getCurrentJointPosition());
      near(robot.getCurrentJointPosition().get(), robot.getInverseKinematicFromFrameAndRedundancy(fk).get(), .003, "SDK FK/IK with redundancy");
      Frame invalid = new Frame(plain.transformationFromWorld());
      invalid.setRedundancyInformation(robot, new LBRE1Redundancy().setE1(4));
      double[] before = robot.getCurrentJointPosition().get();
      IMotionContainer rejected = tool.moveAsync(ptp(invalid));
      try {rejected.await(); throw new AssertionError("await must report invalid commands");}
      catch(com.kuka.roboticsAPI.executionModel.CommandInvalidException expected) {}
      require(rejected.hasError() && rejected.getErrorMessage().contains("E1"), "Invalid E1 is an explicit motion error");
      near(before, robot.getCurrentJointPosition().get(), .003, "Rejected E1 causes no motion");

      // Course Task 4: all three kinds of batch plus its requested distance logs.
      JointPosition[] jp = {joints, new JointPosition(.12,.55,.08,-1.05,.03,.8,0),
          new JointPosition(-.12,.5,-.06,-1.05,-.03,.8,0)};
      Frame[] frames = new Frame[3];
      for (int i=0; i<3; i++) {
        robot.move(ptp(jp[i]));
        frames[i] = SunriseConnector.getTcpFrame();
      }
      for (int type=0; type<3; type++) {
        robot.move(ptp(jp[0]));
        RobotMotion<?>[] motions = new RobotMotion<?>[3];
        for (int i=0; i<3; i++)
          motions[i] = type==0 ? ptp(jp[(i+1)%3]) : type==1 ? ptp(frames[(i+1)%3]) : lin(frames[(i+1)%3]);
        String log = "task04_"+type;
        FileLogger.startLogging(log, FileLogger.Fields.TIME, FileLogger.Fields.TRANSLATIONAL_DISTANCE,
            FileLogger.Fields.ROTATIONAL_DISTANCE, FileLogger.Fields.JOINT_SPACE_DISTANCE);
        try { tool.move(new MotionBatch(motions).setBlendingCart(10).setJointVelocityRel(.2)); }
        finally { FileLogger.stopLogging(); }
        double[] totals = new double[3];
        var lines = Files.readAllLines(Paths.get(DataHandler.DataPath,"logs",log+".log"));
        for (String line : lines) {
          if (!line.matches("[0-9].*")) continue;
          String[] cells = line.trim().split("\\s+");
          require(cells.length == 4, "Task 4 log columns");
          for (int j=0;j<3;j++) {
            double value = Double.parseDouble(cells[j+1]);
            require(Double.isFinite(value) && value >= 0, "Task 4 finite distance");
            totals[j] += value;
          }
        }
        require(totals[0] > 10 && totals[1] > .01 && totals[2] > .1, "Task 4 records actual motion distances");
        require(frames[0].distanceTo(SunriseConnector.getTcpFrame()) < 2, "Task 4 batch endpoint");
      }

      // A 12 N force must fire a 10 N break, regardless of the SDK's default
      // 10 N uncertainty tolerance. Readings default to the measuring frame.
      robot.move(ptp(new JointPosition(0,.5,0,-1,0,.8,Math.PI/2)));
      JointImpedanceControlMode stiff = new JointImpedanceControlMode(2000,2000,2000,2000,1000,800,500);
      ForceCondition condition = ForceCondition.createNormalForceCondition(null, robot.getRootFrame(), CoordinateAxis.Y, 10);
      IMotionContainer hold = tool.moveAsync(positionHold(stiff,-1,TimeUnit.SECONDS).breakWhen(condition));
      bridge.call("force", "force", new double[] {0,12,0});
      hold.await(10,TimeUnit.SECONDS);
      require(hold.hasFired(condition), "10 N break fires at 12 N");
      Thread.sleep(400);
      var world = robot.getExternalForceTorque(tool.getDefaultMotionFrame(),robot.getRootFrame()).getForce();
      var local = robot.getExternalForceTorque(tool.getDefaultMotionFrame()).getForce();
      Matrix r = SunriseConnector.getTcpFrame().transformationFromWorld().getRotationMatrix();
      for (int col=0;col<3;col++) {
        double expected=0;
        for(int row=0;row<3;row++) expected += r.get(row,col)*world.get(row);
        require(Math.abs(local.get(col)-expected) < .5, "Force expressed in requested frame");
      }
      bridge.call("force", "force", new double[3]);
      SunriseConnector.goFront();
      System.out.println("EXERCISE REQUIREMENTS PASS");
    }
  }
}
