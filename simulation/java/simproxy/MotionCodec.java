package simproxy;

import com.google.gson.*;
import com.kuka.roboticsAPI.conditionModel.*;
import com.kuka.roboticsAPI.geometricModel.*;
import com.kuka.roboticsAPI.motionModel.*;
import com.kuka.roboticsAPI.motionModel.controlModeModel.*;
import java.util.*;
import utility.DataHandler;

/** Deliberately bounded Sunrise subset: unknown commands fail before submission. */
final class MotionCodec {
  final List<ICondition> conditions = new ArrayList<>();
  final java.util.function.Function<AbstractFrame, String> movingFrame;

  MotionCodec(java.util.function.Function<AbstractFrame, String> movingFrame) {
    this.movingFrame = movingFrame;
  }

  private void reference(JsonObject result, String key, AbstractFrame ref, boolean matrix) {
    String moving = movingFrame.apply(ref);
    if (moving != null) result.addProperty(key + "Which", moving);
    else {
      if (!ref.hasStaticPathTo(World.Current.getRootFrame()))
        throw new UnsupportedOperationException("Custom moving reference frames are not supported");
      result.add(key, Bridge.JSON.toJsonTree(matrix ? Poses.matrix(ref) : Poses.pose(ref)));
    }
  }

  JsonObject encode(IMotion motion, String which) {
    return encode(motion, which, new JsonObject());
  }

  private JsonObject encode(IMotion motion, String which, JsonObject inherited) {
    if (!(motion instanceof KROSMotion<?> m)) throw unsupported(motion);
    if (m.hasTrigger() || m.hasMotionOverlay())
      throw new UnsupportedOperationException(
          "Simulator does not support motion triggers/overlays");
    JsonObject out = inherited.deepCopy();
    out.addProperty("which", which);
    if (m instanceof RobotMotion<?> r) {
      if (r.hasJointVelocityRel()) out.addProperty("jointVelocity", r.getJointVelocityRel());
      if (r.hasJointAccelerationRel())
        out.addProperty("jointAcceleration", r.getJointAccelerationRel());
      if (r.hasBlendingRel()) out.addProperty("blendRel", r.getBlendingRel());
      if (r.hasBlendingCart()) out.addProperty("blendCart", r.getBlendingCart() / 1000);
      if (r.hasBlendingOri())
        throw new UnsupportedOperationException("Orientation-only blending is not supported");
    }
    if (m.getMode() != null) {
      JsonObject mode = control(m.getMode());
      if (mode != null) out.add("control", mode);
      else out.remove("control");
    }
    JsonArray breaks = new JsonArray();
    if (out.has("conditions")) for (JsonElement e : out.getAsJsonArray("conditions")) breaks.add(e);
    for (ICondition c : m.getConditions()) {
      if (!conditions.contains(c)) conditions.add(c);
      JsonObject b = condition(c);
      b.addProperty("id", conditions.indexOf(c));
      breaks.add(b);
    }
    out.add("conditions", breaks);
    if (m instanceof MotionBatch batch) {
      JsonArray items = new JsonArray();
      for (RobotMotion<?> child : batch.getMotions()) items.add(encode(child, which, out));
      return Bridge.object("type", "batch", "motions", items);
    }
    if (m instanceof PTP p) {
      out.addProperty("type", "ptp");
      out.add("q", Bridge.JSON.toJsonTree(p.getDestination().get()));
    } else if (m instanceof PTPHome) {
      out.addProperty("type", "ptp");
      out.add("q", Bridge.JSON.toJsonTree(new double[7]));
    } else if (m instanceof CartesianPTP p) {
      out.addProperty("type", "ptp");
      target(out, p.getDestination());
    } else if (m instanceof LIN p) {
      out.addProperty("type", "lin");
      target(out, p.getDestination());
    } else if (m instanceof RelativeLIN p) {
      out.addProperty("type", "relative");
      out.add("offset", Bridge.JSON.toJsonTree(Poses.pose(p.getOffset())));
      AbstractFrame ref = p.getReferenceFrame();
      if (ref != null) reference(out, "reference", ref, false);
    } else if (m instanceof PositionHold h) {
      out.addProperty("type", "hold");
      out.addProperty("seconds", h.getTimeoutMillis() / 1000.0);
    } else throw unsupported(m);
    if (m instanceof SplineMotionCP<?> p) {
      if (p.hasCartVelocity()) out.addProperty("cartVelocity", p.getCartVelocity() / 1000);
      if (p.hasCartAcceleration())
        out.addProperty("cartAcceleration", p.getCartAcceleration() / 1000);
      if (p.hasCartJerk()
          || p.hasOrientationJerk()
          || p.hasOrientationType()
          || p.hasOrientationVelocity()
          || p.hasOrientationAcceleration())
        throw new UnsupportedOperationException(
            "Spline jerk/orientation limits and modes are not supported");
    }
    return out;
  }

  static void target(JsonObject o, AbstractFrame frame) {
    if (!frame.hasStaticPathTo(World.Current.getRootFrame()))
      throw new UnsupportedOperationException(
          "Copy a moving destination into a world Frame before submitting it");
    o.add("pose", Bridge.JSON.toJsonTree(Poses.pose(frame)));
    if (frame.hasAdditionalParameter(DataHandler.SIM_JOINT_SEED))
      o.add(
          "seed", Bridge.JSON.toJsonTree(frame.getAdditionalParameter(DataHandler.SIM_JOINT_SEED)));
    if (!frame.getRedundancyInformation().isEmpty())
      throw new UnsupportedOperationException(
          "KUKA E1/status/turn constraints are not simulated. Teach simulator frames to preserve"
              + " their joint seed, or use explicit joint targets.");
  }

  private JsonObject control(IMotionControlMode mode) {
    if (mode instanceof PositionControlMode) return null;
    if (mode instanceof JointImpedanceControlMode j)
      return Bridge.object(
          "kind", "joint", "stiffness", j.getStiffness(), "damping", j.getDamping());
    if (mode instanceof CartesianImpedanceControlMode c) {
      if (c.getSpringPosition() != null)
        throw new UnsupportedOperationException(
            "A custom impedance spring frame is not implemented");
      JsonObject result =
          Bridge.object(
              "kind",
              "cartesian",
              "stiffness",
              c.getStiffness(),
              "damping",
              c.getDamping(),
              "nullStiffness",
              c.hasNullSpaceStiffness() ? c.getNullSpaceStiffness() : 0.,
              "nullDamping",
              c.hasNullSpaceDamping() ? c.getNullSpaceDamping() : .3);
      if (c.getReferenceSystem() != null)
        reference(result, "rotation", c.getReferenceSystem(), true);
      CartesianImpedanceControlMode defaults = new CartesianImpedanceControlMode();
      if (!Arrays.equals(c.getMaxCartesianVelocity(), defaults.getMaxCartesianVelocity())
          || !Arrays.equals(c.getMaxPathDeviation(), defaults.getMaxPathDeviation())
          || c.hasMaxControlForceStopCondition())
        throw new UnsupportedOperationException(
            "Custom impedance velocity/path limits and stop-on-force limits are not supported");
      if (!Arrays.equals(c.getMaxControlForce(), defaults.getMaxControlForce()))
        result.add("maxForce", Bridge.JSON.toJsonTree(c.getMaxControlForce()));
      double[] extra = c.getAdditionalControlForce();
      if (extra != null && Arrays.stream(extra).anyMatch(v -> v != 0))
        throw new UnsupportedOperationException("Additional control force is not implemented");
      return result;
    }
    throw unsupported(mode);
  }

  private JsonObject condition(ICondition c) {
    if (c instanceof ForceCondition f) {
      JsonObject o =
          Bridge.object(
              "type",
              "force",
              "axes",
              f.getCoordinateAxes().stream().mapToInt(Enum::ordinal).toArray(),
              "threshold",
              f.getThreshold(),
              "tolerance",
              f.getTolerance());
      if (f.getOrientationFrame() != null) reference(o, "rotation", f.getOrientationFrame(), true);
      return o;
    }
    if (c instanceof JointTorqueCondition t)
      return Bridge.object(
          "type",
          "jointTorque",
          "joint",
          t.getJoint().ordinal(),
          "min",
          t.getMinTorque(),
          "max",
          t.getMaxTorque());
    if (c instanceof AbstractComplexCondition a) {
      String type = c instanceof OrCondition ? "or" : c instanceof AndCondition ? "and" : null;
      if (type == null) throw unsupported(c);
      JsonArray children = new JsonArray();
      for (ICondition child : a.getConditions()) children.add(condition(child));
      return Bridge.object("type", type, "children", children);
    }
    throw unsupported(c);
  }

  static UnsupportedOperationException unsupported(Object o) {
    return new UnsupportedOperationException(
        "Simulator does not support " + o.getClass().getName());
  }
}
