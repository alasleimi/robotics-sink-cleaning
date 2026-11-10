package sinkcleaning;

import com.google.gson.*;
import com.kuka.roboticsAPI.controllerModel.sunrise.ISafetyState;
import com.kuka.roboticsAPI.deviceModel.*;
import com.kuka.roboticsAPI.geometricModel.*;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.sensorModel.*;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.Arrays;

/** Explicit list of recorded SDK channels, separate from the recording loop. */
final class RecordingTelemetry {
    private final DemoRobot app;
    private final OptionalTelemetry optional;
    private final Gson json = new Gson();
    private double nextExtra, nextStatus;

    RecordingTelemetry(DemoRobot app, OptionalTelemetry optional) {
        this.app = app;
        this.optional = optional;
    }

    void configuration(Path folder, String calibration, JsonObject metadata) throws IOException {
        if (calibration != null && !calibration.equals("-")) {
            JsonObject sink = optional.read("sink_calibration", () -> {
                try (Reader input = Files.newBufferedReader(Paths.get(calibration), StandardCharsets.UTF_8)) {
                    JsonObject value = JsonParser.parseReader(input).getAsJsonObject();
                    double[] pose = json.fromJson(value.get("sink_pose"), double[].class);
                    checkPose(pose);
                    return value;
                }
            }, value -> value);
            if (sink != null) {
                metadata.add("sink_pose", sink.get("sink_pose"));
                metadata.addProperty("calibration_file", Paths.get(calibration).toAbsolutePath().toString());
            }
        }
        for (String resource : new String[] {"RoboticsAPI.data.xml", "RoboticsAPI.config.xml", "StationSetup.cat"}) {
            optional.read("configuration_file_" + resource, () -> {
                try (InputStream input = Record.class.getResourceAsStream("/" + resource)) {
                    if (input == null) throw new FileNotFoundException(resource);
                    String name = resource.equals("RoboticsAPI.data.xml") ? "tool-and-application-data.xml" : resource;
                    Files.copy(input, folder.resolve(name));
                    return name;
                }
            }, JsonPrimitive::new);
        }
        double[] tcp = optional.read("tcp_offset", () -> pose(app.tool.getDefaultMotionFrame()), OptionalTelemetry::numbers);
        if (tcp != null) metadata.add("tcp", json.toJsonTree(tcp));
        optional.read("robot_name", () -> app.robot.getName(), JsonPrimitive::new);
        optional.read("controller_name", () -> app.robot.getController().getName(), JsonPrimitive::new);
        optional.read("torque_sensor_limits_Nm", () -> app.robot.getTorqueSensorLimits(), OptionalTelemetry::numbers);
        optional.read("joint_limits_rad", () -> {
            JointLimits limits = app.robot.getJointLimits();
            JsonObject value = new JsonObject();
            value.add("min", OptionalTelemetry.numbers(limits.getMinJointPosition().get()));
            value.add("max", OptionalTelemetry.numbers(limits.getMaxJointPosition().get()));
            return value;
        }, value -> value);
        optional.read("tool_load", () -> {
            LoadData load = app.tool.getLoadData();
            JsonObject value = new JsonObject();
            value.add("mass_kg", OptionalTelemetry.numbers(load.getMass()));
            value.add("centre_of_mass_pose", OptionalTelemetry.numbers(Pose.pose(load.getCenterOfMass())));
            value.add("inertia_kg_m2", OptionalTelemetry.numbers(load.getInertia().getJX(), load.getInertia().getJY(), load.getInertia().getJZ()));
            return value;
        }, value -> value);
    }

    double[] sample(double time, int index) throws IOException {
        optional.sample(index);
        double[] row = new double[22];
        Arrays.fill(row, Double.NaN); // Missing optional data is never a fabricated zero.
        row[0] = time;
        Frame frame = app.robot.getCurrentCartesianPosition(app.tool.getDefaultMotionFrame(), app.robot.getRootFrame());
        double[] tcp = pose(frame);
        checkPose(tcp); // A valid measured tool path is essential for learning/replay.
        System.arraycopy(tcp, 0, row, 1, 7);

        double[] joints = optional.read("joint_position_rad", () -> {
            double[] value = app.robot.getCurrentJointPosition().get();
            if (value.length != 7) throw new IllegalStateException("Expected seven joints");
            return value;
        }, OptionalTelemetry::numbers);
        if (joints != null) System.arraycopy(joints, 0, row, 8, 7);

        ForceSensorData wrench = optional.read("tcp_wrench_base_N_Nm",
                () -> app.robot.getExternalForceTorque(app.tool.getDefaultMotionFrame(), app.robot.getRootFrame()),
                value -> OptionalTelemetry.numbers(value.getForce().getX(), value.getForce().getY(), value.getForce().getZ(),
                        value.getTorque().getX(), value.getTorque().getY(), value.getTorque().getZ()));
        if (wrench != null) {
            for (int axis = 0; axis < 3; axis++) {
                row[15 + axis] = wrench.getForce().get(axis);
                row[18 + axis] = wrench.getTorque().get(axis);
            }
        }
        if (time >= nextExtra) {
            extra(frame, wrench);
            nextExtra = time + .1; // Extra channels: at most 10 Hz, not every 50 Hz path sample.
        }
        if (time >= nextStatus) {
            status();
            nextStatus = time + 1.;
        }
        return row;
    }

    private void extra(Frame measured, ForceSensorData wrench) throws IOException {
        TorqueSensorData measuredTorque = optional.read("measured_joint_torque_Nm", app.robot::getMeasuredTorque,
                value -> OptionalTelemetry.numbers(value.getTorqueValues()));
        TorqueSensorData externalTorque = optional.read("external_joint_torque_Nm", app.robot::getExternalTorque,
                value -> OptionalTelemetry.numbers(value.getTorqueValues()));
        if (measuredTorque != null) optional.read("measured_torque_sdk_timestamp_ns", measuredTorque::getTimestamp, JsonPrimitive::new);
        if (externalTorque != null) optional.read("external_torque_sdk_timestamp_ns", externalTorque::getTimestamp, JsonPrimitive::new);
        optional.read("commanded_joint_position_rad", () -> app.robot.getCommandedJointPosition().get(), OptionalTelemetry::numbers);
        optional.read("commanded_tcp_pose", () -> pose(app.robot.getCommandedCartesianPosition(
                app.tool.getDefaultMotionFrame(), app.robot.getRootFrame())), OptionalTelemetry::numbers);
        optional.read("flange_pose", () -> pose(app.robot.getCurrentCartesianPosition(
                app.robot.getFlange(), app.robot.getRootFrame())), OptionalTelemetry::numbers);
        optional.read("redundancy", () -> {
            LBRE1Redundancy redundancy = new LBRE1Redundancy(measured.getRedundancyInformationForDevice(app.robot).getAllParameters());
            JsonObject value = new JsonObject();
            value.add("e1_rad", OptionalTelemetry.numbers(redundancy.getE1()));
            value.addProperty("status", redundancy.getStatus());
            value.addProperty("turn", redundancy.getTurn());
            return value;
        }, value -> value);
        if (wrench != null) {
            optional.read("wrench_inaccuracy_N_Nm", () -> OptionalTelemetry.numbers(
                    wrench.getForceInaccuracy().getX(), wrench.getForceInaccuracy().getY(), wrench.getForceInaccuracy().getZ(),
                    wrench.getTorqueInaccuracy().getX(), wrench.getTorqueInaccuracy().getY(), wrench.getTorqueInaccuracy().getZ()), value -> value);
            // SDK documents this as an internal system-dependent nanosecond clock.
            optional.read("wrench_sdk_timestamp_ns", wrench::getTimestamp, JsonPrimitive::new);
        }
    }

    private void status() throws IOException {
        optional.read("safety_state", () -> {
            ISafetyState state = app.robot.getSafetyState();
            JsonObject value = new JsonObject();
            value.addProperty("operation_mode", state.getOperationMode().name());
            value.addProperty("safety_stop", state.getSafetyStopSignal().name());
            value.addProperty("external_emergency_stop", state.getEmergencyStopEx().name());
            value.addProperty("internal_emergency_stop", state.getEmergencyStopInt().name());
            value.addProperty("operator_safety", state.getOperatorSafetyState().name());
            value.addProperty("enabling_device", state.getEnablingDeviceState().name());
            value.addProperty("axes_position_referenced", state.areAllAxesPositionReferenced());
            value.addProperty("axes_torque_referenced", state.areAllAxesGMSReferenced());
            return value;
        }, value -> value);
        optional.read("joint_temperatures_C", () -> {
            LbrJointTemperatureSensorData temperatures = app.robot.getTemperatures().getJointTemperatureSensorData();
            JsonObject value = new JsonObject();
            value.add("motor_1", json.toJsonTree(temperatures.getTemperaturesMotor1()));
            value.add("motor_2", json.toJsonTree(temperatures.getTemperaturesMotor2()));
            value.add("torque_sensor_1", json.toJsonTree(temperatures.getTemperaturesGms1()));
            value.add("torque_sensor_2", json.toJsonTree(temperatures.getTemperaturesGms2()));
            return value;
        }, value -> value);
        optional.read("drive_temperatures_C", () -> {
            LbrPdsTemperatureSensorData temperatures = app.robot.getTemperatures().getPdsTemperatureSensorData();
            JsonObject value = new JsonObject();
            value.add("rcu_1", json.toJsonTree(temperatures.getTemperaturesRcu1()));
            value.add("rcu_2", json.toJsonTree(temperatures.getTemperaturesRcu2()));
            value.add("fcu_motor", json.toJsonTree(temperatures.getTemperaturesFcuMotor()));
            value.add("fcu_brake", json.toJsonTree(temperatures.getTemperaturesFcuBrake()));
            return value;
        }, value -> value);
        optional.read("mastered", () -> app.robot.isMastered(), JsonPrimitive::new);
        optional.read("active_motion", () -> app.robot.hasActiveMotionCommand(), JsonPrimitive::new);
        optional.read("program_override", () -> app.robot.getController().getProgramOverride(), value -> OptionalTelemetry.numbers(value));
    }

    private static double[] pose(AbstractFrame frame) {
        return Pose.pose(Transformation.of(frame.getTransformationProvider().getTransformation()));
    }

    private static void checkPose(double[] pose) {
        if (pose == null || pose.length != 7) throw new IllegalArgumentException("Expected XYZ and XYZW");
        OptionalTelemetry.numbers(pose);
        double norm = 0;
        for (int i = 3; i < 7; i++) norm += pose[i] * pose[i];
        if (Math.abs(norm - 1) > .01) throw new IllegalArgumentException("Invalid orientation quaternion");
    }
}
