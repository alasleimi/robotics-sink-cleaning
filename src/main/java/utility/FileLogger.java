package utility;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.HashSet;
import java.util.Locale;
import java.util.Set;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;

import robot.SunriseConnector;

public class FileLogger extends Thread {

	/**
	 * Possible fields to be logged to a file.
	 */
	public enum Fields {
		/**
		 * Time in ms since start of logging.
		 */
		TIME,

		/**
		 * Current joint position in rad [7 values].
		 */
		JOINT_POSITION,

		/**
		 * Current tcp translation in mm [3 values].
		 */
		TRANSLATION,

		/**
		 * Current tcp orientation in deg [3 values].
		 */
		ORIENTATION,

		/**
		 * Joint space distance to the last log entry [Euclidean norm].
		 */
		JOINT_SPACE_DISTANCE,

		/**
		 * Shortest angle between current and previous log entry tcp orientations.
		 */
		ROTATIONAL_DISTANCE,

		/**
		 * Distance between current and previous log entry tcp translations [Euclidean norm].
		 */
		TRANSLATIONAL_DISTANCE,

		/**
		 * Measured motor torques.for each joint in [Nm].
		 */
		MEASURED_TORQUES,

		/**
		 * Calculated external torques for each joint in [Nm].
		 */
		EXTERNAL_TORQUES,

		/**
		 * Calculated Cartesian forces and torques at the TCP in [N] and [Nm]. First three values are XYZ forces, second
		 * three values are ABC torques.
		 */
		EXTERNAL_TCP_FORCES_TORQUES,
	}

	private boolean active = false;

	private boolean finished = false;

	private SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd_HHmmss");

	private String filename = null;

	private Set<Fields> fieldsToWrite = new HashSet<Fields>();

	/**
	 * Create a file logger with specified fields. If not set by setFilename(), the file name will be the time when
	 * starting logging.
	 * 
	 * @param fields
	 *            list of fields to written to the log file. See FileLogger.Fields for reference.
	 */
	private FileLogger(Fields... fields) {
		System.out.println("logging fields:");
		for (Fields f : fields) {
			fieldsToWrite.add(f);
			System.out.println(f.name());
		}
	}

	/**
	 * Create a file logger all possible fields. If not set by setFilename(), the file name will be the time when
	 * starting logging.
	 * 
	 * @param intervall
	 *            time between log entries in ms.
	 */
	// public FileLogger(int intervall) {
	public FileLogger() {
		// this.intervall = intervall;
		System.out.println("logging all fields:");
		for (Fields f : Fields.values()) {
			fieldsToWrite.add(f);
		}
	}

	private static FileLogger logger = null;

	public static void startLogging(String filename, Fields... fields) {
		logger = new FileLogger(fields);
		logger.setFilename(filename);

		logger.activate();
	}

	/**
	 * Set the name of the log file.
	 * 
	 * @param filename
	 *            name of the file [ending .log will automatically be added]
	 */
	public void setFilename(String filename) {
		this.filename = DataHandler.DataPath + "/logs/" + filename + ".log";
	}

	private void stopInternal() {
		active = false;
		finished = true;
	}

	/**
	 * Stop logging and disable this FileLogger instance.
	 */
	public static void stopLogging() {
		logger.stopInternal();

		try {
			logger.join();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		logger = null;
	}

	private void activate() {
		if (active && !finished) {
			System.err.println("Logging has already been started!");
			return;
		}
		active = true;

		File fileObject = new File(DataHandler.DataPath + "/logs/");
		fileObject.mkdirs();

		if (filename == null) {
			setFilename(sdf.format(Calendar.getInstance().getTime()));
		}

		this.start();
	}

	/**
	 * Thread target run method. WILL BLOCK EXECUTION IF CALLED MANUALLY.
	 */
	@Override
	public void run() {
		System.out.println("Starting to log to file '" + filename + "' now");

		BufferedWriter writer = null;
		try {
			writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(filename, false), "utf-8"));

			Frame tcpFrame = null;
			Frame tcpFrameLast = null;
			double[] jointPos = null;
			double[] jointPosLast = null;

			TorqueSensorData tsd_int = null;
			TorqueSensorData tsd_ext = null;
			ForceSensorData fsd = null;

			String separator = "\t";

			long start = System.currentTimeMillis();
			long now = start;

			LBR robot = SunriseConnector.getRobot();

			for (Fields f : fieldsToWrite) {
				switch (f) {
				case TIME:
					writer.write("TIME");
					writer.write(separator);
					break;

				case JOINT_POSITION:
					if (fieldsToWrite.contains(Fields.JOINT_POSITION)) {
						for (int i = 0; i < 7; i++) {
							writer.write("J" + i);
							writer.write(separator);
						}
					}
					break;

				case TRANSLATION:
					if (fieldsToWrite.contains(Fields.TRANSLATION)) {
						writer.write("X" + separator);
						writer.write("Y" + separator);
						writer.write("Z" + separator);
					}
					break;
				case ORIENTATION:
					if (fieldsToWrite.contains(Fields.ORIENTATION)) {
						writer.write("A" + separator);
						writer.write("B" + separator);
						writer.write("C" + separator);
					}
					break;

				case JOINT_SPACE_DISTANCE:
					if (fieldsToWrite.contains(Fields.JOINT_SPACE_DISTANCE)) {
						writer.write("JS_DIST");
						writer.write(separator);
					}
					break;

				case TRANSLATIONAL_DISTANCE:
					if (fieldsToWrite.contains(Fields.TRANSLATIONAL_DISTANCE)) {
						writer.write("TRANS_DIST");
						writer.write(separator);
					}
					break;

				case ROTATIONAL_DISTANCE:
					if (fieldsToWrite.contains(Fields.ROTATIONAL_DISTANCE)) {
						writer.write("ROT_DIST");
						writer.write(separator);
					}
					break;

				case MEASURED_TORQUES:
					if (fieldsToWrite.contains(Fields.MEASURED_TORQUES)) {
						for (int i = 0; i < 7; i++) {
							writer.write("MT" + i);
							writer.write(separator);
						}
					}
					break;

				case EXTERNAL_TORQUES:
					if (fieldsToWrite.contains(Fields.EXTERNAL_TORQUES)) {
						for (int i = 0; i < 7; i++) {
							writer.write("ET" + i);
							writer.write(separator);
						}
					}
					break;

				case EXTERNAL_TCP_FORCES_TORQUES:
					if (fieldsToWrite.contains(Fields.EXTERNAL_TCP_FORCES_TORQUES)) {
						writer.write("FX" + separator);
						writer.write("FY" + separator);
						writer.write("FZ" + separator);
						writer.write("TA" + separator);
						writer.write("TB" + separator);
						writer.write("TC" + separator);
					}
					break;
				}
			}

			// end line
			writer.newLine();

			while (active) {
				now = System.currentTimeMillis();
				jointPos = robot.getCurrentJointPosition().get();
				tcpFrame = SunriseConnector.getTcpFrame();

				if (fieldsToWrite.contains(Fields.MEASURED_TORQUES))
					tsd_int = SunriseConnector.getRobot().getMeasuredTorque();
				if (fieldsToWrite.contains(Fields.EXTERNAL_TORQUES))
					tsd_ext = SunriseConnector.getRobot().getExternalTorque();
				if (fieldsToWrite.contains(Fields.EXTERNAL_TCP_FORCES_TORQUES))
					fsd = SunriseConnector.getRobot().getExternalForceTorque(null);

				for (Fields f : fieldsToWrite) {
					switch (f) {
					case TIME:
						if (fieldsToWrite.contains(Fields.TIME)) {
							writer.write("" + (now - start));
							writer.write(separator);
						}
						break;

					case JOINT_POSITION:
						if (fieldsToWrite.contains(Fields.JOINT_POSITION)) {
							for (int i = 0; i < 7; i++) {
								writer.write(String.format(Locale.US, "%.3f", jointPos[i]) + separator);
							}
						}
						break;

					case TRANSLATION:
						if (fieldsToWrite.contains(Fields.TRANSLATION)) {
							writer.write(String.format(Locale.US, "%.3f", tcpFrame.getX()) + separator);
							writer.write(String.format(Locale.US, "%.3f", tcpFrame.getY()) + separator);
							writer.write(String.format(Locale.US, "%.3f", tcpFrame.getZ()) + separator);
						}
						break;
					case ORIENTATION:
						if (fieldsToWrite.contains(Fields.ORIENTATION)) {
							writer.write(String.format(Locale.US, "%.3f", tcpFrame.getAlphaRad()) + separator);
							writer.write(String.format(Locale.US, "%.3f", tcpFrame.getBetaRad()) + separator);
							writer.write(String.format(Locale.US, "%.3f", tcpFrame.getGammaRad()) + separator);
						}
						break;

					case JOINT_SPACE_DISTANCE:
						if (fieldsToWrite.contains(Fields.JOINT_SPACE_DISTANCE)) {
							double dist = 0;
							if (jointPosLast != null) {
								for (int i = 0; i < jointPos.length; i++) {
									dist += Math.pow(jointPosLast[i] - jointPos[i], 2);
								}
							}
							writer.write(String.format(Locale.US, "%.5f", Math.sqrt(dist)) + separator);
						}
						break;

					case TRANSLATIONAL_DISTANCE:
						if (fieldsToWrite.contains(Fields.TRANSLATIONAL_DISTANCE)) {
							double dist = 0;
							if (tcpFrameLast != null) {
								dist = tcpFrameLast.distanceTo(tcpFrame);
							}
							writer.write(String.format(Locale.US, "%.5f", dist) + separator);
						}
						break;

					case ROTATIONAL_DISTANCE:
						if (fieldsToWrite.contains(Fields.ROTATIONAL_DISTANCE)) {
							double dist = 0;
							if (tcpFrameLast != null) {
								dist = tcpFrameLast.rotationalDistanceTo(tcpFrame);
							}
							writer.write(String.format(Locale.US, "%.5f", dist) + separator);
						}
						break;

					case MEASURED_TORQUES:
						if (fieldsToWrite.contains(Fields.MEASURED_TORQUES)) {
							tsd_int = SunriseConnector.getRobot().getMeasuredTorque();
							for (int i = 0; i < 7; i++) {
								writer.write(
										String.format(Locale.US, "%.3f", tsd_int.getTorqueValues()[i]) + separator);
							}
						}
						break;

					case EXTERNAL_TORQUES:
						if (fieldsToWrite.contains(Fields.EXTERNAL_TORQUES)) {
							tsd_int = SunriseConnector.getRobot().getMeasuredTorque();
							for (int i = 0; i < 7; i++) {
								writer.write(
										String.format(Locale.US, "%.3f", tsd_ext.getTorqueValues()[i]) + separator);
							}
						}
						break;

					case EXTERNAL_TCP_FORCES_TORQUES:
						if (fieldsToWrite.contains(Fields.EXTERNAL_TCP_FORCES_TORQUES)) {
							tsd_int = SunriseConnector.getRobot().getMeasuredTorque();
							for (int i = 0; i < 3; i++) {
								writer.write(String.format(Locale.US, "%.3f", fsd.getForce().get(i)) + separator);
							}

							for (int i = 0; i < 3; i++) {
								writer.write(String.format(Locale.US, "%.3f", fsd.getTorque().get(i)) + separator);
							}
						}
						break;
					}
				}

				// end line
				writer.newLine();

				jointPosLast = jointPos;
				tcpFrameLast = tcpFrame;
			}

		} catch (IOException e) {
			// report
		} finally {
			try {
				writer.close();
			} catch (Exception e) {
			}
		}
	}
}