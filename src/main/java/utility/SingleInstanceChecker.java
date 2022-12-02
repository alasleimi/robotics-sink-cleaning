package utility;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.RandomAccessFile;
import java.nio.channels.FileChannel;
import java.nio.channels.FileLock;

import com.kuka.common.ThreadUtil;

public class SingleInstanceChecker extends Thread {

	private static File file;
	private static FileChannel fileChannel;
	private static FileLock lock;

	private static String fname = ".robotapplock";

	private static int ID;
	private static boolean last = true;

	public SingleInstanceChecker() {

		file = new File(fname);
		try {
			boolean newFile = file.createNewFile();
			writeNewIdToFile(file, newFile);
		} catch (IOException e) {
			e.printStackTrace();
		}

//		boolean alreadyRunning = checkIfAlreadyRunning(file);
//
//		while (alreadyRunning) {
//			// System.out.println("trying to acquire lock..");
//			alreadyRunning = checkIfAlreadyRunning(file);
//			ThreadUtil.milliSleep(100);
//		}
		// System.out.println("lock acquired");
	}

//	@SuppressWarnings("resource")
//	public static boolean checkIfAlreadyRunning(File file) {
//		try {
//
//			boolean lockAcquired = false;
//
//			fileChannel = new RandomAccessFile(file, "rw").getChannel();
//			lock = fileChannel.tryLock();
//
//			if (lock == null) {
//				fileChannel.close();
//				lockAcquired = false;
//				// System.err.println(
//				// "Another robot application is already running. Close all other applications and retry!");
//				// System.exit(0);
//				return true;
//			}
//
//			ShutdownHook shutdownHook = new ShutdownHook();
//			Runtime.getRuntime().addShutdownHook(shutdownHook);
//
//			return lockAcquired;
//		} catch (IOException e) {
//			System.err.println(e.getMessage());
//			e.printStackTrace();
//			return false;
//		}
//	}

	private static void writeNewIdToFile(File file, boolean newFile) throws FileNotFoundException, IOException {
		// create file reader
		FileReader reader = new FileReader(file);
		BufferedReader bufferedReader = new BufferedReader(reader);

		if (!newFile) {
			String line = bufferedReader.readLine();
			reader.close();
			ID = Integer.parseInt(line) + 1;
		} else {
			ID = 1;
		}

		// System.out.println("ID: " + ID);

		PrintWriter pw = new PrintWriter(file.getPath());
		pw.close();
		FileWriter writer = new FileWriter(file, true);
		writer.write("" + (ID));
		writer.close();
	}

	public static void unlockFile() {
		try {
			if (lock != null)
				lock.release();
			if (last) {
				fileChannel.close();
				file.delete();
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	static class ShutdownHook extends Thread {
		public void run() {
			unlockFile();
		}
	}

	@Override
	public void run() {
		String line = "";
		RandomAccessFile raf = null;

		while (true) {
			try {
				// File file = new File(fname);
				raf = new RandomAccessFile(file, "r");
				line = raf.readLine();
				raf.seek(0);

				// System.out.println("readLine: " + line);

				if (Integer.parseInt(line) > ID) {
					System.out.println("New robot application started; exiting.");
					raf.close();
					last = false;
					System.exit(0);
				}
			} catch (IOException e) {
				e.printStackTrace();
			}

			ThreadUtil.milliSleep(200);
		}
	}

}
