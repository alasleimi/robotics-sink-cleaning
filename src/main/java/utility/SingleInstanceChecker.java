package utility;

import java.io.IOException;
import java.nio.channels.*;
import java.nio.file.*;

/** Hold an OS file lock; do not terminate another running robot application. */
public class SingleInstanceChecker extends Thread {
  private final FileChannel channel;
  private final FileLock lock;

  public SingleInstanceChecker() {
    setDaemon(true);
    try {
      Path folder = Paths.get(DataHandler.DataPath);
      Files.createDirectories(folder);
      channel =
          FileChannel.open(
              folder.resolve(".robotapplock"), StandardOpenOption.CREATE, StandardOpenOption.WRITE);
      FileLock acquired;
      try {
        acquired = channel.tryLock();
      } catch (OverlappingFileLockException e) {
        acquired = null;
      }
      if (acquired == null) {
        channel.close();
        throw new IllegalStateException(
            "An application already uses " + folder + ". Close it first.");
      }
      lock = acquired;
      Runtime.getRuntime()
          .addShutdownHook(
              new Thread(
                  new Runnable() {
                    public void run() {
                      try {
                        lock.release();
                        channel.close();
                      } catch (IOException ignored) {
                      }
                    }
                  }));
    } catch (IOException e) {
      throw new IllegalStateException("Cannot acquire application lock", e);
    }
  }

  @Override
  public void run() {
    /* The OS holds the lock until this JVM exits. */
  }
}
