package simproxy;

import java.security.Permission;

/**
 * Java 17 is pinned for this simulator JVM. Block accidental SDK controller connections, including
 * calls from threads outside Mockito's thread-local static interception.
 */
@SuppressWarnings("removal")
final class NetworkGuard extends SecurityManager {
  static synchronized void install() {
    if (!(System.getSecurityManager() instanceof NetworkGuard))
      System.setSecurityManager(new NetworkGuard());
  }

  @Override
  public void checkPermission(Permission permission) {
    if (permission instanceof java.net.URLPermission)
      checkConnect(java.net.URI.create(permission.getName()).getHost(), -1);
  }

  @Override
  public void checkPermission(Permission permission, Object context) {
    checkPermission(permission);
  }

  @Override
  public void checkConnect(String host, int port) {
    if (!"127.0.0.1".equals(host)
        && !"localhost".equals(host)
        && !"::1".equals(host)
        && !"[::1]".equals(host))
      throw new SecurityException("Simulator JVM blocks non-loopback connections: " + host);
  }

  @Override
  public void checkConnect(String host, int port, Object context) {
    checkConnect(host, port);
  }
}
