package simproxy;

import com.kuka.roboticsAPI.geometricModel.*;
import com.kuka.roboticsAPI.geometricModel.math.*;

/** KUKA millimetres / rotation matrices <-> MuJoCo metres / XYZW quaternions. */
final class Poses {
  static double[] pose(AbstractFrame f) {
    return pose(f.transformationFromWorld());
  }

  static double[] pose(Transformation t) {
    Matrix m = t.getRotationMatrix();
    double x, y, z, w, trace = m.get(0, 0) + m.get(1, 1) + m.get(2, 2);
    if (trace > 0) {
      double s = Math.sqrt(trace + 1) * 2;
      w = s / 4;
      x = (m.get(2, 1) - m.get(1, 2)) / s;
      y = (m.get(0, 2) - m.get(2, 0)) / s;
      z = (m.get(1, 0) - m.get(0, 1)) / s;
    } else if (m.get(0, 0) > m.get(1, 1) && m.get(0, 0) > m.get(2, 2)) {
      double s = Math.sqrt(1 + m.get(0, 0) - m.get(1, 1) - m.get(2, 2)) * 2;
      w = (m.get(2, 1) - m.get(1, 2)) / s;
      x = s / 4;
      y = (m.get(0, 1) + m.get(1, 0)) / s;
      z = (m.get(0, 2) + m.get(2, 0)) / s;
    } else if (m.get(1, 1) > m.get(2, 2)) {
      double s = Math.sqrt(1 + m.get(1, 1) - m.get(0, 0) - m.get(2, 2)) * 2;
      w = (m.get(0, 2) - m.get(2, 0)) / s;
      x = (m.get(0, 1) + m.get(1, 0)) / s;
      y = s / 4;
      z = (m.get(1, 2) + m.get(2, 1)) / s;
    } else {
      double s = Math.sqrt(1 + m.get(2, 2) - m.get(0, 0) - m.get(1, 1)) * 2;
      w = (m.get(1, 0) - m.get(0, 1)) / s;
      x = (m.get(0, 2) + m.get(2, 0)) / s;
      y = (m.get(1, 2) + m.get(2, 1)) / s;
      z = s / 4;
    }
    return new double[] {t.getX() / 1000, t.getY() / 1000, t.getZ() / 1000, x, y, z, w};
  }

  static Transformation transformation(double[] p) {
    double x = p[3], y = p[4], z = p[5], w = p[6];
    Matrix m =
        Matrix.ofRowFirst(
            1 - 2 * y * y - 2 * z * z,
            2 * x * y - 2 * z * w,
            2 * x * z + 2 * y * w,
            2 * x * y + 2 * z * w,
            1 - 2 * x * x - 2 * z * z,
            2 * y * z - 2 * x * w,
            2 * x * z - 2 * y * w,
            2 * y * z + 2 * x * w,
            1 - 2 * x * x - 2 * y * y);
    return Transformation.of(Vector.of(p[0] * 1000, p[1] * 1000, p[2] * 1000), m);
  }

  static Frame frame(double[] p) {
    return new Frame(transformation(p));
  }

  static double[] matrix(AbstractFrame f) {
    Matrix m = f.transformationFromWorld().getRotationMatrix();
    double[] result = new double[9];
    for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) result[3 * r + c] = m.get(r, c);
    return result;
  }
}
