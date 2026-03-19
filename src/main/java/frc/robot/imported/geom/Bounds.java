// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.imported.geom;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

public record Bounds(double minX, double maxX, double minY, double maxY) {
  /** Whether the translation is contained within the bounds. */
  public boolean contains(Translation2d translation) {
    return translation.getX() >= minX()
        && translation.getX() <= maxX()
        && translation.getY() >= minY()
        && translation.getY() <= maxY();
  }

  /** Clamps the translation to the bounds. */
  public Translation2d clamp(Translation2d translation) {
    return new Translation2d(
        MathUtil.clamp(translation.getX(), minX(), maxX()),
        MathUtil.clamp(translation.getY(), minY(), maxY()));
  }
}
