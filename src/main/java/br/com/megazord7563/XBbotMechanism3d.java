// Copyright (c) 2026 Megazord 7563
// All rights reserved.
//
// This file is part of the XBbot FRC robot code.
// See the LICENSE file in the project root for license terms.

package br.com.megazord7563;

import br.com.megazord7563.util.geometry.GeomUtil;
import edu.wpi.first.math.geometry.Pose3d;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class XBbotMechanism3d {
  private static XBbotMechanism3d instance;

  public static XBbotMechanism3d getInstance() {
    if (instance == null) {
      instance = new XBbotMechanism3d();
    }
    return instance;
  }

  public void log() {
    var model = new Pose3d();
    var model_0 = new Pose3d();
    var model_1 = new Pose3d();
    var model_2 = new Pose3d();

    Logger.recordOutput("/Mechanism3d/Components", model, model_0, model_1, model_2);
  }
}
