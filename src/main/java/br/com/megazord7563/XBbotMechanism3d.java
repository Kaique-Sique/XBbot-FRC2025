package br.com.megazord7563;

import br.com.megazord7563.util.geometry.GeomUtil;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({GeomUtil.class})
public class XBbotMechanism3d {
  private static XBbotMechanism3d instance;

  public static XBbotMechanism3d getInstance() {
    if (instance == null) {
      instance = new XBbotMechanism3d();
    }
    return instance;
  }

  public void log() {}
}
