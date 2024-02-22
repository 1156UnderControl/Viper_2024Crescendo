package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue;

  public LedSubsystem() {
    this.m_led = new AddressableLED(0);
    this.m_ledBuffer = new AddressableLEDBuffer(60);
    this.m_led.setLength(m_ledBuffer.getLength());
    this.m_led.setData(m_ledBuffer);
    this.m_led.start();
  }

  public void setRainbow() {
    rainbow();
    m_led.setData(m_ledBuffer);
  }

  private void rainbow() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Define o valor do pixel
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    m_rainbowFirstPixelHue += 3;
    // Verifica Limites
    m_rainbowFirstPixelHue %= 180;
  }
}
