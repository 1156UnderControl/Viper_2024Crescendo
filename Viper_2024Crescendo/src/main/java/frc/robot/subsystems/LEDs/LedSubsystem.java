package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue;
  private Timer timerBlink;
  private boolean firstBlink = true;
  private static LedSubsystem instance;

  public static LedSubsystem getInstance() {
    if (instance == null) {
      instance = new LedSubsystem();
    }
    return instance;
  }

  private LedSubsystem() {
    this.timerBlink.start();
    this.m_led = new AddressableLED(0);
    this.m_ledBuffer = new AddressableLEDBuffer(26);
    this.m_led.setLength(m_ledBuffer.getLength());
    this.m_led.setData(m_ledBuffer);
    this.m_led.start();
  }

  public void setLEDOff() {
    none();
    m_led.setData(m_ledBuffer);
  }

  private void none() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
    }
    firstBlink = true;
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

  public void setOrange() {
    orange();
    m_led.setData(m_ledBuffer);
  }

  private void orange() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 0, 165);
    }
  }

  public void setGreen() {
    green();
    m_led.setData(m_ledBuffer);
  }

  private void green() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 255);
    }
  }

  public void setRed() {
    red();
    m_led.setData(m_ledBuffer);
  }

  public void blinkRed() {
    red();
    m_led.setData(m_ledBuffer);
    none();
    m_led.setData(m_ledBuffer);
  }

  private void red() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 0, 0);
    }
  }
}
