package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

class LedMode {
  public static final String SOLID = "SOLID";
  public static final String RAINBOW = "RAINBOW";
  public static final String BLINK = "BLINK";  
}

public class LedSubsystem extends SubsystemBase implements ILed {
  private static LedSubsystem instance = null;

  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private int rainbowFirstPixelHue;

  private Timer blinkTimer = new Timer();

  private double blinkFrequency;

  private double blinkPeriod;

  private double halfBlinkPeriod;

  private int numberOfBlinks;

  private Color colorAfterBlink;

  private String mode;

  private Color color;

  public static LedSubsystem getInstance() {
    if (instance == null) {
      instance = new LedSubsystem();
    }
    return instance;
  }

  private LedSubsystem() {
    this.setBlinkFrequency(3);
    this.mode = LedMode.SOLID;
    this.color = LedColor.OFF;
    this.led = new AddressableLED(0);
    this.ledBuffer = new AddressableLEDBuffer(26);
    this.led.setLength(ledBuffer.getLength());
    this.led.setData(ledBuffer);
    this.led.start();
  }

  @Override
  public void periodic() {
    switch (this.mode) {
      case LedMode.SOLID:
        this.processSolidColor();
        break;
      case LedMode.RAINBOW:
        this.processRainbow();
        break;
      case LedMode.BLINK:
        processBlink();
        break;
    }
  }

  public void setBlinkFrequency(double frequency) {
    this.blinkFrequency = frequency;
    this.blinkPeriod = 1 / this.blinkFrequency;
    this.halfBlinkPeriod = this.blinkPeriod / 2;
  }

  public void turnOff() {
    this.setSolidColor(LedColor.OFF);
  }

  public void setSolidColor(Color color){
    this.mode = LedMode.SOLID;
    this.color = color;
    this.processSolidColor();
  }

  public void setBlink(Color color, int numberOfBlinks, Color colorAfterBlink) {
    this.mode = LedMode.BLINK;
    this.color = color;
    this.numberOfBlinks = numberOfBlinks;
    this.colorAfterBlink = colorAfterBlink;
    this.blinkTimer.reset();
    this.blinkTimer.start();
    this.processBlink();
  }

  public void setBlink(Color color, int numberOfBlinks) {
    this.setBlink(color, numberOfBlinks, LedColor.OFF);
  }

  public void setBlink(Color color) {
    this.setBlink(color, 0, LedColor.OFF);
  }

  public void setRainbow() {
    this.mode = LedMode.RAINBOW;
    this.color = LedColor.OFF;
    this.processRainbow();
  }

  private void processRainbow() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      final var hue = (this.rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      // Define o valor do pixel
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    this.rainbowFirstPixelHue += 3;
    // Verifica Limites
    this.rainbowFirstPixelHue %= 180;
    led.setData(ledBuffer);
  }

  private void processSolidColor(){
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, this.color.red, this.color.blue, this.color.green);
    }
    led.setData(ledBuffer);
  }

  private void processBlink(){
    double elapsedTime = this.blinkTimer.get();
    if (this.numberOfBlinks > 0){
      double elapsedBlinkPeriods = elapsedTime / this.blinkPeriod;
      if (elapsedBlinkPeriods > this.numberOfBlinks){
        this.setSolidColor(this.colorAfterBlink);
        return;
      }
    }
    if (elapsedTime % this.blinkPeriod < this.halfBlinkPeriod){
      this.setSolidColor(this.color);
    } else {
      this.setSolidColor(LedColor.OFF);
    }
  }
}
