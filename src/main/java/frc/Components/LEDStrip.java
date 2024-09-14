package frc.Components;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
    private AddressableLED ledStrip;
    private AddressableLEDBuffer ledConfigs;
    private int rainbowFirstPixelHue;
    private static int ledNum = 0;
    public boolean isCustomColor;

    public LEDStrip(int port, int length) {
        ledNum++;
        this.isCustomColor = false;
        this.ledStrip = new AddressableLED(port);
        this.ledConfigs = new AddressableLEDBuffer(length);
        this.ledStrip.setLength(length);
        this.ledStrip.setData(ledConfigs);
        this.ledStrip.start();
        for (int i = 0; i < ledConfigs.getLength(); i++) {
            ledConfigs.setRGB(i, 255, 0, 0);
        }

        this.rainbowFirstPixelHue = 0;
        Shuffleboard.getTab("SmartDashboard").add("Set LED " + ledNum + " Rainbow", new Command() {
            @Override
            public void initialize() {
                setMovingRainbow();
                isFinished();
            }
        });
        Shuffleboard.getTab("SmartDashboard").add("Set LED " + ledNum + " Neon", new Command() {
            @Override
            public void initialize() {
                setColorNeon();
                isFinished();
            }
        });

    }

    public void setColorNeon() {
        for (int i = 0; i < ledConfigs.getLength(); i += 4) {
            ledConfigs.setHSV(i, 259, 100, 27);
            ledConfigs.setHSV(i + 1, 324, 99, 83);
            ledConfigs.setHSV(i + 2, 58, 71, 98);
            ledConfigs.setHSV(i + 3, 180, 65, 80);
        }
        if (ledConfigs.getLength() % 4 != 0) {
            switch (ledConfigs.getLength() % 4) {
                case 1: {
                    ledConfigs.setHSV(ledConfigs.getLength() - 1, 259, 100, 27);
                }
                case 2: {
                    ledConfigs.setHSV(ledConfigs.getLength() - 2, 259, 100, 27);
                    ledConfigs.setHSV(ledConfigs.getLength() - 1, 324, 99, 83);
                }
                default: {
                    ledConfigs.setHSV(ledConfigs.getLength() - 3, 259, 100, 27);
                    ledConfigs.setHSV(ledConfigs.getLength() - 2, 324, 99, 83);
                    ledConfigs.setHSV(ledConfigs.getLength() - 1, 58, 71, 98);
                }
            }
        }
        isCustomColor = true;
    }

    private void setColorRainbow() {
        // For every pixel
        for (int i = 0; i < ledConfigs.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final int hue = (rainbowFirstPixelHue + (i * 180 / ledConfigs.getLength())) % 180;
            // Set the value
            ledConfigs.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }

    public void setMovingRainbow() {
        setColorRainbow();
        isCustomColor = true;
    }

    public void setColorRGB(int r, int g, int b) {
        for (int i = 0; i < ledConfigs.getLength(); i++) {
            ledConfigs.setRGB(i, r, g, b);
        }
        isCustomColor = false;
    }

    public void setColorHSV(int h, int s, int v) {
        for (int i = 0; i < ledConfigs.getLength(); i++) {
            ledConfigs.setHSV(i, h, s, v);
        }
        isCustomColor = false;
    }

    @Override
    public void periodic() {
    }
}
