package frc.robot.subsystems.ledFolder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class led extends SubsystemBase {
   private AddressableLED m_led;
   private AddressableLEDBuffer m_led;
   // Store what the last hue of the first pixle is
   private int m_rainbowFirstPixelHue;

public led(){
    // PWM port 9
    // Must be PWM header, not MXP or DIO
    m_led = new AddressableLED(9);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update the data
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
   
} 

public void setLEDRed(){
 for (var i = 0; i < m_ledBuffer.getLength.();++) {
    // Sets the specific LED to the RGB values for red
    m_ledBuffer.setRGB(i, 255, 0, 0);
   }
   m_led.setData(m_ledBuffer);
}

 private void rainbow() {
    //For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
        // Set the value
        m_ledBuff.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
 }
}