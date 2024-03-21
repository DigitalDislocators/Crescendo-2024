package frc.robot.util.led;

import edu.wpi.first.wpilibj.util.Color;

public interface LEDParent {

    public enum TranslateDirection {
        FORWARD,
        REVERSE
    }
    
    public Color[] getLEDBuffer();

    public Color getLED(int index);

    public Color[] getPixelBuffer();

    public Color getPixel(int index);

    public Color[] getColorBuffer();

    public Color getColor(int index);

    public double[] getValueBuffer();

    public double getValue(int index);

    public int getLength();

    public boolean isReversed();
    public void setReversed(boolean isReversed);

    public void setBrightness(double brightness);

    public void setColor(Color color);

    public void setColor(Color color, int index);

    public void translateColors(TranslateDirection direction, Color... voidColors);

    public void setValue(double value);

    public void setValue(double value, int index);

    public void translateValues(TranslateDirection direction, double... voidValues);

}
