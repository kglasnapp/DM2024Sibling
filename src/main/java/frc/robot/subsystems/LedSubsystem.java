package frc.robot.subsystems;

import static frc.robot.Util.logf;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import static frc.robot.utilities.Util.logf;

public class LedSubsystem extends SubsystemBase {
    boolean changed = false;
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    public LedSubsystem() {

        initNeoPixel();
        setColors(Leds.GRABBER, 0, 0, 0);
    }

    public enum Leds {
        GRABBER(0, 144);

        public final int val;
        public final int number;

        private Leds(int val, int number) {
            this.val = val;
            this.number = number;
        }
    };

    enum State {
        SET_IDLE_COLOR, IDLE, FLASH, SET_HOLDING_COLOR, HOLDING, SET_INDEXED_COLOR, INDEXED, SET_ERROR_COLOR, ERROR
    };

    State state = State.SET_IDLE_COLOR;

    int counter = 0;
    boolean light = false;
    boolean on = true;

    @Override
    public void periodic() {
        if (changed) {
            m_led.setData(m_ledBuffer);
        }
        changed = false;
    }

    private void initNeoPixel() {
        m_led = new AddressableLED(9);
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(144);
        m_led.setLength(m_ledBuffer.getLength());
        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

    }

    private void setColors(Leds led, int r, int g, int b) {
        for (int i = led.val; i < led.val + led.number; i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        changed = true;
    }

    public void setOverCurrent(Leds led, boolean value) {
        logf("**** set over current led %b\n", value);
        if (value) {
            setColors(led, 80, 0, 0);
        } else {
            setColors(led, 0, 80, 0);
        }
    }

    public void setNoteState(Leds led, boolean notePresent) {
        if (notePresent) {
            setColors(led, 0, 80, 0);
        } else {
            setColors(led, 80, 0, 0);
        }
    }
}
