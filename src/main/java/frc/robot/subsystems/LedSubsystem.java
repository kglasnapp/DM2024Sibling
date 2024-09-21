package frc.robot.subsystems;

import static frc.robot.Util.logf;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import static frc.robot.utilities.Util.logf;

public class LedSubsystem extends SubsystemBase {
    boolean changed = false;
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private boolean whiteActive = false;

    public LedSubsystem() {
        initNeoPixel();
        setColors(Leds.GRABBER, 0, 0, 0);
    }

    public enum Leds {
        GRABBER(0, 144),
        ALLEDS(0, 144);

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
    Alliance lastAlliance = Alliance.Blue;
    boolean lastNoteState = false;

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

    public void setNoteState(boolean notePresent) {
        if (whiteActive && !notePresent) {
            return;
        }
        Leds leds = Leds.ALLEDS;
        if (notePresent != lastNoteState) {
            lastNoteState = notePresent;
            if (notePresent) {
                setColors(leds, 0, 80, 0);
            } else {
                setAllianceState(lastAlliance);
            }
        }
    }

    public void setAllianceState(Alliance alliance) {
        Leds leds = Leds.ALLEDS;
        lastAlliance = alliance;
        if (alliance == Alliance.Blue) {
            setColors(leds, 0, 0, 80);
        } else if (alliance == Alliance.Red) {
            setColors(leds, 80, 0, 0);
        } else {
            setColors(leds, 0, 0, 0);
        }
    }

    public void setLedsToWhite(boolean active) {
        if (lastNoteState) {
            whiteActive = false;
            return;
        }
        whiteActive = active;
        if (active) {
            Leds leds = Leds.ALLEDS;
            if (counter % 10 == 0) {
                setAllianceState(lastAlliance);
            } else {
                setColors(leds, 128, 128, 128);
            }
        } else {
            setAllianceState(lastAlliance);
        }
    }
}
