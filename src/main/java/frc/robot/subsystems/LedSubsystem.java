package frc.robot.subsystems;

import static frc.robot.Util.logf;

import org.opencv.features2d.FlannBasedMatcher;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

//import static frc.robot.utilities.Util.logf;

public class LedSubsystem extends SubsystemBase {
    boolean changed = false;
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;    
    private GrabberSubsystem grabberSubsystem;
    private ShooterSubsystem shooterSubsystem;

    public LedSubsystem(GrabberSubsystem grabberSubsystem, ShooterSubsystem shooterSubsystem) {
        this.grabberSubsystem = grabberSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        initNeoPixel();
        setColors(Leds.GRABBER, 0,0,0);
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
        // logf("Current status = %s\n", state.toString());
        switch (state) {
            case SET_IDLE_COLOR:
                setColors(Leds.GRABBER, 0, 0, 200);
                state = State.IDLE;
                break;
            case IDLE:                
                if (grabberSubsystem.noteFullyIn()) { 
                    state = State.FLASH;
                }
                break;
            case FLASH:
                if (counter % 10 == 0) {
                    if (on) {
                        setColors(Leds.GRABBER, 255, 255, 255);    
                    } else {
                        setColors(Leds.GRABBER, 0, 0, 0);    
                    }                    
                    on = !on;    
                }
                counter++;
                if (counter > 60) {
                    counter = 0;
                    state = State.SET_HOLDING_COLOR;
                }
                break;
            case SET_HOLDING_COLOR:
                // the note is fully in, but not indexed. We set magenta.
                setColors(Leds.GRABBER, 255, 0, 255);    
                state = State.HOLDING;
                break;             
            case HOLDING:
                if (shooterSubsystem.noteIsIndexedSensor.getVoltage() < 1) {
                    state = State.SET_ERROR_COLOR;
                } else if (shooterSubsystem.noteIsIndexedSensor.getVoltage() < 2.9) {                    
                    state = State.SET_INDEXED_COLOR;    
                } else if (!grabberSubsystem.noteFullyIn() && !grabberSubsystem.seeNote()) {
                    state = State.SET_IDLE_COLOR;
                }
                break;
            case SET_ERROR_COLOR:
                setColors(Leds.GRABBER, 255, 0, 0); 
                state = State.ERROR;
                break;
            case ERROR:
                if (!grabberSubsystem.noteFullyIn() && !grabberSubsystem.seeNote()) {
                    state = State.SET_IDLE_COLOR;
                }            
                break;
            case SET_INDEXED_COLOR:
                setColors(Leds.GRABBER, 0, 255, 0); 
                state = State.INDEXED;
                break;
            case INDEXED:
                if (shooterSubsystem.noteIsIndexedSensor.getVoltage() < 1) {
                    state = State.SET_ERROR_COLOR;
                } else if (!grabberSubsystem.noteFullyIn() && !grabberSubsystem.seeNote()) {
                    state = State.SET_IDLE_COLOR;
                }    
                if (!grabberSubsystem.noteFullyIn() && !grabberSubsystem.seeNote()) {
                    state = State.SET_IDLE_COLOR;
                }            
                break;
        }        
        if (changed) {
                m_led.setData(m_ledBuffer);
        }
        changed = false;
    }


    // @Override
    public void periodic1() {    
        if (grabberSubsystem.noteFullyIn()) {        
            if (counter % 10 == 0) {
                on = !on;    
            }
            if (on && counter < 60) {
                setColors(Leds.GRABBER, 255, 255, 255);    
            } else {
                setColors(Leds.GRABBER, 0, 0, 0);    
            }
            if (counter > 60) {
                if (shooterSubsystem.noteIsIndexedSensor.getVoltage() > 2.9) {
                    // the note is fully in, but not indexed. We set magenta.
                    setColors(Leds.GRABBER, 255, 0, 255); 
                }
            }
        } else {
            counter = 0;
            setColors(Leds.GRABBER, 0, 0, 200);
            if ((shooterSubsystem.noteIsIndexedSensor.getVoltage() > 1) &&
                           (shooterSubsystem.noteIsIndexedSensor.getVoltage() < 2.9)) {
                    // the note is fully in and indexed. We set green.
                    setColors(Leds.GRABBER, 0, 255, 0); 
            } else if (shooterSubsystem.noteIsIndexedSensor.getVoltage() < 1) {
                // there is a problem with the sensor, we set color to red.
                setColors(Leds.GRABBER, 255, 0, 0); 
            }
        }
        // setColors(Leds.GRABBER,100,100,0);
       counter++;
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
}
