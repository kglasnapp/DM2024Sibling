package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.utilities.Util.logf;

public class PDHData extends SubsystemBase {
    int PDH_CAN_ID = 1;
    int NUM_PDH_CHANNELS = 24;
    PowerDistribution m_pdh = new PowerDistribution(PDH_CAN_ID, ModuleType.kRev);

    public PDHData() {
    }

    public void clearStickyFaults() {
        m_pdh.clearStickyFaults();
    }

    public void displayPDHData() {
        /**
         * Get the input voltage of the PDH and display it on Shuffleboard.
         */
        SmartDashboard.putNumber("Voltage", m_pdh.getVoltage());

        /**
         * Get the total current of the PDH and display it on Shuffleboard. This will
         * be to the nearest even number.
         *
         * To get a better total current reading, sum the currents of all channels.
         * See below for getting channel current.
         */
        SmartDashboard.putNumber("Total Current", m_pdh.getTotalCurrent());

        /**
         * Get the currents of each channel of the PDH and display them on
         * Shuffleboard.
         */
        for (int channel = 0; channel < NUM_PDH_CHANNELS; channel++) {
            SmartDashboard.putNumber(
                    ("Ch" + String.valueOf(channel) + " Current"),
                    m_pdh.getCurrent(channel));
        }
        m_pdh.close();

    }

    public void logPDHData() {
        String s = "";
        for (int channel = 0; channel < NUM_PDH_CHANNELS; channel++) {
            double current = m_pdh.getCurrent(channel);
            if (current > .01) {
                s += String.format(" %d:%.2f ", channel, current);
            }
        }
        logf("Voltage %.2f Current:%.2f %s\n", m_pdh.getVoltage(), m_pdh.getTotalCurrent(), s);
    }
}
