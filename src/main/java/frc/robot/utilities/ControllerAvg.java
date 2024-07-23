package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerAvg {
    CommandXboxController m_controller;

    RunningAverage runningAverageLeftX = new RunningAverage(10);
    RunningAverage runningAverageLeftY = new RunningAverage(10);
    RunningAverage runningAverageRightX = new RunningAverage(10);

    public ControllerAvg(CommandXboxController m_controller) {
        this.m_controller = m_controller;
    }

    public double getLeftY() {
        double y = m_controller.getLeftY();
        runningAverageLeftY.add(y);
        return runningAverageLeftY.getAverage();
    }
    
    public double getLeftX() {
        double x = m_controller.getLeftX();
        runningAverageLeftX.add(x);
        return runningAverageLeftX.getAverage();
    }

    public double getRightX() {
        double x = m_controller.getRightX();
        runningAverageRightX.add(x);
        return runningAverageRightX.getAverage();
    }
}
