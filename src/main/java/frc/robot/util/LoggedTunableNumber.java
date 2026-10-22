package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LoggedTunableNumber {
  private final String key;
  private final double defaultValue;
  private double lastValue;

  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this.key = dashboardKey;
    this.defaultValue = defaultValue;
    this.lastValue = defaultValue;
    SmartDashboard.putNumber(dashboardKey, defaultValue);
  }

  public double get() {
    return SmartDashboard.getNumber(key, defaultValue);
  }

  public boolean hasChanged(int id) {
    double currentValue = get();
    boolean changed = currentValue != lastValue;
    lastValue = currentValue;
    return changed;
  }
}