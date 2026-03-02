package org.firstinspires.ftc.teamcode;

/**
 * This is an example odometer for the common three wheel odometer setup.
 */
public interface Odometer {
    double getX();
    double getY();
    double getZ();
    void resetTo(double x, double y, double z);
}
