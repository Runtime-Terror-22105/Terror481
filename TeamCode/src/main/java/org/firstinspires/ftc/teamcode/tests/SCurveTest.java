package org.firstinspires.ftc.teamcode.tests;

import org.firstinspires.ftc.teamcode.math.Scurve;

public class SCurveTest {
    public static void main(String[] args) {
        Scurve curve = new Scurve(
                0, 1, 1
        );

        System.out.println(Math.abs(curve.getVelocity(0.73) - 0.26645) < 0.0001); // concave up accel
        System.out.println(Math.abs(curve.getVelocity(1.29) - 0.74795) < 0.0001); // concave down accel
        System.out.println(Math.abs(curve.getVelocity(2.165) - 1) < 0.0001); // linear
        System.out.println(Math.abs(curve.getVelocity(2.917) - 0.86636) < 0.0001); // concave down decel
        System.out.println(Math.abs(curve.getVelocity(3.685) - 0.25561) < 0.0001); // concave up decel
    }
}
