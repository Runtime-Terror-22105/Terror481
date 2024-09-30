package org.firstinspires.ftc.teamcode.tests;

import org.firstinspires.ftc.teamcode.math.Scurve;

public class SCurveTest {
    public static void main(String[] args) {
        Scurve curve = new Scurve(
                0, 1, 1,1
        );

//        System.out.println(Math.abs(curve.getVelocity(0.73) - 0.26645) < 0.0001); // concave up accel

//        System.out.println(Math.abs(curve.getVelocity(0.5))); // concave down accel
//        System.out.println(Math.abs(curve.getVelocity(1.5))); // concave down accel
//        System.out.println(Math.abs(curve.getVelocity(2.5))); // concave down accel
//        System.out.println(Math.abs(curve.getVelocity(3.5))); // concave down accel
//        System.out.println(Math.abs(curve.getVelocity(4.5))); // concave down accel

        System.out.println(curve.getPosition(0.5));
    }
}
