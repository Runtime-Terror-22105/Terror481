package org.firstinspires.ftc.teamcode.math;

import java.lang.Math;

public class cubicformula {

    public static double solveCubic(double a, double b, double c, double d) {
        // Constants and intermediate calculations
        double delta1 = (-Math.pow(b, 3) / (27 * Math.pow(a, 3))) + (b * c) / (6 * Math.pow(a, 2)) - (d / (2 * a));
        double delta2 = (-Math.pow(b, 3) / (27 * Math.pow(a, 3))) + (b * c) / (6 * Math.pow(a, 2)) - (d / (2 * a));

        double discriminant = Math.pow(delta1, 2) + Math.pow((c / (3 * a)) - (Math.pow(b, 2) / (9 * Math.pow(a, 2))), 3);
        // First cube root part
        double part1 = Math.cbrt(delta1 + Math.sqrt(discriminant));
        // Second cube root part
        double part2 = Math.cbrt(delta1 - Math.sqrt(discriminant));
        // Sum up the two parts (cube roots)
        double x = part1 + part2 - (b / (3 * a));
        return x;
    }


    public static void main(String[] args) {
        double a = 1;  // Coefficients for the equation x^3 + bx^2 + cx + d = 0
        double b = 3;
        double c = 2;
        double d = 1;

        double root = solveCubic(a, b, c, d);
        System.out.println("Root: " + root);
    }
}
