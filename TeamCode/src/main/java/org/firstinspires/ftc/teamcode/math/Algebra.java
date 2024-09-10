package org.firstinspires.ftc.teamcode.math;

import androidx.annotation.NonNull;

import org.jetbrains.annotations.Contract;

public class Algebra {
    public static class QuadraticFormulaResult {
        public final int answerCount;
        public final double root1;
        public final double root2;

        public QuadraticFormulaResult(double root1, double root2, int answerCount) {
            this.answerCount = answerCount;
            this.root1 = root1;
            this.root2 = root2;
        }
    }

    public static int sign(double num) {
        if (num >= 0) {
            return 1;
        } else {
            return -1;
        }
    }

    /**
     * Solve for all real solutions to a quadratic.
     * @param a The coefficient of the x^2 term.
     * @param b The coefficient of the x term.
     * @param c The final term.
     * @return All real solutions to the quadratic.
     */
    @NonNull
    @Contract(value = "_, _, _ -> new", pure = true)
    public static QuadraticFormulaResult solveQuadraticRoots(double a, double b, double c) {
        if (a == 0) {
            if (b == 0) { // we literally just have a single point
                return new QuadraticFormulaResult(c, 0, 1);
            }
            // this means that we just have a line
            return new QuadraticFormulaResult(-c/b, 0, 1);
        }

        double discriminant = b*b - 4*a*c;
        if (discriminant < 0) { // no real slns
            return new QuadraticFormulaResult(0, 0, 0);
        }

        double ans1 = (-b + Math.sqrt(discriminant)) / (2*a);
        if (discriminant == 0) { // one real sln
            return new QuadraticFormulaResult(ans1, ans1, 1);
        }

        double ans2 = (-b - Math.sqrt(discriminant)) / (2*a);
        return new QuadraticFormulaResult(ans1, ans2, 2);
    }

    /**
     * Round a number.
     * @param number The number to round.
     * @param decimalPlaces The amount of decimal places to round to.
     * @return The rounded number.
     */
    public static double round(double number, double decimalPlaces) {
        double scale = Math.pow(10, decimalPlaces);
        return (double)Math.round(number * scale) / scale;
    }
}
