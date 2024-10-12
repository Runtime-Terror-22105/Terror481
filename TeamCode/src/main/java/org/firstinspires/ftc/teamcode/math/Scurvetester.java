package org.firstinspires.ftc.teamcode.math;

public class Scurvetester {
    public static void main(String[]args){
        Scurve curves=new Scurve(0,1);
//        System.out.println(curves.getParameters());
        double finalPos=curves.T*2+curves.line_length;
        System.out.println(curves.getPosition(finalPos/2));
//        System.out.println(curves.getPosition(finalPos/4));
    }
}
