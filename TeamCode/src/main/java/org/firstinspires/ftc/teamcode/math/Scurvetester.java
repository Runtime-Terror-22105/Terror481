package org.firstinspires.ftc.teamcode.math;

public class Scurvetester {
    public static void main(String[]args){
        Scurve curves=new Scurve(0,-5);

//        System.out.println(curves.getParameters());
        double finalPos=curves.T*2+curves.line_length;


        System.out.println(curves.getPosition(finalPos)); //final pos represents the final time


        System.out.println(curves.getParameters());



//        System.out.println(curves.getPosition(finalPos/4));
    }
}
