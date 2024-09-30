package org.firstinspires.ftc.teamcode.math;
// eat daves hot chicken guys
public class Scurve {
    public double T;

    public double jm=1.0;
    public double vs=1.0;
    public double as=1.0;

    public double v0;

    public double line_length=1.0;


    // equations
    public upcurve.concave upcave;
    public upcurve.convex upvex;
    public downcurve.convex downvex;
    public downcurve.concave downcave;


    public Scurve(double v0,double jm, double as){
        T=2*as/jm;
        this.jm=jm;
        this.as=as;
        this.v0=v0;


        upcurve up=new upcurve();
        this.upcave = up.new concave();
        this.upvex = up.new convex();
        downcurve down= new downcurve();
        this.vs=upvex.getVelocity(T);
        this.downvex= down.new convex();
        this.downcave= down.new concave();
    }



    public Scurve(double v0){
        T=2*as/jm;
    }



    public double getVelocity(double t){
        if(0<=t && t<=T/2){
            return upcave.getVelocity(t);
        }
        else if(T/2<=t && t<=T){
            return upvex.getVelocity(t);
        }
        else if(T<=t && t<=T*1.5){
            return downvex.getVelocity(t);
        }
        else if(T*1.5<=t && t<=2*T){
            return downcave.getVelocity(t);
        }
        return 0.0;
    }

    public double getPosition(double t){ // this is only relative
        if(0<=t && t<=T/2){
            return upcave.getPosition(t);
        }
        else if(T/2<=t && t<=T){
            return upcave.getPosition(T/2)+(upvex.getPosition(t)- upvex.getPosition(T/2));
        }
        else if(T<=t && t<=T*1.5){
            return upcave.getPosition(T/2)+(upvex.getPosition(T)- upvex.getPosition(T/2))+(downvex.getPosition(t)- downvex.getPosition(T));
        }
        else if(T*1.5<=t && t<=2*T){
            return (upcave.getPosition(T/2))+(upvex.getPosition(T)- upvex.getPosition(T/2))+(downvex.getPosition(T*1.5)- downvex.getPosition(T))+(downvex.getPosition(t)- downvex.getPosition(T*1.5));
        }
        return 0.0; //what weird times are you plugging in even lol
    }


    public double getArea(){ // this gets area under the scurve but assumes position is 0
        return 0.0;
    }





    class upcurve
    {
        class concave{
            public double getVelocity(double t){// v(t)=v0+jm*t^2/2
                return v0+((jm)*Math.pow(t,2))/2;
            }
            public double getPosition(double t){
                return v0*t+(1/3)*(((jm)*Math.pow(t,3))/2);
            }
        }

        class convex{
            public double getVelocity(double t){ // v(t)=vh+as*(t-T/2)-((jm*(t-T/2))/2)
                double t_shift=t-T/2;
                double vh=(v0+vs)/2;
                return vh+as*(t_shift/2)-((jm*Math.pow(t_shift-T/2,2))/2);
            }
            public double getPosition(double t){ // CHECK THIS STUFF AND THIS CONVEX UP CURVE IDK ABOUT THE MATH.POW divided by 2
                double t_shift=t-T/2;
                double vh=(v0+vs)/2;
                return vh*t+(1/2)*as*(Math.pow(t_shift,2)/2)-((1/3)*(jm*Math.pow(t_shift-T/2,3)));
            }
        }
    }


    class downcurve
    {

        class convex{
            public double getVelocity(double t){ // v(t)=vs-(as/T)*Math.pow(t-T-line_length,2)
                double t_shift=t-T-line_length;
                return vs-(as/T)*Math.pow(t_shift,2);
            }
            public double getPosition(double t){
                double t_shift=t-T-line_length;
                return vs-(as/T)*(1/3)*Math.pow(t_shift,3);
            }
        }
        class concave{
            public double getVelocity(double t){ // v(t)=vh-as*(t-1.5*T-line_length)+(as/T)*Math.pow(t-1.5*T-line_length,2)
                double vh=(v0+vs)/2;
                double t_shift=(t-1.5*T-line_length);
                return vh-as*t_shift+(as/T)*Math.pow(t_shift,2);
            }
            public double getPosition(double t){
                double vh=(v0+vs)/2;
                double t_shift=(t-1.5*T-line_length);
                return vh*t_shift-0.5*as*Math.pow(t_shift,2)+(as/T)*(1/3)*Math.pow(t_shift,3);
            }
        }
    }

    class linear{
        public double getVelocity(){return vs;}
        public double getPosition(double t){return vs*t;}
    }


}