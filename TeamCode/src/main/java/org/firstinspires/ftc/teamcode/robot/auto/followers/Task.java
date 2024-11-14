package org.firstinspires.ftc.teamcode.robot.auto.followers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;

import java.util.function.Predicate;

public class Task {
    public enum Type {
        DRIVING,
        ACTION,
    }

    public static class Context {
        private final Drivetrain drivetrain;
        public final PidToPoint p2p;
        private final ElapsedTime taskTimer;
        private Pose2d currentPos; // NOT FINAL

        public Context(Drivetrain drivetrain) {
            this.drivetrain = drivetrain;
            this.currentPos = new Pose2d();
            this.p2p = new PidToPoint();
            this.taskTimer = new ElapsedTime();
        }

        /**
         * Start the timer for the task
         */
        public void startTimer() {
            this.taskTimer.reset();
        }

        /**
         * Get the amount of time since the task started (ms).
         * @return The amount of time since the task started, in milliseconds.
         */
        public double getTime() {
            return this.taskTimer.milliseconds();
        }

        public Drivetrain getDrivetrain() {
            return drivetrain;
        }

        public Pose2d getCurrentPos() {
            return currentPos;
        }

        public void setCurrentPos(Pose2d currentPos) {
            this.currentPos = currentPos;
        }

        public void setGoal(Pose2d goalPoint, Pose2d tolerances, double reachedTime) {
            p2p.setGoal(goalPoint, tolerances, reachedTime);
        }
    }

    private final Type taskType;
    private final Predicate<Context> task;
    private final Context context;
    private final double timeLimit;
    private final String name;

    /**
     *
     * @param context The task's context
     * @param task The function for the task to run
     * @param taskType The type of task
     * @param timeLimit The time limit, in milliseconds
     */
    public Task(String name,
                Context context,
                Predicate<Context> task,
                Type taskType,
                double timeLimit) {
        this.name = name;
        this.context = context;
        this.task = task;
        this.taskType = taskType;
        this.timeLimit = timeLimit;
    }

    public boolean execute(Context context) {
        return task.test(context);
    }

    /**
     * Whether or not the task has passed its maximum time limit and has stalled.
     * @return A boolean, if true, the task should be killed, if not it should keep going
     */
    public boolean taskHasStalled() {
        return this.context.getTime() > this.timeLimit;
    }

    public String getName() {
        return this.name;
    }

//    public void updateContext(String key, Object value) {
//        Object oldValue = context.get(key);
//        if (oldValue == null) {
//            context.put(key, value);
//        } else {
//            context.replace(key, oldValue, value);
//        }
//    }

    public Type getTaskType() {
        return taskType;
    }

    public Context getContext() {
        return context;
    }
}
