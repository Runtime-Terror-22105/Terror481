package org.firstinspires.ftc.teamcode.robot.auto.followers;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;

import java.util.function.Predicate;

public class Task {
    public enum TaskType {
        DRIVING,
        ACTION,
        FINISH_ACTIONS
    }

    public static class Context {
        private Drivetrain drivetrain;
        private Pose2d currentPos;
        public PidToPoint p2p;

        public Context(Drivetrain drivetrain) {
            this.drivetrain = drivetrain;
            this.currentPos = new Pose2d();
            this.p2p = new PidToPoint();
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

        public void setGoal(Pose2d goalPoint, Pose2d tolerances) {
            p2p.setGoal(goalPoint, tolerances);
        }
    }

    public final TaskType taskType;
    private final Predicate<Context> task;
    public final Context context;

    public Task(Context context,
                Predicate<Context> task,
                TaskType taskType) {
        this.context = context;
        this.task = task;
        this.taskType = taskType;
    }

    public boolean execute(Context context) {
        return task.test(context);
    }

//    public void updateContext(String key, Object value) {
//        Object oldValue = context.get(key);
//        if (oldValue == null) {
//            context.put(key, value);
//        } else {
//            context.replace(key, oldValue, value);
//        }
//    }

    public TaskType getTaskType() {
        return taskType;
    }
}
