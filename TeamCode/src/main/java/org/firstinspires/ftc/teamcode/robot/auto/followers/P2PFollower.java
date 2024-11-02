package org.firstinspires.ftc.teamcode.robot.auto.followers;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;

import java.util.HashMap;
import java.util.Stack;
import java.util.function.Predicate;

public class P2PFollower {
    private static class Task {
        private final HashMap<String, Object> context;
        private final Predicate<HashMap<String, Object>> task;

        public Task(HashMap<String, Object> context, Predicate<HashMap<String, Object>> task) {
            this.context = context;
            this.task = task;
        }

        public boolean execute() {
            return task.test(context);
        }
    }

    public static class Builder {
        private Drivetrain drivetrain;
        private Stack<Task> tasks = new Stack<>();

        public Builder(Drivetrain drivetrain) {
            this.drivetrain = drivetrain;
        }

        /**
         * Creates the bare minimum context passed to all functions.
         * @return A minimal context
         */
        @NonNull
        private HashMap<String, Object> createBasicContext() {
            HashMap<String, Object> context = new HashMap<>();
            context.put("drivetrain", drivetrain);
            context.put("currentPos", new Pose2d(0, 0, 0));
            return context;
        }

        public Builder addPoint(Pose2d point, Pose2d tolerance) {
            HashMap<String, Object> context = createBasicContext();
            context.put("p2p", new PidToPoint(point, tolerance));

            Task task = new Task(
                    context,
                    (HashMap<String, Object> ctx) -> {
                        PidToPoint p2p = (PidToPoint) ctx.get("p2p");
                        Drivetrain dt = (Drivetrain) ctx.get("drivetrain");
                        Pose2d currentPos = (Pose2d) ctx.get("currentPos");
                        return p2p.driveToDestination(dt, currentPos);
                    }
            );
            tasks.add(task);

            return this;
        }
    }
}
