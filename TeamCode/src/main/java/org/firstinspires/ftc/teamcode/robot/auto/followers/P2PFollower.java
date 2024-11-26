package org.firstinspires.ftc.teamcode.robot.auto.followers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.function.Supplier;

public class P2PFollower {
    public static class Builder {
        private final Drivetrain drivetrain;
        private final Queue<Task> tasks = new LinkedList<>();
        private final RobotHardware hardware;
        private final MultipleTelemetry telemetry;

        public Builder(@NonNull Robot robot) {
            this.drivetrain = robot.drivetrain;
            this.hardware = robot.hardware;
            this.telemetry = robot.telemetry;
        }

        public Builder addPoint(
                Pose2d point,
                Pose2d tolerance,
                double reachedTime,
                double timeLimit
        ) {
            return this.addPoint(
                    "Drive to point (" + point.x + "," + point.y + "," + point.heading + ")",
                    point,
                    tolerance,
                    reachedTime,
                    timeLimit
            );
        }

        /**
         * Adds an instruction to drive to a point.
         * @param pointName The name of the point
         * @param point The point to drive to.
         * @param tolerance The tolerance for how much error there can be on the x,y,h.
         * @param reachedTime How long the robot needs to stay at its destination.
         * @param timeLimit How long the robot can take to drive (ms) before it is considered to have stalled.
         * @return The builder object, to allow for chaining.
         */
        public Builder addPoint(
                String pointName,
                Pose2d point,
                Pose2d tolerance,
                double reachedTime,
                double timeLimit
        ) {
            Task.Context context = new Task.Context(drivetrain);
            context.setGoal(point, tolerance, reachedTime);
            Task task = new Task(
                    pointName,
                    context,
                    (Task.Context ctx) -> {
                        Pose2d powers = ctx.p2p.calculatePower(ctx.getCurrentPos());
                        return ctx.p2p.driveToDestination(ctx.getDrivetrain(), powers, ctx.getCurrentPos());
                    },
                    Task.Type.DRIVING,
                    timeLimit
            );
            tasks.add(task);

            return this;
        }

        /**
         * Execute some action once.
         * @param taskName The name of the task in logs
         * @param action The function to run.
         * @param timeLimit How long the action can take to run (ms), before it is considered to have stalled.
         * @return The builder object, to allow for chaining.
         */
        public Builder executeActionOnce(String taskName, Consumer<Task.Context> action, double timeLimit) {
            Task.Context context = new Task.Context(drivetrain);
            Task task = new Task(
                    taskName,
                    context,
                    (Task.Context ctx) -> {
                        action.accept(ctx);
                        return true;
                    },
                    Task.Type.ACTION,
                    timeLimit
            );
            tasks.add(task);
            return this;
        }

        /**
         *
         * @param milliseconds
         * @return
         */
        public Builder sleep(Consumer<Long> sleep, Long milliseconds) {
            Task.Context context = new Task.Context(drivetrain);
            Task task = new Task(
                    "Sleep for " + milliseconds + " ms",
                    context,
                    (Task.Context ctx) -> {
                        sleep.accept(milliseconds);
                        return true;
                    },
                    Task.Type.ACTION,
                    milliseconds
            );
            tasks.add(task);
            return this;
        }

        /**
         * Execute some action repeatedly (async) until some condition is true.
         * @param taskName The name of the task to run.
         * @param condition The condition.
         * @param action The function to run.
         * @param timeLimit How long the action can take to run (ms), before it is considered to have stalled.
         * @return The builder object, to allow for chaining.
         */
        public Builder executeUntilTrue(
                String taskName,
                Predicate<Task.Context> condition,
                Consumer<Task.Context> action,
                double timeLimit
        ) {
            Task.Context context = new Task.Context(drivetrain);
            Task task = new Task(
                    taskName,
                    context,
                    (Task.Context ctx) -> {
                        action.accept(ctx);
                        return condition.test(ctx);
                    },
                    Task.Type.ACTION,
                    timeLimit
            );
            tasks.add(task);
            return this;
        }

        /**
         * Run some action for some amount of time.
         * @param taskName The name of the task to run.
         * @param action The function to run.
         * @param timeLimit The amount of time, in milliseconds
         * @return The builder object, to allow for chaining.
         */
        public Builder executeForTime(
                String taskName,
                Consumer<Task.Context> action,
                double timeLimit
        ) {
            Task.Context context = new Task.Context(drivetrain);
            Task task = new Task(
                    taskName,
                    context,
                    (Task.Context ctx) -> {
                        action.accept(ctx);
                        return false;
                    },
                    Task.Type.ACTION,
                    timeLimit
            );
            tasks.add(task);
            return this;
        }

        public P2PFollower build() {
            return new P2PFollower(tasks, hardware, telemetry);
        }
    }

    private final Queue<Task> pendingTasks;
    private final ArrayList<Task> runningTasks = new ArrayList<>();
    private final RobotHardware hardware;
    private final MultipleTelemetry telemetry;

    private P2PFollower(Queue<Task> tasks, RobotHardware hardware, MultipleTelemetry telemetry) {
        this.pendingTasks = tasks;
        this.hardware = hardware;
        this.telemetry = telemetry;
    }

    public void printTasks(MultipleTelemetry telemetry) {
        for (Task task : pendingTasks) {
            telemetry.addData("task", task.getName());
        }
        telemetry.update();
    }

    public void follow(BooleanSupplier opModeIsActive, Supplier<Pose2d> currentPos) {
        boolean canContinue = true;

        while (!(pendingTasks.isEmpty() && runningTasks.isEmpty()) && opModeIsActive.getAsBoolean()) {
            // clear bulk cache (assume manual is being used)
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }
            telemetry.clearAll();

//            telemetry.addData("Pending tasks", pendingTasks.size());
//            telemetry.addData("Running tasks", runningTasks.size());
//            for (Task task : runningTasks) {
//                telemetry.addData("Running task", task.getName());
//            }

            for (int i = runningTasks.size()-1; i >= 0; i--) {
                Task task = runningTasks.get(i);
                Task.Context ctx = task.getContext();
                ctx.setCurrentPos(currentPos.get());

                if (task.execute(ctx) || task.taskHasStalled()) { // run the task
                    // if the task finished, remove it from the list
                    runningTasks.remove(i);
                }

                telemetry.addData("Task "+i, task.getName());
            }

            if (runningTasks.isEmpty()) {
                canContinue = true;
            }

            Task newTask;
            while (canContinue && !pendingTasks.isEmpty()) {
                newTask = pendingTasks.remove();
                newTask.getContext().startTimer();
                runningTasks.add(newTask);

                if (newTask.getTaskType().equals(Task.Type.DRIVING)) {
                    canContinue = false;
                }
            }

            telemetry.update();
            hardware.write();
        }
    }
}

