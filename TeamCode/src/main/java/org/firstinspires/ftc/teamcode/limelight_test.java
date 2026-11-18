package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "limelight test", group = "testing")
public class limelight_test extends LinearOpMode {

        @Override
        public void runOpMode() {
                // Only hardware device required: "limelight" in Robot Configuration
                Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

                // Faster Driver Station telemetry refresh (~90 Hz) and 100 Hz Limelight polling
                telemetry.setMsTransmissionInterval(11);
                limelight.setPollRateHz(100);
                limelight.pipelineSwitch(0);
                limelight.start();

                // Edge-detection flags for optional Gamepad1 button presses
                boolean prevDpadUp = false;
                boolean prevDpadRight = false;
                boolean prevDpadDown = false;
                boolean prevDpadLeft = false;
                boolean prevButtonA = false;
                int snapshotCount = 0;

                // Show USB connection status immediately during INIT before pressing Play
                while (!isStarted() && !isStopRequested()) {
                        LLStatus status = limelight.getStatus();
                        telemetry.addLine("=== LIMELIGHT 3A (INIT CHECK) ===");
                        telemetry.addData("USB Connected", limelight.isConnected());
                        telemetry.addData("Polling Running", limelight.isRunning());
                        telemetry.addData("Camera Name", "%s", status.getName());
                        telemetry.addData("Pipeline", "%d (%s)", status.getPipelineIndex(), status.getPipelineType());
                        telemetry.addData("System", "Temp: %.1f C | CPU: %.1f%% | FPS: %d",
                                status.getTemp(), status.getCpu(), (int) status.getFps());
                        telemetry.addLine("\nPress PLAY to stream live target telemetry.");
                        telemetry.update();
                }

                waitForStart();

                while (opModeIsActive()) {
                        // Optional: Switch pipelines 0-3 using Gamepad 1 D-Pad
                        if (gamepad1.dpad_up && !prevDpadUp) {
                                limelight.pipelineSwitch(0);
                        } else if (gamepad1.dpad_right && !prevDpadRight) {
                                limelight.pipelineSwitch(1);
                        } else if (gamepad1.dpad_down && !prevDpadDown) {
                                limelight.pipelineSwitch(2);
                        } else if (gamepad1.dpad_left && !prevDpadLeft) {
                                limelight.pipelineSwitch(3);
                        }
                        prevDpadUp = gamepad1.dpad_up;
                        prevDpadRight = gamepad1.dpad_right;
                        prevDpadDown = gamepad1.dpad_down;
                        prevDpadLeft = gamepad1.dpad_left;

                        // Optional: Press Gamepad 1 'A' (Cross) to capture a snapshot for Web UI tuning
                        if (gamepad1.a && !prevButtonA) {
                                snapshotCount++;
                                limelight.captureSnapshot("bench_test_" + snapshotCount);
                        }
                        prevButtonA = gamepad1.a;

                        // 1. Hardware & Pipeline Status
                        LLStatus status = limelight.getStatus();
                        telemetry.addLine("=== LIMELIGHT 3A STATUS ===");
                        telemetry.addData("Connected", "%b (Last update: %d ms ago)",
                                limelight.isConnected(), limelight.getTimeSinceLastUpdate());
                        telemetry.addData("Hardware", "Temp: %.1f C | CPU: %.1f%% | FPS: %d",
                                status.getTemp(), status.getCpu(), (int) status.getFps());
                        telemetry.addData("Pipeline", "Slot %d (%s)",
                                status.getPipelineIndex(), status.getPipelineType());
                        telemetry.addData("Snapshots Taken", snapshotCount);

                        // 2. Latest Vision Result
                        telemetry.addLine("\n=== TARGET TELEMETRY ===");
                        LLResult result = limelight.getLatestResult();

                        if (result != null && result.isValid()) {
                                telemetry.addData("Target Status", "VALID TARGET FOUND");
                                telemetry.addData("tx (Horizontal deg)", "%.2f°", result.getTx());
                                telemetry.addData("ty (Vertical deg)", "%.2f°", result.getTy());
                                telemetry.addData("ta (Target Area %)", "%.2f%%", result.getTa());

                                // 3. AprilTag / Fiducial Details (if running an AprilTag pipeline)
                                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                                if (fiducials != null && !fiducials.isEmpty()) {
                                        telemetry.addData("AprilTags Seen", fiducials.size());
                                        for (LLResultTypes.FiducialResult tag : fiducials) {
                                                telemetry.addData("  -> Tag ID " + tag.getFiducialId(),
                                                        "tx: %.1f° | ty: %.1f°",
                                                        tag.getTargetXDegrees(), tag.getTargetYDegrees());
                                        }
                                }

                                // 4. 3D Botpose (if "Full 3D" is enabled in the AprilTag pipeline)
                                Pose3D botpose = result.getBotpose();
                                if (botpose != null) {
                                        telemetry.addData("3D BotPose", botpose.toString());
                                }

                                // 5. Color / Neural Detector Counts (for BIOBUZZ Pollen / Nectar pipelines)
                                List<LLResultTypes.ColorResult> colors = result.getColorResults();
                                if (colors != null && !colors.isEmpty()) {
                                        telemetry.addData("Color Blobs", colors.size());
                                }

                                List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
                                if (detections != null && !detections.isEmpty()) {
                                        telemetry.addData("Detector Objects", detections.size());
                                }
                        } else {
                                telemetry.addData("Target Status", "NO VALID TARGET");
                        }

                        telemetry.addLine("\nControls: D-Pad [Up=0, Right=1, Down=2, Left=3] | A = Snapshot");
                        telemetry.update();
                }

                // Cleanly stop polling when the OpMode ends
                limelight.stop();
        }
}
