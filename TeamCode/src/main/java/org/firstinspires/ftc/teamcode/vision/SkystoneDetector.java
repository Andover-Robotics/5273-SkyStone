package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

public class SkystoneDetector {
    private OpenCvInternalCamera camera;
    private StoneProcessingPipeline pipeline;
    private int stoneBaseYCoordinate, stoneWidth, stoneHeight;

    public SkystoneDetector(HardwareMap hardwareMap, int stoneBaseYCoordinate, int stoneWidth, int stoneHeight) {
        this.stoneBaseYCoordinate = stoneBaseYCoordinate;
        this.stoneWidth = stoneWidth;
        this.stoneHeight = stoneHeight;

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        pipeline = new StoneProcessingPipeline();
        camera.setPipeline(pipeline);
    }

    public void start() {
        camera.openCameraDevice();
        camera.startStreaming(960, 720, OpenCvCameraRotation.UPSIDE_DOWN);
    }

    public void setFlashLight(boolean value) {
        camera.setFlashlightEnabled(value);
    }

    public void stop() {
        camera.setFlashlightEnabled(false);
        camera.stopStreaming();
    }

    public SkystoneLocation getSkystoneLocation() {
        try {
            return pipeline.getSkystoneLocation();
        } catch (NullPointerException e) {
            return null;
        }
    }

    public int getStoneBaseYCoordinate() {
        return stoneBaseYCoordinate;
    }

    public void setStoneBaseYCoordinate(int stoneBaseYCoordinate) {
        this.stoneBaseYCoordinate = stoneBaseYCoordinate;
    }

    public int getStoneWidth() {
        return stoneWidth;
    }

    public void setStoneWidth(int stoneWidth) {
        this.stoneWidth = stoneWidth;
    }

    public int getStoneHeight() {
        return stoneHeight;
    }

    public void setStoneHeight(int stoneHeight) {
        this.stoneHeight = stoneHeight;
    }

    private class StoneProcessingPipeline extends OpenCvPipeline {
        private Scalar[] averageColors = new Scalar[3];

        private Scalar rectColor = new Scalar(255, 0, 0), largeAreaToScanColor = new Scalar(0, 0, 255), smallAreaToScanColor = new Scalar(0, 255, 0);

        @Override
        public Mat processFrame(Mat input) {

            for (int boxNum = 1; boxNum <= 3; boxNum++) {
                double stoneLeftCoordinate = input.cols() / 2.0 - (2.5 - boxNum) * stoneWidth,
                        stoneRightCoordinate = stoneLeftCoordinate + stoneWidth;

                Imgproc.rectangle(input,
                        new Point(stoneLeftCoordinate, input.rows() - stoneBaseYCoordinate),
                        new Point(stoneRightCoordinate, input.rows() - stoneBaseYCoordinate - stoneHeight),
                        rectColor, 4); // Rectangle around the skystone

                Rect smallAreaToScan = new Rect(new Point(stoneLeftCoordinate + stoneWidth * 0.375, input.rows() - stoneBaseYCoordinate - stoneHeight * 0.375),
                        new Point(stoneRightCoordinate - stoneWidth * 0.375, input.rows() - stoneBaseYCoordinate - stoneHeight * 0.625));

                Imgproc.rectangle(input, smallAreaToScan, smallAreaToScanColor, 4); // Smaller rectangle around the area that will be scanned in the final stages

                averageColors[boxNum - 1] = averageColor(input, smallAreaToScan);
            }

            return input;
        }

        SkystoneLocation getSkystoneLocation() throws NullPointerException {
            Object[][] brightnessAndPosition = {
                    {0, SkystoneLocation.LEFT},
                    {0, SkystoneLocation.MIDDLE},
                    {0, SkystoneLocation.RIGHT}
            };

            for (int i = 0; i < brightnessAndPosition.length; i++)
                brightnessAndPosition[i][0] = colorBrightness(averageColors[i]);

            // Sort brightnessAndPosition to find the darkest area and the corresponding location
            Arrays.sort(brightnessAndPosition, (Object[] current, Object[] other) -> (int) current[0] - (int) other[0]);

            return (SkystoneLocation) brightnessAndPosition[0][1];
        }

        // Returns a numerical representation of the brightness of the given RGB color
        // Formula from http://alienryderflex.com/hsp.html
        private int colorBrightness(Scalar color) {
            double[] rgb = color.val;

            return (int) Math.round(Math.sqrt(.241 * Math.pow(rgb[0], 2) + .691 * Math.pow(rgb[1], 2) + .068 * Math.pow(rgb[2], 2)));
        }

        private Scalar averageColor(Mat input, Rect frame) {
            Mat mat = input.submat(frame);
            Mat hsvMat = mat.clone();

            Scalar color = Core.mean(hsvMat);

            mat.release();
            hsvMat.release();

            return color;
        }

    }
}
