package org.firstinspires.ftc.teamcode.vision;

import android.os.Environment;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class VuforiaManager {

    // Used to convert between inches and millimeters ([value in in.] * mmPerInch = [value in mm.])
    private static final float mmPerInch = 25.4f;

    // Fields for Vuforia
    private VuforiaTrackables visionTargets;
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;

    // Stores all the initialized VuforiaTrackables to check for detected markers
    private ArrayList<VuforiaTrackable> allTrackables;

    // The last known location of the bot, obtained from Vuforia
    // The phone's offset from the center of the bot with the long side along the y axis and screen up
    private OpenGLMatrix lastKnownLocation, phoneLocation;

    private float robotX = 0, robotY = 0, robotAngle = 0;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    private Future<?> scanForTargets = null;
    private ExecutorService asyncExecutor = Executors.newSingleThreadExecutor();

    public VuforiaManager(Telemetry telemetry, HardwareMap hardwareMap) throws InterruptedException {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        setupVuforia();

        // We don't know where the robot is, so set it to the origin
        // If we don't include this, it would be null, which would cause errors later on
        lastKnownLocation = OpenGLMatrixManager.originMatrix();

        telemetry.addLine("Ready!");
        telemetry.update();
    }

    public double getRobotX() {
        return (double) robotX;
    }

    public double getRobotY() {
        return (double) robotY;
    }

    public double getRobotAngle() {
        return (double) robotAngle;
    }

    public OpenGLMatrix getTrackableLocation(int trackableIndex) {
        return allTrackables.get(trackableIndex).getLocation();
    }

    public OpenGLMatrix getTrackableLocation(String trackableName) {
        for (VuforiaTrackable trackable : allTrackables)
            if (trackable.getName().toUpperCase().equals(trackableName.toUpperCase()))
                return trackable.getLocation();

        return null;
    }

    public void activate() {
        visionTargets.activate();
        asyncExecutor.submit(targetChecker);
    }

    public void shutdown() {
        asyncExecutor.shutdownNow();
        visionTargets.deactivate();
    }

    private void setupVuforia() throws InterruptedException {
        // Read VUFORIA_KEY from SD Card file for the security of the new developer key
        File sdcard = Environment.getExternalStorageDirectory();

        //Get the text file
        File file = new File(sdcard, "FIRST/Vuforia_Key_2019_2020.txt");

        //Read text from file
        StringBuilder vuforiaKey = new StringBuilder();

        try {
            BufferedReader br = new BufferedReader(new FileReader(file));
            String line;

            while ((line = br.readLine()) != null) {
                vuforiaKey.append(line);
                vuforiaKey.append('\n');
            }
            br.close();
        } catch (IOException e) {
            throw new InterruptedException("Please store the Vuforia key on the phone's SD card in the FIRST folder and make sure the file name reflects the one used in the code!");
        }

        telemetry.addLine("NOTICE: If no camera view is visible on the robot controller, the Vuforia key is missing from the SD card!");
        telemetry.update();

        // Setup parameters to create localizer
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = vuforiaKey.toString();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters);

        // These are the vision targets that we want to use
        visionTargets = vuforiaLocalizer.loadTrackablesFromFile("/sdcard/FIRST/Skystone");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);


        // Set phone location on robot (from center with screen facing up)
        phoneLocation = OpenGLMatrixManager.createMatrix(0, 0, 0, -90, 0, -90);

        // Initialize the VuforiaTrackable objects and stores them in allTrackables
        allTrackables = new ArrayList<>();

        initTrackable(0, "Stone Target", OpenGLMatrixManager.originMatrix());
        initTrackable(1, "Blue Rear Bridge", OpenGLMatrixManager.originMatrix());
        initTrackable(2, "Red Rear Bridge", OpenGLMatrixManager.originMatrix());
        initTrackable(3, "Red Front Bridge", OpenGLMatrixManager.originMatrix());
        initTrackable(4, "Blue Front Bridge", OpenGLMatrixManager.originMatrix());
        initTrackable(5, "Red Perimeter 1", OpenGLMatrixManager.originMatrix());
        initTrackable(6, "Red Perimeter 2", OpenGLMatrixManager.originMatrix());
        initTrackable(7, "Front Perimeter 1", OpenGLMatrixManager.originMatrix());
        initTrackable(8, "Front Perimeter 2", OpenGLMatrixManager.originMatrix());
        initTrackable(9, "Blue Perimeter 1", OpenGLMatrixManager.createMatrix(0, 36 * mmPerInch, 0, 90, 0, 90));
        initTrackable(10, "Blue Perimeter 2", OpenGLMatrixManager.createMatrix(0, 108 * mmPerInch, 0, 90, 0, 90));
        initTrackable(12, "Rear Perimeter 1", OpenGLMatrixManager.originMatrix());
        initTrackable(11, "Rear Perimeter 2", OpenGLMatrixManager.originMatrix());
    }

    private void initTrackable(int targetIndex, String name, OpenGLMatrix location) {
        VuforiaTrackable trackable = visionTargets.get(targetIndex);
        trackable.setName(name);
        trackable.setLocation(location);

        // Setup listener and inform it of phone information
        ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocation, parameters.cameraDirection);

        allTrackables.add(trackable);
    }

    private void checkForInterrupt() throws InterruptedException {
        if (Thread.interrupted())
            throw new InterruptedException();
    }

    private Runnable targetChecker = new Runnable() {
        @Override
        public void run() {
            try {
                while (true) {
                    for (VuforiaTrackable trackable : allTrackables) {
                        checkForInterrupt();
                        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();

                        if (listener.isVisible()) {
                            // Ask the listener for the latest information on where the robot is
                            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

                            // The listener will sometimes return null, so we check for that to prevent errors
                            if (latestLocation != null)
                                lastKnownLocation = latestLocation;

                            float[] coordinates = lastKnownLocation.getTranslation().getData();

                            // Update robot location and angle
                            robotX = coordinates[0];
                            robotY = coordinates[1];
                            robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                            break;
                        }
                    }
                }
            } catch (InterruptedException e) {
                telemetry.addLine("Shutting down vuforia thread...");
                telemetry.update();
            }
        }

    };
}