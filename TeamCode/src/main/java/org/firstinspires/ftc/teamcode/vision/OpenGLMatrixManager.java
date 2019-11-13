package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class OpenGLMatrixManager {
    /* Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    // u is rotation on the x axis, v on the y axis, and w on the z axis
    // The right hand rule should be utilized in keeping dimensions in check
    // The rotations are applied in the following order: x, y, z axes
    // This is crucial to know because the order can greatly alter your final result
    // Once again, use the right hand rule and visualize the rotation of the object
    */
    public static OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    public static OpenGLMatrix createMatrix(double x, double y, double z, double u, double v, double w) {
        return OpenGLMatrix.translation((float) x, (float) y, (float) z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, (float) u, (float) v, (float) w));
    }


    // Creates a matrix with 0 for all values
    // On the field, this represents the bottom left corner of the field from the audience's view
    // This is the corner with the red depot (the red square) by the human player station
    public static  OpenGLMatrix originMatrix() {
        return createMatrix(0, 0, 0, 0, 0, 0);
    }

    // Formats a matrix into a readable string
    public static  String formatMatrix(OpenGLMatrix matrix) {
        return matrix.formatAsTransform();
    }
}
