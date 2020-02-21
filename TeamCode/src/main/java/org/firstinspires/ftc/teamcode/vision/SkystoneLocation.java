package org.firstinspires.ftc.teamcode.vision;

public enum SkystoneLocation {
    LEFT(0), MIDDLE(1), RIGHT(2);

    private int numericalValue;

    SkystoneLocation(int numericalValue) {
        this.numericalValue = numericalValue;
    }

    public int getNumericalValue() {
        return numericalValue;
    }
}
