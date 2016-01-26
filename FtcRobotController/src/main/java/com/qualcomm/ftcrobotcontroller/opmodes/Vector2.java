package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Represents a 2D vector in cartesian coordinates.
 */
public class Vector2 {
    public double x;
    public double y;

    /**
     * Constructs a Vector2 instance.
     * @param x     x coordinate
     * @param y     y coordinate
     */
    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Returns the length of this vector.
     * @return the length
     */
    public double getMagnitude() {
        return Math.sqrt( x * x + y* y);
    }

    /**
     * Returns the dot product between vector a and vector b.
     * @param a     vector a
     * @param b     vector b
     * @return  the dot product
     */
    public static double Dot( Vector2 a, Vector2 b) {
        return a.x * b.x + a.y * b.y;
    }

    /**
     * Returns the the result of subtracting vector a - vector b.
     * @param a     vector a
     * @param b     vector b
     * @return  the vector difference
     */
    public static Vector2 subtract(Vector2 a, Vector2 b) {
        return new Vector2(a.x  - b.x, a.y - b.y);
    }
    /**
     * Multiplies a scalar to a vector.
     * @param p     the scalar
     * @param a     the vector
     * @return  the scaled vector
     */
    public static Vector2 product(double p, Vector2 a) {
        return new Vector2(p * a.x, p * a.y);
    }
}
