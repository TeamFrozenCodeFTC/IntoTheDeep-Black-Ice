package org.firstinspires.ftc.teamcode.blackIce.tuning;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class QuadraticRegression {
    public static double[] quadraticFit(List<double[]> points) {
        int n = points.size();
        double sumX = 0, sumX2 = 0, sumX3 = 0, sumX4 = 0;
        double sumY = 0, sumXY = 0, sumX2Y = 0;

        for (double[] point : points) {
            double x = point[0];
            double y = point[1];

            sumX += x;
            sumX2 += x * x;
            sumX3 += x * x * x;
            sumX4 += x * x * x * x;
            sumY += y;
            sumXY += x * y;
            sumX2Y += x * x * y;
        }

        // Solve for a, b, c using Cramer's Rule
        double[][] matrix = {
            {n, sumX, sumX2},
            {sumX, sumX2, sumX3},
            {sumX2, sumX3, sumX4}
        };

        double[] constants = {sumY, sumXY, sumX2Y};

        return solveLinearSystem(matrix, constants);
    }

    private static double[] solveLinearSystem(double[][] A, double[] B) {
        int n = B.length;
        double[] X = new double[n];
        double detA = determinant(A);

        if (Math.abs(detA) < 1e-10) {
            throw new IllegalArgumentException("Matrix is singular or nearly singular");
        }

        for (int i = 0; i < n; i++) {
            double[][] Ai = replaceColumn(A, B, i);
            X[i] = determinant(Ai) / detA;
        }

        return X; // {c, b, a} in y = ax^2 + bx + c
    }

    private static double determinant(double[][] matrix) {
        return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
            - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
            + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
    }

    private static double[][] replaceColumn(double[][] matrix, double[] column, int colIndex) {
        double[][] newMatrix = new double[matrix.length][matrix[0].length];

        for (int i = 0; i < matrix.length; i++) {
            System.arraycopy(matrix[i], 0, newMatrix[i], 0, matrix[i].length);
            newMatrix[i][colIndex] = column[i];
        }

        return newMatrix;
    }

//    public static void main(String[] args) {
//        List<double[]> points = new ArrayList<>(Arrays.asList(
//            new double[]{1, 2},
//            new double[]{2, 5},
//            new double[]{3, 10},
//            new double[]{4, 17},
//            new double[]{5, 26}
//        ));
//
//        double[] coefficients = quadraticFit(points);
//        System.out.printf("y = %.5fx^2 + %.5fx + %.5f%n", coefficients[2], coefficients[1], coefficients[0]);
//    }
}
