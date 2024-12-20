package org.firstinspires.ftc.teamcode.util;


public class ThreePointQuadratic {

    // Solves a 3x3 linear system using Gaussian elimination
    private static double[] solveSystem(double[][] matrix, double[] rhs) {
        int n = matrix.length;

        // Forward elimination
        for (int i = 0; i < n; i++) {
            // Find the pivot row
            int max = i;
            for (int j = i + 1; j < n; j++) {
                if (Math.abs(matrix[j][i]) > Math.abs(matrix[max][i])) {
                    max = j;
                }
            }

            // Swap rows
            double[] temp = matrix[i];
            matrix[i] = matrix[max];
            matrix[max] = temp;

            double tmp = rhs[i];
            rhs[i] = rhs[max];
            rhs[max] = tmp;

            // Normalize pivot row
            for (int j = i + 1; j < n; j++) {
                double factor = matrix[j][i] / matrix[i][i];
                rhs[j] -= factor * rhs[i];
                for (int k = i; k < n; k++) {
                    matrix[j][k] -= factor * matrix[i][k];
                }
            }
        }

        // Back substitution
        double[] solution = new double[n];
        for (int i = n - 1; i >= 0; i--) {
            double sum = 0.0;
            for (int j = i + 1; j < n; j++) {
                sum += matrix[i][j] * solution[j];
            }
            solution[i] = (rhs[i] - sum) / matrix[i][i];
        }
        return solution;
    }

    public static double[] findQuadratic(double[][] points) {
        if (points.length != 3 || points[0].length != 2) {
            throw new IllegalArgumentException("Must provide exactly three points with x and y coordinates.");
        }

        // Extract x and y values from the points array
        double[] x = new double[3];
        double[] y = new double[3];
        for (int i = 0; i < 3; i++) {
            x[i] = points[i][0];
            y[i] = points[i][1];
        }

        // Set up the matrix and right-hand side
        double[][] matrix = {
                {x[0] * x[0], x[0], 1},
                {x[1] * x[1], x[1], 1},
                {x[2] * x[2], x[2], 1}
        };
        double[] rhs = {y[0], y[1], y[2]};

        // Solve for coefficients a, b, and c
        return solveSystem(matrix, rhs);
    }
}
