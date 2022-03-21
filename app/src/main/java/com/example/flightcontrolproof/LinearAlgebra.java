package com.example.flightcontrolproof;

import java.util.Arrays;

public class LinearAlgebra {
    double[][] matrixAdd(double[][] J, double[][] Q) {
        int n = J.length;
        double[][] matReturn = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                matReturn[i][j] = J[i][j] + Q[i][j];
            }
        }
        return matReturn;
    }

    double[][] matrixTranspose(double[][] J) {
        int n = J.length;
        double[][] matReturn = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                matReturn[i][j] = J[j][i];
            }
        }
        return matReturn;
    }

    double[][] matrixMultiply(double[][] J, double[][] P) {
        int n = J.length;
        double[][] matReturn = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                double temp = 0.0;
                for (int k = 0; k < n; k++) {
                    temp = J[i][k] * P[k][i];
                }
                matReturn[i][j] = temp;
            }
        }
        return matReturn;
    }

    double[][] matrixSubtract(double[][] mat1, double[][] mat2){
        int n = mat1.length;
        double[][] retMat = new double[n][n];
        for(int i = 0; i<n; i++){
            for(int j = 0; j<n; j++){
                retMat[i][j] = mat1[i][j] - mat2[i][j];
            }
        }
        return retMat;
    }

    double[] vectMatMultiply(double[][] mat, double[] vect){
        int n = mat.length;
        int nn = mat[0].length;
        double[] vectRet = new double[n];
        for(int i=0; i<n; i++){
            for(int j=0; j<nn; j++){
                vectRet[i] += mat[i][j]*vect[j];
            }
        }
        return vectRet;
    }

    double[] vectAdd(double[] V1, double[] V2, int s){
        int n = V1.length;
        double[] vectRet = new double[n];
        for(int i=0; i<n; i++){
            vectRet[i] = V1[i]+(V2[i]*s);
        }
        return vectRet;
    }

    double[] vectConstMultiply(double[] vect, double c){
        int n = vect.length;
        double[] v = new double[n];
        for(int i=0; i<n; i++){
            v[i] = vect[i]*c;
        }
        return v;
    }


    public double[][] matrixInverse(final double[][] squareMatrix) {
        final int size = squareMatrix.length;
        final double[][] inverseMatrix = new double[size][size];
        for (int i = 0; i < size; ++i) {
            Arrays.fill(inverseMatrix[i], 0.0);
            inverseMatrix[i][i] = 1.0;
        }
        for (int i = 0; i < size; ++i) {
            findPivotAndSwapRow(i, squareMatrix, inverseMatrix, size);
            sweep(i, squareMatrix, inverseMatrix, size);
        }
        return inverseMatrix;
    }
    /**
     * A utility function to inverse matrix.
     * Find a pivot and swap the row of squareMatrix0 and squareMatrix1
     */
    private static void findPivotAndSwapRow(final int row, final double[][] squareMatrix0, final double[][] squareMatrix1, final int size) {
        int ip = row;
        double pivot = Math.abs(squareMatrix0[row][row]);
        for (int i = row + 1; i < size; ++i) {
            if (pivot < Math.abs(squareMatrix0[i][row])) {
                ip = i;
                pivot = Math.abs(squareMatrix0[i][row]);
            }
        }
        if (ip != row) {
            for (int j = 0; j < size; ++j) {
                final double temp0 = squareMatrix0[ip][j];
                squareMatrix0[ip][j] = squareMatrix0[row][j];
                squareMatrix0[row][j] = temp0;
                final double temp1 = squareMatrix1[ip][j];
                squareMatrix1[ip][j] = squareMatrix1[row][j];
                squareMatrix1[row][j] = temp1;
            }
        }
    }
    /**
     * A utility function to inverse matrix. This function calculates answer for each row by
     * sweeping method of Gauss Jordan elimination
     */
    private static void sweep(final int row, final double[][] squareMatrix0, final double[][] squareMatrix1, final int size){
        final double pivot = squareMatrix0[row][row];

        for (int j = 0; j < size; ++j) {
            squareMatrix0[row][j] /= pivot;
            squareMatrix1[row][j] /= pivot;
        }
        for (int i = 0; i < size; i++) {
            final double sweepTargetValue = squareMatrix0[i][row];
            if (i != row) {
                for (int j = row; j < size; ++j) {
                    squareMatrix0[i][j] -= sweepTargetValue
                            * squareMatrix0[row][j];
                }
                for (int j = 0; j < size; ++j) {
                    squareMatrix1[i][j] -= sweepTargetValue
                            * squareMatrix1[row][j];
                }
            }
        }
    }
}
