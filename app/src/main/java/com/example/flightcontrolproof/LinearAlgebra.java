package com.example.flightcontrolproof;

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
        double[][] matReturn = new double[12][12];
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 12; j++) {
                matReturn[i][j] = J[j][i];
            }
        }
        return matReturn;
    }

    double[][] matrixMultiply(double[][] J, double[][] P) {
        double[][] matReturn = new double[12][12];
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 12; j++) {
                double temp = 0.0;
                for (int k = 0; k < 12; k++) {
                    temp = J[i][k] * P[k][i];
                }
                matReturn[i][j] = temp;
            }
        }
        return matReturn;
    }

    double[][] matrixSub(double[][] mat1, double[][] mat2){
        double[][] retMat = new double[12][12];
        for(int i = 0; i<12; i++){
            for(int j = 0; j<12; j++){
                retMat[i][j] = mat1[i][j] - mat2[i][j];
            }
        }
        return retMat;
    }

    double[] vectMatMul(double[][] mat, double[] vect){
        double[] vectRet = new double[12];
        for(int i=0; i<12; i++){
            for(int j=0; j<12; j++){
                vectRet[i] += mat[i][j]*vect[j];
            }
        }
        return vectRet;
    }

    double[] vectAdd(double[] V1, double[] V2, int s){
        double[] vectRet = new double[12];
        for(int i=0; i<12; i++){
            vectRet[i] = V1[i]+(V2[i]*s);
        }
        return vectRet;
    }
}
