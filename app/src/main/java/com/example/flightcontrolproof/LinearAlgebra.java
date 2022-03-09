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
        double[] vectRet = new double[n];
        for(int i=0; i<n; i++){
            for(int j=0; j<n; j++){
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


    //https://www.sanfoundry.com/java-program-find-inverse-matrix/
    double[][] matrixInverse(double[][] mat){
        int n = mat.length;
        double[][] x = new double[n][n];
        double[][] b = new double[n][n];
        int[] index = new int[n];
        gaussianPivot(mat,index);

        for(int i=0; i<n; i++){
            for(int j=i+1; j<n; ++j){
                for(int k=0; k<n; ++k){
                    b[index[j]][k] -= mat[index[j]][i]*b[index[i]][k];
                }
            }
        }
        for(int i=0; i<n; ++i){
            x[n-1][i] = b[index[n-1]][i]/mat[index[n-1]][n-1];
            for(int j=n-2; j>=0; --j){
                x[j][i] = b[index[j]][i];
                for(int k=j+1; k<n; ++k){
                    x[j][i] -= mat[index[j]][k]*x[k][i];
                }
                x[j][i] /= mat[index[j]][j];
            }
        }
        return x;
    }

    void gaussianPivot(double[][] a, int[] index){
        double[] c = new double[14];
        for(int i=0; i<14;i++){
            index[i] = i;
            double c1 = 0;
            for(int j=0; j<14; j++){
                double c0 = Math.abs(a[i][j]);
                if(c0 > c1){
                    c1 = c0;
                }
            }
            c[i] = c1;
        }
        int k = 0;
        for(int j=0;j<14-1;j++){
            double pi1 = 0;
            for(int i=j;i<14; i++){
                double pi0 = Math.abs(a[index[i]][j]);
                pi0 /= c[index[i]];
                if(pi0>pi1){
                    pi1 = pi0;
                    k = i;
                }
            }
            int itmp = index[j];
            index[j] = index[k];
            index[k] = itmp;
            for(int i=j+1; i<14; i++){
                double pj = a[index[i]][j]/a[index[j]][j];
                a[index[i]][j] = pj;
                for(int l=j+1; l<14; ++l){
                    a[index[i]][l] -= pj*a[index[j]][l];
                }
            }
        }
    }
}
