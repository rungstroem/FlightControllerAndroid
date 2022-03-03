package com.example.flightcontrolproof;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;

import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.IBinder;
import android.os.Looper;
import android.util.Log;

import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import java.util.List;

public class KalmanEstimatorService extends Service {
    SensorManager mSensorManager;
    HandlerThread sensorHandlerThread;
    Handler sensorThreadHandler;
    Looper sensorThreadLooper;

    Sensor accelerometer;
    Sensor gyroscope;
    Sensor magnetometer;
    Sensor geoSensor;

    Thread filterThread;

    LinearAlgebra LA;

    double rad2deg = 180.0/PI;
    double u;
    double v;
    double w;
    double p;
    double q;
    double r;
    double accR;
    double accP;
    double accY;
    double[] x;

    Boolean stopFilterThread = false;
    public KalmanEstimatorService() {
    }

    @Override
    public IBinder onBind(Intent intent) {
        // TODO: Return the communication channel to the service.
        throw new UnsupportedOperationException("Not yet implemented");
    }
    @Override
    public void onCreate(){
        super.onCreate();
        //Sensor thread create
        sensorHandlerThread = new HandlerThread("Sensor handler thread");

        //Access to linear algebra class
        LA = new LinearAlgebra();

        //Sensor create
        mSensorManager = (SensorManager) this.getSystemService(Context.SENSOR_SERVICE);
        accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        gyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        geoSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GEOMAGNETIC_ROTATION_VECTOR);
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startID){
        sensorHandlerThread.start();
        sensorThreadLooper = sensorHandlerThread.getLooper();
        sensorThreadHandler = new Handler(sensorThreadLooper);

        mSensorManager.registerListener(accListener, accelerometer, 1000, sensorThreadHandler);
        mSensorManager.registerListener(gyroListener, gyroscope, 1000, sensorThreadHandler);
        mSensorManager.registerListener(magListener, magnetometer,1000, sensorThreadHandler);
        //mSensorManager.registerListener(geoListener, geoSensor, 1000, sensorThreadHandler);

        return START_STICKY;
    }
    public void onDestroy(){
        mSensorManager.unregisterListener(accListener);
        mSensorManager.unregisterListener(gyroListener);
        mSensorManager.unregisterListener(magListener);
        //mSensorManager.unregisterListener(geoListener);

        sensorThreadLooper.quit();
        sensorHandlerThread.quit();
        stopFilterThread = true;

        Log.i("Kalman","Kalman Service stopped");
    }

    public void sendDataToActivity(double[] stateVector){
        Intent intent = new Intent("KalmanUpdate");
        intent.putExtra("stateVector", stateVector);
        LocalBroadcastManager.getInstance(this).sendBroadcast(intent);
    }

    double[][] R_BN(double R, double P, double Y){
        double[][] RMat = new double[3][3];
        RMat[0][0] = cos(Y) * cos(P);   RMat[0][1] = (-sin(Y))*cos(R) + cos(Y)*sin(P)*sin(R);   RMat[0][2] = sin(Y)*sin(R) + cos(Y)*cos(R)*sin(P);
        RMat[1][0] = sin(Y) * cos(P);   RMat[1][1] = cos(Y)*cos(R) + sin(R)*sin(P)*sin(Y);      RMat[1][2] = (-cos(Y))*sin(R) + sin(P)*sin(Y)*cos(R);
        RMat[2][0] = (-sin(P));         RMat[2][1] = cos(P)*sin(R);                             RMat[2][2] = cos(P)*cos(R);
        return RMat;
    }

    double[][] T_BN(double R, double P){
        double[][] TMat = new double[3][3];
        TMat[0][0] = 1;     TMat[0][1] = sin(R)*tan(P);     TMat[0][2] = cos(R)*tan(P);
        TMat[1][0] = 0;     TMat[1][1] = cos(R);            TMat[1][2] = (-sin(R));
        TMat[2][0] = 0;     TMat[2][1] = sin(R)/cos(P);     TMat[2][2] = cos(R)/cos(P);
        return TMat;
    }

    double[] F = new double[14];
    double[] a_B = new double[3];
    double[] V_B = new double[3];
    double[] V_N = new double[3];
    double[] w_B = new double[3];
    double[] E = new double[3];
    double[] E_dot = new double[3];
    double gx = 0;
    double gy = 0;
    double gz = -9.82;
    double[] stateTransistionFunction(double[] x, double[] U, double dt){
        double Px = x[0];   double Py = x[1];   double Pz = x[2];   double Vx = x[3];   double Vy = x[4];    double Vz = x[5];  double u = x[6];    double v = x[7];    double w = x[8];    double phi = x[9];  double theta = x[10];   double psi = x[11];     double Vg = x[12];  double chi = x[13];
        double ax = U[0];   double ay = U[1];   double az = x[2];   double p = U[3];    double q = U[4];     double r = U[5];
        F[0] = Px + dt*(Vx - dt*(gx + ay*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) - az*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - ax*cos(psi)*cos(theta)));
        F[1] = Py + dt*(Vy - dt*(gy - ay*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) + az*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - ax*cos(theta)*sin(psi)));
        F[2] = Pz + dt*(Vz - dt*(gz + ax*sin(theta) - az*cos(phi)*cos(theta) - ay*cos(theta)*sin(phi)));
        F[3] = Vx - dt*(gx + ay*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) - az*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - ax*cos(psi)*cos(theta));
        F[4] = Vy - dt*(gy - ay*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) + az*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - ax*cos(theta)*sin(psi));
        F[5] = Vz - dt*(gz + az*sin(theta) - az*cos(phi)*cos(theta) - ay*cos(theta)*sin(phi));
        F[6] = u + dt*(ax + gz*sin(theta) - gx*cos(psi)*cos(theta) - gy*cos(theta)*sin(psi));
        F[7] = v + dt*(ay + gx*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) - gy*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - gz*cos(theta)*sin(phi));
        F[8] = w + dt*(az - gz*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + gy*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - gz*cos(phi)*cos(theta));
        F[9] = phi + dt*(p + r*cos(phi)*tan(theta) + q*sin(phi)*tan(theta));
        F[10] = theta + dt*(q*cos(phi) - r*sin(phi));
        F[11] = psi + dt*((r*cos(phi))/cos(theta) + (q*sin(phi))/cos(theta));
        F[12] = sqrt(pow(F[6],2)+pow(F[7],2)+pow(F[8],2));
        F[13] = Math.atan2(F[3],F[4]);

        return F;
    }

    double[][] J = new double[14][14];
    double[][] jacobian(double[] x, double[] U, double dt){
        double Px = x[0];   double Py = x[1];   double Pz = x[2];   double Vx = x[3];   double Vy = x[4];    double Vz = x[5];  double u = x[6];    double v = x[7];    double w = x[8];    double phi = x[9];  double theta = x[10];   double psi = x[11];     double Vg = x[12];  double chi = x[13];
        double ax = U[0];   double ay = U[1];   double az = U[2];   double p = U[3];    double q = U[4];     double r = U[5];

        J[0][0] = 1;    J[0][1] = 0;    J[0][2] = 0;    J[0][3] = dt;  J[0][4] = 0;     J[0][5] = 0;     J[0][6] = 0;     J[0][7] = 0;     J[0][8] = 0;
        J[1][0] = 0;    J[1][1] = 1;    J[1][2] = 0;    J[1][3] = 0;   J[1][4] = dt;    J[1][5] = 0;     J[1][6] = 0;     J[1][7] = 0;     J[1][8] = 0;
        J[2][0] = 0;    J[2][1] = 0;    J[2][2] = 1;    J[2][3] = 0;   J[2][4] = 0;     J[2][5] = dt;    J[2][6] = 0;     J[2][7] = 0;     J[2][8] = 0;
        J[3][0] = 0;    J[3][1] = 0;    J[3][2] = 0;    J[3][3] = 1;   J[3][4] = 0;     J[3][5] = 0;     J[3][6] = 0;     J[3][7] = 0;     J[3][8] = 0;
        J[4][0] = 0;    J[4][1] = 0;    J[4][2] = 0;    J[4][3] = 0;   J[4][4] = 1;     J[4][5] = 0;     J[4][6] = 0;     J[4][7] = 0;     J[4][8] = 0;
        J[5][0] = 0;    J[5][1] = 0;    J[5][2] = 0;    J[5][3] = 0;   J[5][4] = 0;     J[5][5] = 1;     J[5][6] = 0;     J[5][7] = 0;     J[5][8] = 0;
        J[6][0] = 0;    J[6][1] = 0;    J[6][2] = 0;    J[6][3] = 0;   J[6][4] = 0;     J[6][5] = 0;     J[6][6] = 1;     J[6][7] = 0;     J[6][8] = 0;
        J[7][0] = 0;    J[7][1] = 0;    J[7][2] = 0;    J[7][3] = 0;   J[7][4] = 0;     J[7][5] = 0;     J[7][6] = 0;     J[7][7] = 1;     J[7][8] = 0;
        J[8][0] = 0;    J[8][1] = 0;    J[8][2] = 0;    J[8][3] = 0;   J[8][4] = 0;     J[8][5] = 0;     J[8][6] = 0;     J[8][7] = 0;     J[8][8] = 1;
        J[9][0] = 0;    J[9][1] = 0;    J[9][2] = 0;    J[9][3] = 0;   J[9][4] = 0;     J[9][5] = 0;     J[9][6] = 0;     J[9][7] = 0;     J[9][8] = 0;
        J[10][0] = 0;   J[10][1] = 0;   J[10][2] = 0;   J[10][3] = 0;  J[10][4] = 0;    J[10][5] = 0;    J[10][6] = 0;    J[10][7] = 0;    J[10][8] = 0;
        J[11][0] = 0;   J[11][1] = 0;   J[11][2] = 0;   J[11][3] = 0;  J[11][4] = 0;    J[11][5] = 0;    J[11][6] = 0;    J[11][7] = 0;    J[11][8] = 0;
        J[12][0] = 0;   J[12][1] = 0;   J[12][2] = 0;   J[12][3] = 0;  J[12][4] = 0;    J[12][5] = 0;    J[12][6] = 0;    J[12][7] = 0;    J[12][8] = 0;
        J[13][0] = 0;   J[13][1] = 0;   J[13][2] = 0;   J[13][3] = 0;  J[13][4] = 0;    J[13][5] = 0;    J[13][6] = 0;    J[13][7] = 0;    J[13][8] = 0;

        J[0][9] = dt*dt*(ay*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + az*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)));
        J[1][9] = -dt*dt*(ay*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + az*(cos(phi)*cos(psi) +sin(phi)*sin(psi)*sin(theta)));
        J[2][9] = dt*dt*(ay*cos(phi)*cos(theta) - az*cos(theta)*sin(phi));
        J[3][9] = dt*(ay*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + az*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)));
        J[4][9] = -dt*(ay*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + az*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)));
        J[5][9] = dt*(ay*cos(phi)*cos(theta) - az*cos(theta)*sin(phi));
        J[6][9] = 0;
        J[7][9] = -dt*(gx*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - gy*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + gz*cos(phi)*cos(theta));
        J[8][9] = dt*(gy*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - gx*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + gz*cos(theta)*sin(phi));
        J[9][9] = dt*(q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta)) + 1;
        J[10][9] = -dt*(r*cos(phi) + q*sin(phi));
        J[11][9] = dt*((q*cos(phi))/cos(theta) - (r*sin(phi))/cos(theta));
        J[12][9] = 0;
        J[13][9] = 0;

        J[0][10] = dt*dt*(az*cos(phi)*cos(psi)*cos(theta) - ax*cos(psi)*sin(theta) + ay*cos(psi)*cos(theta)*sin(phi));
        J[1][10] = dt*dt*(az*cos(phi)*cos(theta)*sin(psi) - ax*sin(psi)*sin(theta) + ay*cos(theta)*sin(phi)*sin(psi));
        J[2][10] = -dt*dt*(ax*cos(theta) + az*cos(phi)*sin(theta) + ay*sin(phi)*sin(theta));
        J[3][10] = dt*(az*cos(phi)*cos(psi)*cos(theta) - ax*cos(psi)*sin(theta) + ay*cos(psi)*cos(theta)*sin(phi));
        J[4][10] = dt*(az*cos(phi)*cos(theta)*sin(psi) - ax*sin(psi)*sin(theta) + ay*cos(theta)*sin(phi)*sin(psi));
        J[5][10] = -dt*(ax*cos(theta) + az*cos(phi)*sin(theta) + ay*sin(phi)*sin(theta));
        J[6][10] = dt*(gz*cos(theta) + gx*cos(psi)*sin(theta) + gy*sin(psi)*sin(theta));
        J[7][10] = -dt*(gx*cos(psi)*cos(theta)*sin(phi) - gz*sin(phi)*sin(theta) + gy*cos(theta)*sin(phi)*sin(psi));
        J[8][10] = -dt*(gx*cos(phi)*cos(psi)*cos(theta) - gz*cos(phi)*sin(theta) + gy*cos(phi)*cos(theta)*sin(psi));
        J[9][10] = dt*(r*cos(phi)*(pow(tan(theta),2) + 1) + q*sin(phi)*(pow(tan(theta),2) + 1));
        J[10][10] = 1;
        J[11][10] = dt*((r*cos(phi)*sin(theta))/(cos(theta)*cos(theta))+q*sin(phi)*(pow(tan(theta),2)+1));
        J[12][10] = 0;
        J[13][10] = 0;

        J[0][11] = -dt*dt*(ay*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - az*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + ax*cos(theta)*sin(psi));
        J[1][11] = dt*dt*(az*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - ay*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + ax*cos(psi)*cos(theta));
        J[2][11] = 0;
        J[3][11] = -dt*(ay*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - az*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + ax*cos(theta)*sin(psi));
        J[4][11] = dt*(az*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - ay*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + ax*cos(psi)*cos(theta));
        J[5][11] = 0;
        J[6][11] = -dt*(gy*cos(psi)*cos(theta) - gx*cos(theta)*sin(psi));
        J[7][10] = dt*(gx*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) + gy*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)));
        J[8][11] = -dt*(gx*(cos(psi)*sin(phi)-cos(phi)*sin(psi)*sin(theta)) + gy*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)));
        J[9][11] = 0;
        J[10][11] = 0;
        J[11][11] = 1;
        J[12][11] = 0;
        J[13][11] = 0;

        J[0][12] = 0;    J[0][13] = 0;
        J[1][12] = 0;    J[1][13] = 0;
        J[2][12] = 0;    J[2][13] = 0;
        J[3][12] = 0;    J[3][13] = 0;
        J[4][12] = 0;    J[4][13] = 0;
        J[5][12] = 0;    J[5][13] = 0;
        J[6][12] = 0;    J[6][13] = 0;
        J[7][12] = 0;    J[7][13] = 0;
        J[8][12] = 0;    J[8][13] = 0;
        J[9][12] = 0;    J[9][13] = 0;
        J[10][12] = 0;   J[10][13] = 0;
        J[11][12] = 0;   J[11][13] = 0;
        J[12][12] = 0;   J[12][13] = 0;
        J[13][12] = 0;   J[13][13] = 0;

        return J;
    }
    // x = [Px Py Pz Vx Vu Vz u v w R P Y Vg chi]
    void startKalman() {
        filterThread = new Thread(new Runnable() {
            @Override
            public void run() {
                double[] x = new double[14];
                double[] y = new double[14];
                double[] U = new double[6];
                double[][] Jt = new double[14][14];
                double[][] P = new double[14][14];
                double[][] Q = new double[14][14];
                double[][] K = new double[14][14];
                double[][] C = new double[14][14];
                double[][] Ct = new double[14][14];
                double[][] R = new double[14][14];
                double[][] CPR = new double[14][14];
                double[][] I = new double[14][14];

                // Initialize state with values, position set to gps, velocity set to 0, angles set to acc and mag angles, angular velocity set to 0
                x[0] = 0;       x[1] = 0;       x[2] = 0;       x[3] = 0;       x[4] = 0;       x[5] = 0;
                x[6] = 0;       x[7] = 0;       x[8] = 0;       x[9] = accR;    x[10] = accP;   x[11] = accY;
                x[12] = 0;      x[13] = 0;

                // Initialize Q - process noise matrix and R sensor noise matrix
                for (int i = 0; i < 14; i++) {
                    Q[i][i] = 0.1;
                    R[i][i] = 0.1;
                    I[i][i] = 1;
                }

                while (!stopFilterThread) {
                    //Load input vector u
                    U[0] = u; U[1] = v; U[2] = w; U[3] = p; U[4] = q; U[5] = r;   //Initialize input with ned values
                    //Predict
                    x = stateTransistionFunction(x, U, 0.001);    //Calculate the priori estimate
                    J = jacobian(x, U, 0.001);      //Calculate the jacobian of F
                    Jt = LA.matrixTranspose(J);         //matrixTranspose(J);    //Take transpose of the jacobian
                    P = LA.matrixAdd(LA.matrixMultiply(LA.matrixMultiply(J,P),Jt),Q);   //Calculate priori covariance

                    //Update 1
                    y[0] = 0; y[1] = 0; y[2] = 0; y[3] = 0; y[4] = 0; y[5] = 0; y[6] = accR; y[7] = accP; y[8] = accY; y[9] = 0; y[10] = 0; y[11] = 0;
                    C[0][0] = 0; C[1][1] = 0; C[2][2] = 0; C[3][3] = 0; C[4][4] = 0; C[5][5] = 0; C[6][6] = 1; C[7][7] = 1; C[8][8] = 1; C[9][9] = 0; C[10][10] = 0; C[11][11] = 0;
                    //Intermediate steps for clarity
                    Ct = LA.matrixTranspose(C);    //Take inverse of C
                    CPR = LA.matrixInverse(LA.matrixAdd(LA.matrixMultiply(LA.matrixMultiply(C,P),Ct),R));  //Intermediate step for simplicity
                    K = LA.matrixMultiply(LA.matrixMultiply(P,Ct), CPR);   //calculate kalman gain
                    x = LA.vectAdd(x,LA.vectMatMultiply(K,LA.vectAdd(y,LA.vectMatMultiply(C,x),-1)),1);   //calculate posterior estimate
                    P = LA.matrixMultiply(LA.matrixSubtract(I,LA.matrixMultiply(K,C)), P);    //Update posterior covariance

                    //P is often 0 and therefore K is often 0 - something is wrong here.


                    //Loop thread every 1mS
                    try {
                        Thread.sleep(1);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                Log.i("KalmanThread","Shutting down");
                return;
            }
        });
    }


    SensorEventListener geoListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {

        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };
    SensorEventListener accListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            u = sensorEvent.values[0];
            v = sensorEvent.values[1];
            w = sensorEvent.values[2];
            accR = Math.atan2(sensorEvent.values[1], sensorEvent.values[2]) * rad2deg;
            accP = Math.atan2(sensorEvent.values[0], Math.sqrt(Math.pow(sensorEvent.values[1],2)+(Math.pow(sensorEvent.values[2],2))))*rad2deg;
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };
    SensorEventListener gyroListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            p = sensorEvent.values[0]*rad2deg;
            q = sensorEvent.values[1]*rad2deg;
            r = sensorEvent.values[2]*rad2deg;
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };
    SensorEventListener magListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            accY = Math.atan2(sensorEvent.values[1],sensorEvent.values[0])*rad2deg;
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };
}