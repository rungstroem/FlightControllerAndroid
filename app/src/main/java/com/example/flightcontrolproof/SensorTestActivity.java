package com.example.flightcontrolproof;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.sqrt;

import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.util.Log;
import android.widget.TextView;

import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import java.util.concurrent.locks.ReentrantLock;


public class SensorTestActivity extends Activity {
    SensorManager mSensorManger;
    HandlerThread sensorHandlerThread;
    Handler sensorThreadHandler;

    Thread filterThread;

    boolean GPSUpdate = false;
    boolean stopFilterThread = false;
    public ReentrantLock mutex = new ReentrantLock();
    LinearAlgebra LA;

    TextView accView;
    TextView gyroView;
    TextView gpsView;
    TextView magView;
    Location location;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_sensor_test);
        //Handler thread setup
        sensorHandlerThread = new HandlerThread("SensorHandlerThread");
        sensorHandlerThread.start();
        Looper sensorHandlerThreadLooper = sensorHandlerThread.getLooper();
        sensorThreadHandler = new Handler(sensorHandlerThreadLooper);

        //Textviews
        accView = findViewById(R.id.AccDataTest);
        gyroView = findViewById(R.id.GyroDataTest);
        gpsView = findViewById(R.id.GPSDataTest);
        magView = findViewById(R.id.MagDataTest);

        //Sensor setup
        mSensorManger = (SensorManager) this.getSystemService(Context.SENSOR_SERVICE);
        Sensor accelerometer = mSensorManger.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Sensor gyroscope = mSensorManger.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        Sensor magnetometer = mSensorManger.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        mSensorManger.registerListener(accListener, accelerometer, 1000, sensorThreadHandler);
        mSensorManger.registerListener(gyroListener, gyroscope, 1000, sensorThreadHandler);
        mSensorManger.registerListener(magListener, magnetometer, 1000, sensorThreadHandler);

        //Register GPS update receiver
        LocalBroadcastManager.getInstance(this).registerReceiver(GPSReceiver, new IntentFilter("GPSLocationUpdates"));

        //Initialize linear algebra library
        LA = new LinearAlgebra();

        //Initialize Kalman filtering in filterThread
        startKalman();
    }

    @Override
    protected void onStart(){
        super.onStart();

        //Start the Kalman filterThread
        filterThread.start();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        //filterThread.interrupt();
        stopFilterThread = true;
        filterThread.stop();
        unregisterReceiver(GPSReceiver);
        mSensorManger.unregisterListener(accListener);
        mSensorManger.unregisterListener(gyroListener);
        sensorHandlerThread.quit();
        stopService(new Intent(SensorTestActivity.this, GPSService.class));
    }

    //inputVect = [ax, ay, az, wx, wy, wz]
    //stateVect = [Px, Py, Pz, vx, vy, vz, psi, theta, phi, psiDot, thetaDot, phiDot]
    double[] stateTransistionMatrix(double[] stateVect, double inputVect[], double dt) {
        double[] prioriState = new double[12];

        //double psiDot   = inputVect[3] + inputVect[4]*Math.sin(stateVect[6])*Math.tan(stateVect[7]) + inputVect[5]*Math.cos(stateVect[6])*Math.tan(stateVect[7]);
        //double thetaDot = inputVect[4]*Math.cos(stateVect[6]) - inputVect[5]*Math.sin(stateVect[6]);
        //double phiDot   = ((inputVect[3]*Math.cos(stateVect[6]))/Math.cos(stateVect[7])) + ((inputVect[3]*Math.sin(stateVect[3]))/Math.cos(stateVect[7]));
        double psiDot   = stateVect[9] + inputVect[3] + inputVect[4]*Math.sin(stateVect[6])*Math.tan(stateVect[7]) + inputVect[5]*Math.cos(stateVect[6])*Math.tan(stateVect[7]);
        double thetaDot = stateVect[10] + inputVect[4]*Math.cos(stateVect[6]) - inputVect[5]*Math.sin(stateVect[6]);
        double phiDot   = stateVect[11] + ((inputVect[3]*Math.cos(stateVect[6]))/Math.cos(stateVect[7])) + ((inputVect[3]*Math.sin(stateVect[3]))/Math.cos(stateVect[7]));
        //double psi = psiDot*dt;
        //double theta = thetaDot*dt;
        //double phi = phiDot*dt;
        double psi = stateVect[6] + psiDot*dt;
        double theta = stateVect[7] + thetaDot*dt;
        double phi = stateVect[8] + phiDot*dt;
        // P(n+1) = P(n) + Rt*v(n)*dt + 0.5*(Rt*a(n)-g(n))*dt^2     - You may have to use the angles from the stateVector in the velocity rotation matrix, because it is the previous velocity you use so it may need to be the previous angles to
        prioriState[0]  = stateVect[0] + ((Math.cos(phi)*Math.cos(theta))*stateVect[3] + (-Math.cos(psi)*Math.sin(phi)+Math.cos(phi)*Math.sin(psi)*Math.sin(theta))*stateVect[4] + (Math.sin(phi)*Math.sin(psi)+Math.cos(phi)*Math.cos(psi)*Math.sin(theta))*stateVect[5])*dt + 0.5*(((Math.cos(phi)*Math.cos(theta))*inputVect[0] + (-Math.cos(psi)*Math.sin(phi)+Math.cos(phi)*Math.sin(psi)*Math.sin(theta))*inputVect[1] + ( Math.sin(phi)*Math.sin(psi)+Math.cos(phi)*Math.cos(psi)*Math.sin(theta))*inputVect[2])-Gx)*dt*dt;
        prioriState[1]  = stateVect[1] + ((Math.cos(theta)*Math.sin(phi))*stateVect[3] + (Math.cos(phi)*Math.cos(psi)+Math.sin(phi)*Math.sin(psi)*Math.sin(theta))*stateVect[4] + (-Math.cos(phi)*Math.sin(psi)+Math.cos(psi)*Math.sin(phi)*Math.sin(theta))*stateVect[5])*dt + 0.5*(((Math.cos(theta)*Math.sin(phi))*inputVect[0] + ( Math.cos(phi)*Math.cos(psi)+Math.sin(phi)*Math.sin(psi)*Math.sin(theta))*inputVect[1] + (-Math.cos(phi)*Math.sin(psi)+Math.cos(psi)*Math.sin(phi)*Math.sin(theta))*inputVect[2])-Gy)*dt*dt;
        prioriState[2]  = stateVect[2] + ((-Math.sin(theta))*stateVect[3] + (Math.cos(theta)*Math.sin(psi))*stateVect[4] + (Math.cos(psi)*Math.cos(theta))*stateVect[5])*dt + 0.5*(((-Math.sin(theta))*inputVect[0] + (Math.cos(theta)*Math.sin(psi))*inputVect[1] + (Math.cos(psi)*Math.cos(theta))*inputVect[2])-Gz)*dt*dt;
        prioriState[3]  = stateVect[3] + (inputVect[0]-Gx)*dt;
        prioriState[4]  = stateVect[4] + (inputVect[1]-Gy)*dt;
        prioriState[5]  = stateVect[5] + (inputVect[2]-Gz)*dt;
        //prioriState[6]  = stateVect[6] + psi;
        //prioriState[7]  = stateVect[7] + theta;
        //prioriState[8]  = stateVect[8] + phi;
        prioriState[6]  = psi;
        prioriState[7]  = theta;
        prioriState[8]  = phi;
        //prioriState[9]  = stateVect[9] + psiDot;
        //prioriState[10] = stateVect[10] + thetaDot;
        //prioriState[11] = stateVect[11] + phiDot;
        prioriState[9]  = psiDot;
        prioriState[10] = thetaDot;
        prioriState[11] = phiDot;

        return prioriState;
    }

    //inputVect = [ax, ay, az, wx, wy, wz]
    //vect = [Px, Py, Pz, vx, vy, vz, psi, theta, phi, psiDot, thetaDot, phiDot]
    double[][] jacobian(double[] vect, double[] input, double dt) {
        double[][] J = new double[12][12];
        J[0][0] = 1;    J[0][1] = 0;    J[0][2] = 0;    J[0][3] = Math.cos(vect[8])*Math.cos(vect[7])*dt;   J[0][4] = (Math.cos(vect[6])*Math.sin(vect[8])-Math.cos(vect[8])*Math.sin(vect[6])*Math.sin(vect[7]))*dt;   J[0][5] = ( Math.sin(vect[8])*Math.sin(vect[6])+Math.cos(vect[8])*Math.cos(vect[6])*Math.sin(vect[7]))*dt;  J[0][6] = ( (input[1]*(Math.sin(vect[8])*Math.sin(vect[6]) + Math.cos(vect[8])*Math.cos(vect[6])*Math.sin(vect[7])))/2 + (input[2]*(Math.cos(vect[6])*Math.sin(vect[8]) - Math.cos(vect[8])*Math.sin(vect[6])*Math.sin(vect[7])))/2)*dt*dt + ( vect[4]*(Math.sin(vect[8])*Math.sin(vect[6]) + Math.cos(vect[8])*Math.cos(vect[6])*Math.sin(vect[7])) + vect[5]*(Math.cos(vect[6])*Math.sin(vect[8]) - Math.cos(vect[8])*Math.sin(vect[6])*Math.sin(vect[7])))*dt;   J[0][7] = (-(input[2]*Math.cos(vect[8])*Math.sin(vect[7]))/2 + (input[2]*Math.cos(vect[8])*Math.cos(vect[6])*Math.cos(vect[7]))/2 + (input[1]*Math.cos(vect[8])*Math.cos(vect[7])*Math.sin(vect[6]))/2)*dt*dt + (- vect[3]*Math.cos(vect[8])*Math.sin(vect[7]) + vect[5]*Math.cos(vect[8])*Math.cos(vect[6])*Math.cos(vect[7]) + vect[4]*Math.cos(vect[8])*Math.cos(vect[7])*Math.sin(vect[6]))*dt; J[0][8] = (- (input[1]*(Math.cos(vect[8])*Math.cos(vect[6]) + Math.sin(vect[8])*Math.sin(vect[6])*Math.sin(vect[7])))/2 + (input[2]*(Math.cos(vect[8])*Math.sin(vect[6]) - Math.cos(vect[6])*Math.sin(vect[8])*Math.sin(vect[7])))/2 - (input[0]*Math.cos(vect[7])*Math.sin(vect[8]))/2)*dt*dt + (- vect[4]*(Math.cos(vect[8])*Math.cos(vect[6]) + Math.sin(vect[8])*Math.sin(vect[6])*Math.sin(vect[7])) + vect[5]*(Math.cos(vect[8])*Math.sin(vect[6]) - Math.cos(vect[6])*Math.sin(vect[8])*Math.sin(vect[7])) - vect[3]*Math.cos(vect[7])*Math.sin(vect[8]))*dt;    J[0][9] = 0;    J[0][10] = 0;   J[0][11] = 0;
        J[1][0] = 0;    J[1][1] = 1;    J[1][2] = 0;    J[1][3] = Math.cos(vect[7])*Math.sin(vect[8])*dt;   J[1][4] = (Math.cos(vect[8])*Math.cos(vect[6])+Math.sin(vect[8])*Math.sin(vect[6])*Math.sin(vect[7]))*dt;   J[1][5] = (-Math.cos(vect[8])*Math.sin(vect[6])+Math.cos(vect[6])*Math.sin(vect[8])*Math.sin(vect[7]))*dt;  J[1][6] = (-(input[1]*(Math.cos(vect[8])*Math.sin(vect[6]) - Math.cos(vect[6])*Math.sin(vect[8])*Math.sin(vect[7])))/2 - (input[2]*(Math.cos(vect[8])*Math.cos(vect[6]) + Math.sin(vect[8])*Math.sin(vect[6])*Math.sin(vect[7])))/2)*dt*dt + (-vect[4]*(Math.cos(vect[8])*Math.sin(vect[6]) - Math.cos(vect[6])*Math.sin(vect[8])*Math.sin(vect[7])) - vect[5]*(Math.cos(vect[8])*Math.cos(vect[6]) + Math.sin(vect[8])*Math.sin(vect[6])*Math.sin(vect[7])))*dt;   J[1][7] = (-(input[0]*Math.sin(vect[8])*Math.sin(vect[7]))/2 + (input[2]*Math.cos(vect[6])*Math.cos(vect[7])*Math.sin(vect[8]))/2 + (input[1]*Math.cos(vect[7])*Math.sin(vect[8])*Math.sin(vect[6]))/2)*dt*dt + (- vect[3]*Math.sin(vect[8])*Math.sin(vect[7]) + vect[5]*Math.cos(vect[6])*Math.cos(vect[7])*Math.sin(vect[8]) + vect[4]*Math.cos(vect[7])*Math.sin(vect[8])*Math.sin(vect[6]))*dt; J[1][8] = (- (input[1]*(Math.cos(vect[6])*Math.sin(vect[8]) - Math.cos(vect[8])*Math.sin(vect[6])*Math.sin(vect[7])))/2 + (input[2]*(Math.sin(vect[8])*Math.sin(vect[6]) + Math.cos(vect[8])*Math.cos(vect[6])*Math.sin(vect[7])))/2 + (input[0]*Math.cos(vect[8])*Math.cos(vect[7]))/2)*dt*dt + (- vect[4]*(Math.cos(vect[6])*Math.sin(vect[8]) - Math.cos(vect[8])*Math.sin(vect[6])*Math.sin(vect[7])) + vect[5]*(Math.sin(vect[8])*Math.sin(vect[6]) + Math.cos(vect[8])*Math.cos(vect[6])*Math.sin(vect[7])) + vect[3]*Math.cos(vect[8])*Math.cos(vect[7]))*dt;    J[1][9] = 0;    J[1][10] = 0;   J[1][11] = 0;
        J[2][0] = 0;    J[2][1] = 0;    J[2][2] = 1;    J[2][3] = -Math.sin(vect[7])*dt;                    J[2][4] = (Math.cos(vect[7])*Math.sin(vect[6]))*dt;                                                         J[2][5] = (Math.cos(vect[6])*Math.cos(vect[7]))*dt;                                                         J[2][6] = ((input[1]*Math.cos(vect[6])*Math.cos(vect[7]))/2 - (input[2]*Math.cos(vect[7])*Math.sin(vect[6]))/2)*dt*dt + (vect[4]*Math.cos(vect[6])*Math.cos(vect[7]) - vect[5]*Math.cos(vect[7])*Math.sin(vect[6]))*dt;                                                                                                                                                                                                                                             J[2][7] = (-(input[0]*Math.cos(vect[7]))/2 - (input[2]*Math.cos(vect[6])*Math.sin(vect[7]))/2 - (input[1]*Math.sin(vect[6])*Math.sin(vect[7]))/2)*dt*dt + (- vect[3]*Math.cos(vect[7]) - vect[5]*Math.cos(vect[6])*Math.sin(vect[7]) - vect[4]*Math.sin(vect[6])*Math.sin(vect[7]))*dt;                                                                                                             J[2][8] = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            J[2][9] = 0;    J[2][10] = 0;   J[2][11] = 0;
        J[3][0] = 0;    J[3][1] = 0;    J[3][2] = 0;    J[3][3] = 1;                                        J[3][4] = 0;                                                                                                J[3][5] = 0;                                                                                                J[3][6] = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                        J[3][7] = 0;                                                                                                                                                                                                                                                                                                                                                                                        J[3][8] = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            J[3][9] = 0;    J[3][10] = 0;   J[3][11] = 0;
        J[4][0] = 0;    J[4][1] = 0;    J[4][2] = 0;    J[4][3] = 0;                                        J[4][4] = 1;                                                                                                J[4][5] = 0;                                                                                                J[4][6] = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                        J[4][7] = 0;                                                                                                                                                                                                                                                                                                                                                                                        J[4][8] = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            J[4][9] = 0;    J[4][10] = 0;   J[4][11] = 0;
        J[5][0] = 0;    J[5][1] = 0;    J[5][2] = 0;    J[5][3] = 0;                                        J[5][4] = 0;                                                                                                J[5][5] = 1;                                                                                                J[5][6] = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                        J[5][7] = 0;                                                                                                                                                                                                                                                                                                                                                                                        J[5][8] = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            J[5][9] = 0;    J[5][10] = 0;   J[5][11] = 0;
        J[6][0] = 0;    J[6][1] = 0;    J[6][2] = 0;    J[6][3] = 0;                                        J[6][4] = 0;                                                                                                J[6][5] = 0;                                                                                                J[6][6] = 1;                                                                                                                                                                                                                                                                                                                                                                                                                                                        J[6][7] = 0;                                                                                                                                                                                                                                                                                                                                                                                        J[6][8] = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            J[6][9] = dt;   J[6][10] = 0;   J[6][11] = 0;
        J[7][0] = 0;    J[7][1] = 0;    J[7][2] = 0;    J[7][3] = 0;                                        J[7][4] = 0;                                                                                                J[7][5] = 0;                                                                                                J[7][6] = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                        J[7][7] = 1;                                                                                                                                                                                                                                                                                                                                                                                        J[7][8] = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            J[7][9] = 0;    J[7][10] = dt;  J[7][11] = 0;
        J[8][0] = 0;    J[8][1] = 0;    J[8][2] = 0;    J[8][3] = 0;                                        J[8][4] = 0;                                                                                                J[8][5] = 0;                                                                                                J[8][6] = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                        J[8][7] = 0;                                                                                                                                                                                                                                                                                                                                                                                        J[8][8] = 1;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            J[8][9] = 0;    J[8][10] = 0;   J[8][11] = dt;
        J[9][0] = 0;    J[9][1] = 0;    J[9][2] = 0;    J[9][3] = 0;                                        J[9][4] = 0;                                                                                                J[9][5] = 0;                                                                                                J[9][6] =  vect[4]*Math.cos(vect[6])*Math.tan(vect[7]) - vect[5]*Math.sin(vect[6])*Math.tan(vect[7]);                                                                                                                                                                                                                                                                                                                                                               J[9][7] = input[5]*Math.cos(vect[6])*(Math.tan(vect[7])*Math.tan(vect[7]) + 1) + input[5]*Math.sin(vect[6])*(Math.tan(vect[7])*Math.tan(vect[7]) + 1);                                                                                                                                                                                                                                              J[9][8] = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            J[9][9] = 1;    J[9][10] = 0;   J[9][11] = 0;
        J[10][0] = 0;   J[10][1] = 0;   J[10][2] = 0;   J[10][3] = 0;                                       J[10][4] = 0;                                                                                               J[10][5] = 0;                                                                                               J[10][6] = -vect[5]*Math.cos(vect[6]) - vect[4]*Math.sin(vect[6]);                                                                                                                                                                                                                                                                                                                                                                                                  J[10][7] = 0;                                                                                                                                                                                                                                                                                                                                                                                       J[10][8] = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           J[10][9] = 0;   J[10][10] = 1;  J[10][11] = 0;
        J[11][0] = 0;   J[11][1] = 0;   J[11][2] = 0;   J[11][3] = 0;                                       J[11][4] = 0;                                                                                               J[11][5] = 0;                                                                                               J[11][6] = (vect[4]*Math.cos(vect[6]))/Math.cos(vect[7]) - (vect[5]*Math.sin(vect[6]))/Math.cos(vect[7]);                                                                                                                                                                                                                                                                                                                                                           J[11][7] = (input[5]*Math.cos(vect[6])*Math.sin(vect[7]))/(Math.cos(vect[7])*Math.cos(vect[7])) + (input[4]*Math.sin(vect[6])*Math.sin(vect[7]))/(Math.cos(vect[7])*Math.cos(vect[7]));                                                                                                                                                                                                             J[11][8] = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           J[11][9] = 0;   J[11][10] = 0;  J[11][11] = 1;

        return J;
    }

    double[] xTemp = new double[3];
    void startKalman() {
        filterThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(GPSUpdate == false){
                    Log.i("GPSUpdate,", "Waiting for GPS");
                    continue;
                }
                double[][] J = new double[12][12];
                double[][] Jt = new double[12][12];
                double[] x = new double[12];
                double[] u = new double[6];
                double[][] P = new double[12][12];
                double[][] Q = new double[12][12];
                double[][] K = new double[12][12];
                double[][] C = new double[12][12];
                double[][] Ct = new double[12][12];
                double[][] R = new double[12][12];
                double[][] CPR = new double[12][12];
                double[] y = new double[12];
                double[][] I = new double[12][12];

                // Initialize state with values, position set to gps, velocity set to 0, angles set to acc and mag angles, angular velocity set to 0
                x[0] = GPSx;       x[1] = GPSy;       x[2] = GPSz;       x[3] = 0;   x[4] = 0;   x[5] = 0;
                x[6] = accR;    x[7] = accP;    x[8] = magY;    x[9] = 0;   x[10] = 0;  x[11] = 0;
                // Initialize Q - process noise matrix and R sensor noise matrix
                for (int i = 0; i < 12; i++) {
                    Q[i][i] = 0.1;
                    R[i][i] = 0.1;
                    I[i][i] = 1;
                    C[i][0] = 0;    C[i][1] = 0;    C[i][2] = 0;    C[i][3] = 0;    C[i][4] = 0;    C[i][5] = 0;
                    C[i][6] = 0;    C[i][7] = 0;    C[i][8] = 0;    C[i][9] = 0;    C[i][10] = 0;   C[i][11] = 0;
                    P[i][0] = 0;    P[i][1] = 0;    P[i][2] = 0;    P[i][3] = 0;    P[i][4] = 0;    P[i][5] = 0;
                    P[i][6] = 0;    P[i][7] = 0;    P[i][8] = 0;    P[i][9] = 0;    P[i][10] = 0;   P[i][11] = 0;
                }
                // Initialize the covariance matrix P
                P[0][0] = 10;  P[1][1] = 10;  P[2][2] = 10;  P[3][3] = 2;    P[4][4] = 2;    P[5][5] = 2;    P[6][6] = 1000; P[7][7] = 1000; P[8][8] = 1000; P[9][9] = 2;    P[10][10] = 2;  P[11][11] = 2;

                while (!stopFilterThread) {
                    //Load input vector u
                    u[0] = ax; u[1] = ay; u[2] = az; u[3] = wr; u[4] = wp; u[5] = wy;   //Initialize input with ned values
                    //Predict
                    x = stateTransistionMatrix(x, u, 0.001);    //Calculate the priori estimate
                    J = jacobian(x, u, 0.001);  //Calculate the jacobian of F
                    Jt = LA.matrixTranspose(J);//matrixTranspose(J);    //Take transpose of the jacobian
                    P = LA.matrixAdd(LA.matrixMultiply(LA.matrixMultiply(J,P),Jt),Q);//matrixAdd(matrixMultiply(matrixMultiply(J,P),Jt),Q);    //Calculate priori covariance

                    //Update 1
                    y[0] = 0; y[1] = 0; y[2] = 0; y[3] = 0; y[4] = 0; y[5] = 0; y[6] = accR; y[7] = accP; y[8] = magY; y[9] = 0; y[10] = 0; y[11] = 0;
                    C[0][0] = 0; C[1][1] = 0; C[2][2] = 0; C[3][3] = 0; C[4][4] = 0; C[5][5] = 0; C[6][6] = 1; C[7][7] = 1; C[8][8] = 1; C[9][9] = 0; C[10][10] = 0; C[11][11] = 0;
                    //Intermediate steps for clarity
                    Ct = LA.matrixTranspose(C);    //Take inverse of C
                    CPR = LA.matrixInverse(LA.matrixAdd(LA.matrixMultiply(LA.matrixMultiply(C,P),Ct),R));  //Intermediate step for simplicity
                    K = LA.matrixMultiply(LA.matrixMultiply(P,Ct), CPR);   //calculate kalman gain
                    x = LA.vectAdd(x,LA.vectMatMultiply(K,LA.vectAdd(y,LA.vectMatMultiply(C,x),-1)),1);   //calculate posterior estimate
                    P = LA.matrixMultiply(LA.matrixSubtract(I,LA.matrixMultiply(K,C)), P);    //Update posterior covariance

                    //P is often 0 and therefore K is often 0 - something is wrong here.

                    //Update 2
                    if(GPSUpdate == true) {

                        try{
                            mutex.lock();
                            GPSUpdate = false;
                        }finally {
                            mutex.unlock();
                        }
                        y[0] = GPSx; y[1] = GPSy; y[2] = GPSz; y[3] = 0; y[4] = 0; y[5] = 0; y[6] = accR; y[7] = accP; y[8] = magY; y[9] = 0; y[10] = 0; y[11] = 0;
                        C[0][0] = 1; C[1][1] = 1; C[2][2] = 1; C[3][3] = 0; C[4][4] = 0; C[5][5] = 0; C[6][6] = 1; C[7][7] = 1; C[8][8] = 1; C[9][9] = 0; C[10][10] = 0; C[11][11] = 0;
                        Ct = LA.matrixTranspose(C);
                        CPR = LA.matrixInverse(LA.matrixAdd(LA.matrixMultiply(LA.matrixMultiply(C,P),Ct),R));
                        K = LA.matrixMultiply(LA.matrixMultiply(P,Ct),CPR);
                        x = LA.vectAdd(x,LA.vectMatMultiply(K,LA.vectAdd(y,LA.vectMatMultiply(C,x),-1)),1);
                        P = LA.matrixMultiply(LA.matrixSubtract(I,LA.matrixMultiply(K,C)),P);

                        xTemp[0] = x[0]; xTemp[1] = x[1]; xTemp[2] = x[2];
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                gpsView.setText("N "+xTemp[0]+" E "+xTemp[1]+" A "+xTemp[2]);
                            }
                        });
                    }
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

    double rad2deg = 180.0/PI;

    double Gx = 0.0;
    double Gy = 0.0;
    double Gz = 0.0;
    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;
    double alpha = 0.0;
    double accR = 0.0;
    double accP = 0.0;
    SensorEventListener accListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            //Log.i("SensorUpdates","Accelerometer update");
            alpha = 0.8; //alpha = t/(t+dT) where t is the filter time constant

            //Low-pass filter to extract gravity - I'm not sure extracting the gravity is a good idea. This seems to do some kind of normalization of the accelerometer
            Gx = alpha*Gx+(1-alpha)*sensorEvent.values[0];
            Gy = alpha*Gy+(1-alpha)*sensorEvent.values[1];
            Gz = alpha*Gz+(1-alpha)*sensorEvent.values[2];

            //acceleration variables
            ax = sensorEvent.values[0];
            ay = sensorEvent.values[1];
            az = sensorEvent.values[2];

            //Euler angles
            accR = Math.atan2(sensorEvent.values[1], sensorEvent.values[2])*rad2deg;
            accP = -Math.atan2(sensorEvent.values[0], Math.sqrt(Math.pow(sensorEvent.values[1],2)+(Math.pow(sensorEvent.values[2],2))))*rad2deg;

            /*runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    //accView.setText("accR "+accR+"\naccP "+accP);
                    //accView.setText("gX "+gravityX+"\ngY "+gravityY+"\ngZ "+gravityZ);
                    //accView.setText("accX "+sensorEvent.values[0]+"\naccY "+sensorEvent.values[1]+"\naccZ "+sensorEvent.values[2]);
                }
            });*/
        }
        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {
            //Nothing to do
        }
    };

    private static final double NS2S = 1.0/1000000000.0;   //Nano-seconds to seconds conversion
    private double timestamp = 0.0;
    double wr = 0.0;
    double wp = 0.0;
    double wy = 0.0;
    //double dT;
    SensorEventListener gyroListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            wr = sensorEvent.values[0]*rad2deg;
            wp = sensorEvent.values[1]*rad2deg;
            wy = sensorEvent.values[2]*rad2deg;

            /*Log.i("SensorUpdates","Gyroscope update");
            if(timestamp != 0){
                //dT = (sensorEvent.timestamp-timestamp)*NS2S;
                //wr += sensorEvent.values[0]*dT*rad2deg;
                //wp += sensorEvent.values[1]*dT*rad2deg;
                //wy += sensorEvent.values[2]*dT*rad2deg;

            }
            timestamp = sensorEvent.timestamp;
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    //gyroView.setText("Roll "+wr+"\nPitch "+wp+"\nYaw "+wy);
                }
            });*/
        }
        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {
            //Nothing to do
        }
    };

    double magY = 0.0;
    SensorEventListener magListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            magY = Math.atan2(sensorEvent.values[1],sensorEvent.values[0])*rad2deg;

            /*runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    magView.setText("MagY "+magY);
                    //magView.setText("MagX "+sensorEvent.values[0]+"\nMagY "+sensorEvent.values[1]+"\nMagZ "+sensorEvent.values[2]);
                }
            });*/

        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };

    double GPSx;
    double GPSy;
    double GPSz;
    private BroadcastReceiver GPSReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            Bundle b = intent.getBundleExtra("Location");
            location = b.getParcelable("Location");
            double[] UTM = GPSToUTM(location);

            GPSx = UTM[0];
            GPSy = UTM[1];
            GPSz = UTM[2];


            try{
                mutex.lock();
                GPSUpdate = true;
            }catch (Exception e){
                //Nothing now
            }finally{
                mutex.unlock();
            }


            //gpsView.setText("Lat "+ location.getLatitude()+" Lon "+location.getLongitude()+" Alt "+location.getAltitude());
            //gpsView.setText("Eastings "+ UTM[0]+" Northings "+UTM[1]+" Alt "+UTM[2]);
        }
    };

    public double[] GPSToUTM(Location GPS){
        double[] UTM = new double[3];

        //Convert Lat Lon to UTM coordinates.
        int Zone;
        double Lat = GPS.getLatitude();
        double Lon = GPS.getLongitude();
        double Easting, Northing;
        Zone= (int) Math.floor(Lon/6+31);

        Easting=0.5*Math.log((1+Math.cos(Lat*Math.PI/180)*Math.sin(Lon*Math.PI/180-(6*Zone-183)*Math.PI/180))/(1-Math.cos(Lat*Math.PI/180)*Math.sin(Lon*Math.PI/180-(6*Zone-183)*Math.PI/180)))*0.9996*6399593.62/Math.pow((1+Math.pow(0.0820944379, 2)*Math.pow(Math.cos(Lat*Math.PI/180), 2)), 0.5)*(1+ Math.pow(0.0820944379,2)/2*Math.pow((0.5*Math.log((1+Math.cos(Lat*Math.PI/180)*Math.sin(Lon*Math.PI/180-(6*Zone-183)*Math.PI/180))/(1-Math.cos(Lat*Math.PI/180)*Math.sin(Lon*Math.PI/180-(6*Zone-183)*Math.PI/180)))),2)*Math.pow(Math.cos(Lat*Math.PI/180),2)/3)+500000;
        Easting=Math.round(Easting*100)*0.01;
        Log.i("UTMCoordinates","Easting "+Easting);

        Northing = (Math.atan(Math.tan(Lat*Math.PI/180)/Math.cos((Lon*Math.PI/180-(6*Zone -183)*Math.PI/180)))-Lat*Math.PI/180)*0.9996*6399593.625/ sqrt(1+0.006739496742*Math.pow(Math.cos(Lat*Math.PI/180),2))*(1+0.006739496742/2*Math.pow(0.5*Math.log((1+Math.cos(Lat*Math.PI/180)*Math.sin((Lon*Math.PI/180-(6*Zone -183)*Math.PI/180)))/(1-Math.cos(Lat*Math.PI/180)*Math.sin((Lon*Math.PI/180-(6*Zone -183)*Math.PI/180)))),2)*Math.pow(Math.cos(Lat*Math.PI/180),2))+0.9996*6399593.625*(Lat*Math.PI/180-0.005054622556*(Lat*Math.PI/180+Math.sin(2*Lat*Math.PI/180)/2)+4.258201531e-05*(3*(Lat*Math.PI/180+Math.sin(2*Lat*Math.PI/180)/2)+Math.sin(2*Lat*Math.PI/180)*Math.pow(Math.cos(Lat*Math.PI/180),2))/4-1.674057895e-07*(5*(3*(Lat*Math.PI/180+Math.sin(2*Lat*Math.PI/180)/2)+Math.sin(2*Lat*Math.PI/180)*Math.pow(Math.cos(Lat*Math.PI/180),2))/4+Math.sin(2*Lat*Math.PI/180)*Math.pow(Math.cos(Lat*Math.PI/180),2)*Math.pow(Math.cos(Lat*Math.PI/180),2))/3);
        Northing=Math.round(Northing*100)*0.01;
        Log.i("UTMCoordinates","Northing "+Northing);

        UTM[0] = Northing;  UTM[1] = Easting;   UTM[2] = GPS.getAltitude();
        return UTM;
    }

}

