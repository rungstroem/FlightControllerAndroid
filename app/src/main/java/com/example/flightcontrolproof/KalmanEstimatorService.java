package com.example.flightcontrolproof;

import static java.lang.Math.E;
import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;

import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
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

import java.util.concurrent.locks.ReentrantLock;

public class KalmanEstimatorService extends Service {
    SensorManager mSensorManager;
    HandlerThread sensorHandlerThread;
    Handler sensorThreadHandler;
    Looper sensorThreadLooper;

    Sensor accelerometer;
    Sensor gravity;
    Sensor gyroscope;
    Sensor magnetometer;
    Sensor pressure;
    Sensor orientation;
    Sensor rotation;

    double groundlevelPresure;    //This should be calibrated before flight. Put drone on the ground and measure average pressure.
    double ambientTemperature;
    double groundlevelGPS;
    double rad2deg = 180.0/PI;

    volatile boolean threadInterrupt = false;
    Thread complementaryThread;
    Thread kalmanThread;
    LinearAlgebra LA;

    double[] vehicleState = {0,0,0,0,0,0,0,0,0,0};  //Initialize to 0??
    double[] UTMPos = new double[4];
    double[] vel_N = {0,0,0};
    volatile boolean newGPSData = false;
    ReentrantLock mutex;

    double p;
    double q;
    double r;
    double accR;
    double accP;
    double accY;
    double[] estAngles = {accR, accP, accY};

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
        //Instantiate mutex lock
        mutex = new ReentrantLock();

        //Sensor thread create
        sensorHandlerThread = new HandlerThread("Sensor handler thread");

        //Register GPS receiver
        LocalBroadcastManager.getInstance(this).registerReceiver(GPSReceiver, new IntentFilter("GPSLocationUpdate"));

        //Access to linear algebra class
        LA = new LinearAlgebra();

        //Sensor create
        mSensorManager = (SensorManager) this.getSystemService(Context.SENSOR_SERVICE);
        accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);           //accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER); - accelerometer values with gravity
        gravity = mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
        gyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        pressure = mSensorManager.getDefaultSensor(Sensor.TYPE_PRESSURE);
        orientation = mSensorManager.getDefaultSensor(Sensor.TYPE_ORIENTATION);     //Older android phones have this senor  //rotation = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);    //Newer android phones have this sensor

        //Get calibration data
        SharedPreferences mSharedPref = getSharedPreferences("CalibrationData", Context.MODE_PRIVATE);
        groundlevelPresure = getPreference(mSharedPref, "BarometerCalibration", 1027);
        ambientTemperature = getPreference(mSharedPref, "AmbientTemperature", 20) + 273.15;     //Temp in degrees C + 273.15K
        groundlevelGPS = getPreference(mSharedPref, "GPSAltitudecalibration",37 );              //37m is ground altitude above sea level in Odense
        //Filter threads initialization
        complementaryThread = new Thread(complementaryFilterAttitude);
        kalmanThread = new Thread(KalmanFilter);
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startID){
        //Sensor thread handle and looper
        sensorHandlerThread.start();
        sensorThreadLooper = sensorHandlerThread.getLooper();
        sensorThreadHandler = new Handler(sensorThreadLooper);

        //Sensor listeners
        mSensorManager.registerListener(accListener, accelerometer, 5000, sensorThreadHandler);     //5000µS is 200Hz
        mSensorManager.registerListener(gravityListener, gravity, 5000, sensorThreadHandler);
        mSensorManager.registerListener(gyroListener, gyroscope, 5000, sensorThreadHandler);
        mSensorManager.registerListener(presListener, pressure, 5000, sensorThreadHandler);
        mSensorManager.registerListener(magListener, magnetometer,5000, sensorThreadHandler);
        mSensorManager.registerListener(orientListener, orientation, 5000, sensorThreadHandler);    //Only on older phones
        //mSensorManager.registerListener(rotListener, rotation, 5000, sensorThreadHandler);    //Only on newer phones

        //Filter threads start
        complementaryThread.start();
        //kalmanThread.start();     //This is started from the GPS broadcast receiver so the kalman filter only starts when GPS is available

        Log.i("SystemState","Kalman service started");

        return START_STICKY;
    }
    public void onDestroy(){
        sensorThreadLooper.quit();  //Also to stop sensor thread
        sensorHandlerThread.quit(); //Stops the sensor thread
        threadInterrupt = true;

        mSensorManager.unregisterListener(accListener);
        mSensorManager.unregisterListener(presListener);
        mSensorManager.unregisterListener(gravityListener);
        mSensorManager.unregisterListener(gyroListener);
        mSensorManager.unregisterListener(magListener);
        mSensorManager.unregisterListener(orientListener);
        //mSensorManager.unregisterListener(rotListener);
        Log.i("SystemState","Kalman Service stopped");
    }

    double getPreference(final SharedPreferences prefs, final String key, final double defaultValue) {
        if ( !prefs.contains(key))
            return defaultValue;
        return Double.longBitsToDouble(prefs.getLong(key, 0));
    }

    public void sendDataToActivity(double[] stateVector){
        Intent intent = new Intent("attitudeUpdate");
        intent.putExtra("stateVector", stateVector);
        LocalBroadcastManager.getInstance(this).sendBroadcast(intent);
    }

    private Runnable complementaryFilterAttitude = new Runnable() {     //This actually works
        double[] accAngles = new double[3];
        double[] gyroRates = new double[3];
        double[] gyroAngles = new double[3];
        double[] gyroAngleSum = {accR, accP, -accY};
        double[] compAngles = {accR, accP, -accY};
        double[][] TMat_BN;
        double[][] RMat_BN;
        double K = 0.02;    //Tuning parameter
        double dt = 0.005;  //5mS

        double[][] T_BN(double R, double P){
            double[][] TMat = new double[3][3];
            TMat[0][0] = 1;     TMat[0][1] = sin(R)*tan(P);     TMat[0][2] = cos(R)*tan(P);
            TMat[1][0] = 0;     TMat[1][1] = cos(R);            TMat[1][2] = (-sin(R));
            TMat[2][0] = 0;     TMat[2][1] = sin(R)/cos(P);     TMat[2][2] = cos(R)/cos(P);
            return TMat;
        }

        double[][] R_BN(double R, double P, double Y){
            double[][] RMat = new double[3][3];
            RMat[0][0] = cos(Y) * cos(P);   RMat[0][1] = (-sin(Y))*cos(R) + cos(Y)*sin(P)*sin(R);   RMat[0][2] = sin(Y)*sin(R) + cos(Y)*cos(R)*sin(P);
            RMat[1][0] = sin(Y) * cos(P);   RMat[1][1] = cos(Y)*cos(R) + sin(R)*sin(P)*sin(Y);      RMat[1][2] = (-cos(Y))*sin(R) + sin(P)*sin(Y)*cos(R);
            RMat[2][0] = (-sin(P));         RMat[2][1] = cos(P)*sin(R);                             RMat[2][2] = cos(P)*cos(R);
            return RMat;
        }

        double[] magCorr = new double[3];
        double magY;
        double lastMagY;

        @Override
        public void run() {
            while (!threadInterrupt) {

                RMat_BN = R_BN(compAngles[0],compAngles[1],compAngles[2]);
                magCorr = LA.vectMatMultiply(RMat_BN, magData);
                magY = Math.atan2(magCorr[1],magCorr[0])*rad2deg;    //atan2(East, North) - atan2(S.values[1],S.values[0])
                lastMagY += (magY-lastMagY)/100;

                accAngles[0] = accR;
                accAngles[1] = accP;
                accAngles[2] = -lastMagY;//lastAccY;
                gyroRates[0] = lastRates[0];    //p
                gyroRates[1] = lastRates[1];    //q
                gyroRates[2] = lastRates[2];    //r
                //gyroRates[0] = p;
                //gyroRates[1] = q;
                //gyroRates[2] = r;

                TMat_BN = T_BN(compAngles[0], -compAngles[1]);
                gyroAngles = LA.vectConstMultiply(LA.vectMatMultiply(TMat_BN, gyroRates), dt);

                gyroAngles[1] = -gyroAngles[1];

                //For test
                //gyroAngleSum[0] += gyroAngles[0];
                //gyroAngleSum[1] += -gyroAngles[1];
                //gyroAngleSum[2] += -gyroAngles[2];
                //
                //compAngles[0] = (1-K)*(compAngles[0] + gyroAngles[0]) +K*accAngles[0];
                //compAngles[1] = (1-K)*(compAngles[1] + gyroAngles[1]) +K*accAngles[1];
                //compAngles[2] = (1-K)*(compAngles[2] + gyroAngles[2]) +K*accAngles[2];

                //For test
                //gyroAngleSum[0] = compAngles[0] + gyroAngles[0];
                //gyroAngleSum[1] = compAngles[1] + gyroAngles[1];
                //gyroAngleSum[2] = compAngles[2] + gyroAngles[2];
                //gyroAngleSum[0] += -gyroAngles[0];
                //gyroAngleSum[1] += gyroAngles[1];
                //gyroAngleSum[2] += gyroAngles[2];
                gyroAngleSum[0] += gyroRates[0]*dt;
                gyroAngleSum[1] += gyroRates[1]*dt;
                gyroAngleSum[2] += gyroRates[2]*dt;
                //
                compAngles[0] = (1-K)*(compAngles[0] + gyroRates[0]*dt) +K*accAngles[0];
                compAngles[1] = (1-K)*(compAngles[1] + gyroRates[1]*dt) +K*accAngles[1];
                compAngles[2] = (1-K)*(compAngles[2] + gyroRates[2]*dt) +K*accAngles[2];

                //Log.i("ComplementaryFilter", "R " + compAngles[0] + " P " + compAngles[1] + " Y " + compAngles[2]+" accR "+accAngles[0]+" accP "+accAngles[1]+" accY "+accAngles[2]+
                //        " gyroP "+lastGyro[0]+" gyroQ "+lastGyro[1]+" gyroR "+lastGyro[2] + " gyroR "+gyroAngleSum[0]+" gyroP "+gyroAngleSum[1]+" gyroY "+gyroAngleSum[2]);

                Log.i("ComplementaryFilter", "R " + compAngles[0] + " P " + compAngles[1] + " Y " + compAngles[2]+" accR "+accAngles[0]+" accP "+accAngles[1]+" accY "+accAngles[2]+
                        " gyroP "+gyroAngles[0]+" gyroQ "+gyroAngles[1]+" gyroR "+gyroAngles[2]+ " gyroR "+gyroAngleSum[0]+" gyroP "+gyroAngleSum[1]+" gyroY "+gyroAngleSum[2]);


                //estAngles[0] += gyroAngles[0];
                //estAngles[1] += gyroAngles[1];
                //estAngles[2] += gyroAngles[2];

                //Actual filter algorithm
                //estAngles[0] = (1 - K) * estAngles[0] + K * accAngles[0];
                //estAngles[1] = (1 - K) * estAngles[1] + K * accAngles[1];
                //estAngles[2] = (1 - K) * estAngles[2] + K * accAngles[2];

                //vehicleState[0] = estAngles[0]; vehicleState[1] = estAngles[1]; vehicleState[2] = estAngles[2];     //Euler angles for stability controllers
                //vehicleState[3] = p;            vehicleState[4] = q;            vehicleState[5] = r;                //Gyro rates for damper controllers

                //sendDataToActivity(vehicleState);
                //Log.i("ComplementaryFilter", "R " + estAngles[0] + " P " + estAngles[1] + " Y " + estAngles[2]);
                //Log.i("ComplementaryFilter", "R " + estAngles[0] + " P " + estAngles[1] + " Y " + estAngles[2]+" accR "+accAngles[0]+" accP "+accAngles[1]+" accY "+accAngles[2]+" gyroP "+p+" gyroQ "+q+" gyroR "+r + " gyroR "+gyroAngleSum[0]+" gyroP "+gyroAngleSum[1]+" gyroY "+gyroAngleSum[2]);


                //Loop thread every 5mS - 200Hz
                try {
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    };

    private Runnable KalmanFilter = new Runnable() {
        double[][] R_BN(double R, double P, double Y){
            double[][] RMat = new double[3][3];
            RMat[0][0] = cos(Y) * cos(P);   RMat[0][1] = (-sin(Y))*cos(R) + cos(Y)*sin(P)*sin(R);   RMat[0][2] = sin(Y)*sin(R) + cos(Y)*cos(R)*sin(P);
            RMat[1][0] = sin(Y) * cos(P);   RMat[1][1] = cos(Y)*cos(R) + sin(R)*sin(P)*sin(Y);      RMat[1][2] = (-cos(Y))*sin(R) + sin(P)*sin(Y)*cos(R);
            RMat[2][0] = (-sin(P));         RMat[2][1] = cos(P)*sin(R);                             RMat[2][2] = cos(P)*cos(R);
            return RMat;
        }
        double dt = 0.05;
        int speedResetCounter = 0;
        double[][] RMat;
        double[] acc_N;

        double[][] p = new double[6][6];
        double[] x = new double[6];

        double[] xm = new double[6];
        double[] xp = new double[6];
        double[] gps = new double[6];

        double[][] K = new double[6][6];
        double[][] S = new double[6][6];
        double[][] pm = new double[6][6];
        double[][] pp = {{10,0,0,0,0,0},{0,10,0,0,0,0},{0,0,10,0,0,0},{0,0,0,10,0,0},{0,0,0,0,10,0},{0,0,0,0,0,10}};

        double[][] A = {{1,0,0,dt,0,0},{0,1,0,0,dt,0},{0,0,1,0,0,dt},{0,0,0,1,0,0},{0,0,0,0,1,0},{0,0,0,0,0,1}};
        //double[][] A = {{1,0,0,1,0,0},{0,1,0,0,1,0},{0,0,1,0,0,1},{0,0,0,1,0,0},{0,0,0,0,1,0},{0,0,0,0,0,1}};
        double[][] B = {{0.5*dt*dt,0,0},{0,0.5*dt*dt,0},{0,0,0.5*dt*dt},{dt,0,0},{0,dt,0},{0,0,dt}};
        //double[][] B = {{0.5*dt,0,0},{0,0.5*dt,0},{0,0,0.5*dt},{1,0,0},{0,1,0},{0,0,1}};
        double[][] H = {{1,0,0,0,0,0},{0,1,0,0,0,0},{0,0,1,0,0,0},{0,0,0,1,0,0},{0,0,0,0,1,0},{0,0,0,0,0,1}};
        double[][] R = {{0.2,0,0,0,0,0},{0,0.2,0,0,0,0},{0,0,0.2,0,0,0},{0,0,0,0.2,0,0},{0,0,0,0,0.2,0},{0,0,0,0,0,0.2}};
        double[][] Q = {{0.2,0,0,0,0,0},{0,0.2,0,0,0,0},{0,0,10,0,0,0},{0,0,0,0.2,0,0},{0,0,0,0,0.2,0},{0,0,0,0,0,10}};
        double[][] I = {{1,0,0,0,0,0},{0,1,0,0,0,0},{0,0,1,0,0,0},{0,0,0,1,0,0},{0,0,0,0,1,0},{0,0,0,0,0,1}};

        //Altitude kalman
        double xmAltitude;
        double xpAltitude;
        double pmAltitude;
        double ppAltitude = 2;
        double QAltitude = 5;
        double RAltitude = 0.1;
        double KAltitude;


        @Override
        public void run() {
            xp[0] = UTMPos[0]; xp[1] = UTMPos[1]; xp[2] = UTMPos[2]; xp[3] = vel_N[0]; xp[4] = vel_N[1]; xp[5] = vel_N[2];
            xpAltitude = UTMPos[2];

            while (!threadInterrupt) {
                //Transform acceleration
                RMat = R_BN(estAngles[0], estAngles[1], estAngles[2]);    //Creates the DCM from body to ned
                acc_N = LA.vectMatMultiply(RMat, acc_B);
                Log.i("AccN","AccN"+acc_N[0]+" "+acc_N[1]+" "+acc_N[2]);

                //Predict step
                x = LA.vectAdd(LA.vectMatMultiply(A,x),LA.vectMatMultiply(B,acc_N),1);
                //Log.i("XM","XM "+xm[0]+" "+xm[1]+" "+xm[2]+" "+xm[3]+" "+xm[4]+" "+xm[5]);
                p = LA.matrixAdd(LA.matrixMultiply(LA.matrixMultiply(A,p),LA.matrixTranspose(A)), Q);
                //Log.i("PM","PM "+p[0][0]+" "+p[1][1]+" "+p[2][2]+" "+p[3][3]+" "+p[4][4]+" "+p[5][5]);

                //Altitude Kalman - A 2 input 1 output additional kalman filter
                //xm[2] = xmAltitude = barAltitude;
                x[2] = xmAltitude = barAltitude;
                pmAltitude = ppAltitude + QAltitude;
                //pm[2][2] = pmAltitude;
                p[2][2] = pmAltitude;

                //Set vehicle state
                //vehicleState[6] = xm[0];
                //vehicleState[7] = xm[1];
                //vehicleState[8] = xm[2];
                //xm[5] = 0;
                //pm[5][5] = 0;
                vehicleState[6] = x[0];
                vehicleState[7] = x[1];
                vehicleState[8] = x[2];
                x[5] = 0;
                p[5][5] = 0;
                if(acc_B[0] == 0 && acc_B[1] == 0){
                    speedResetCounter++;
                }else{
                    speedResetCounter = 0;
                }
                if(speedResetCounter > 25){
                    //xm[3] = 0;
                    //xm[4] = 0;
                    x[3] = 0;
                    x[4] = 0;
                }
                Log.i("XM","XM "+x[0]+" "+x[1]+" "+x[2]+" "+x[3]+" "+x[4]+" "+x[5]);
                Log.i("PM","PM "+p[0][0]+" "+p[1][1]+" "+p[2][2]+" "+p[3][3]+" "+p[4][4]+" "+p[5][5]);

                //Update step
                if (newGPSData) {
                    gps[0] = UTMPos[0]; gps[1] = UTMPos[1]; gps[2] = UTMPos[2]; gps[3] = vel_N[0]; gps[4] = vel_N[1]; gps[5] = vel_N[2];
                    S = LA.matrixAdd(LA.matrixMultiply(LA.matrixMultiply(H,p),LA.matrixTranspose(H)),R);
                    Log.i("SMatrix", "SMatrix "+S[0][0]+" "+S[1][1]+" "+S[2][2]+" "+S[3][3]+" "+S[4][4]+" "+S[5][5]);
                    //K = LA.matrixMultiply(LA.matrixMultiply(pm,LA.matrixTranspose(H)),LA.matrixInverse(S));
                    K = LA.matrixMultiply(p, LA.matrixMultiply(LA.matrixTranspose(H),LA.matrixInverse(S)));
                    if(K[5][5] < 0.0000001) K[5][5] = 0;
                    Log.i("K","K "+K[0][0]+" "+K[1][1]+" "+K[2][2]+" "+K[3][3]+" "+K[4][4]+" "+K[5][5]);
                    x = LA.vectAdd(x,LA.vectMatMultiply(K,LA.vectAdd(gps, x,-1)),1);
                    //Log.i("XP","XP "+x[0]+" "+x[1]+" "+x[2]+" "+x[3]+" "+x[4]+" "+x[5]);
                    p = LA.matrixMultiply(LA.matrixSubtract(I,LA.matrixMultiply(K,H)),p);
                    if(pp[5][5] < 0.0000001) pp[5][5] = 0;
                    //Log.i("PP","PP "+p[0][0]+" "+p[1][1]+" "+p[2][2]+" "+p[3][3]+" "+p[4][4]+" "+p[5][5]);

                    //Altitude Kalman
                    KAltitude = pmAltitude/(pmAltitude+RAltitude);
                    xpAltitude = xmAltitude + KAltitude*(UTMPos[2]-xmAltitude);
                    //xp[2] = xpAltitude;
                    x[2] = xpAltitude;
                    ppAltitude = (1-KAltitude)*pmAltitude;
                    p[2][2] = ppAltitude;
                    Log.i("AltitudeEstimate","EstAlt "+xpAltitude+" KGain "+KAltitude);

                    Log.i("XP","XP "+x[0]+" "+x[1]+" "+x[2]+" "+x[3]+" "+x[4]+" "+x[5]);
                    Log.i("PP","PP "+p[0][0]+" "+p[1][1]+" "+p[2][2]+" "+p[3][3]+" "+p[4][4]+" "+p[5][5]);

                    //Set vehicle state
                    //vehicleState[6] = xp[0];
                    //vehicleState[7] = xp[1];
                    //vehicleState[8] = xp[2];
                    //vehicleState[9] = UTMPos[3];
                    vehicleState[6] = x[0];
                    vehicleState[7] = x[1];
                    vehicleState[8] = x[2];
                    vehicleState[9] = UTMPos[3];

                    try {
                        mutex.lock();
                        newGPSData = false;
                    } finally {
                        mutex.unlock();
                    }
                }

                //Loop thread every 5mS - 200Hz
                try {
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    };

    private Runnable kalmanFilterPosition = new Runnable() {
        double[][] R_BN(double R, double P, double Y){
            double[][] RMat = new double[3][3];
            RMat[0][0] = cos(Y) * cos(P);   RMat[0][1] = (-sin(Y))*cos(R) + cos(Y)*sin(P)*sin(R);   RMat[0][2] = sin(Y)*sin(R) + cos(Y)*cos(R)*sin(P);
            RMat[1][0] = sin(Y) * cos(P);   RMat[1][1] = cos(Y)*cos(R) + sin(R)*sin(P)*sin(Y);      RMat[1][2] = (-cos(Y))*sin(R) + sin(P)*sin(Y)*cos(R);
            RMat[2][0] = (-sin(P));         RMat[2][1] = cos(P)*sin(R);                             RMat[2][2] = cos(P)*cos(R);
            return RMat;
        }
        double dt = 0.05;
        double[][] RMat;
        double[] acc_N;

        double[] xm_n = new double[2];
        double[] xp_n = new double[2];
        double[] xm_e = new double[2];
        double[] xp_e = new double[2];
        double[] gps_n = new double[2];
        double[] gps_e = new double[2];

        double[][] K_n;
        double[][] S_n;
        double[][] pm_n;
        double[][] pp_n = {{0.1,0},{0,10}};
        double[][] K_e;
        double[][] S_e;
        double[][] pm_e;
        double[][] pp_e = {{0.1,0},{0,10}};

        double[][] A = {{1, dt},{0,1}};
        double[] B = {0.5*dt*dt,dt};
        double[][] H = {{1,0},{0,1}};
        double[][] R = {{0.2,0},{0,0.2}};
        double[][] Q = {{0.2,0},{0,0.2}};
        double[][] I = {{1,0},{0,1}};


        @Override
        public void run() {
            xp_n[0] = UTMPos[0]; xp_n[1] = 0;
            xp_e[0] = UTMPos[1]; xp_e[1] = 0;
            while (!threadInterrupt) {
                //Transform acceleration
                RMat = R_BN(estAngles[0], estAngles[1], estAngles[2]);    //Creates the DCM from body to ned
                acc_N = LA.vectMatMultiply(RMat, acc_B);
                Log.i("AccN","AccN"+acc_N[0]+" "+acc_N[1]+" "+acc_N[2]);

                //Predict step
                xm_n = LA.vectAdd(LA.vectMatMultiply(A,xp_n),LA.vectConstMultiply(B,acc_N[0]),1);
                Log.i("XM", "XMn "+xm_n[0]+" VNn "+xm_n[1]);
                pm_n = LA.matrixAdd(LA.matrixMultiply(LA.matrixMultiply(A,pp_n),LA.matrixTranspose(A)), Q);
                Log.i("PM","PNn "+pm_n[0][0]+" VNn "+pm_n[1][1]);
                xm_e = LA.vectAdd(LA.vectMatMultiply(A,xp_e),LA.vectConstMultiply(B,acc_N[1]),1);
                Log.i("XM", "XMe "+xm_e[0]+" VNe "+xm_e[1]);
                pm_e = LA.matrixAdd(LA.matrixMultiply(LA.matrixMultiply(A,pp_e),LA.matrixTranspose(A)), Q);
                Log.i("PM","PNe "+pm_e[0][0]+" VNe "+pm_e[1][1]);

                //Update step
                if (newGPSData) {
                    gps_n[0] = UTMPos[0]; gps_n[1] = vel_N[0];  gps_e[0] = UTMPos[1];   gps_e[1] = vel_N[1];
                    S_n = LA.matrixAdd(LA.matrixMultiply(LA.matrixMultiply(H,pm_n),LA.matrixTranspose(H)),R);
                    K_n = LA.matrixMultiply(LA.matrixMultiply(pm_n,LA.matrixTranspose(H)), LA.matrixInverse(S_n));
                    Log.i("K", "Kn "+K_n[0][0]+" "+K_n[0][1]+" "+K_n[1][0]+" "+K_n[1][1]);

                    S_e = LA.matrixAdd(LA.matrixMultiply(LA.matrixMultiply(H,pm_e),LA.matrixTranspose(H)),R);
                    K_e = LA.matrixMultiply(LA.matrixMultiply(pm_e,LA.matrixTranspose(H)),LA.matrixInverse(S_e));
                    Log.i("K", "Ke "+K_e[0][0]+" "+K_e[0][1]+" "+K_e[1][0]+" "+K_e[1][1]);

                    xp_n = LA.vectAdd(xm_n,LA.vectMatMultiply(K_n,LA.vectAdd(gps_n, xm_n,-1)),1);
                    Log.i("XP","XPn "+xp_n[0]+" VP "+xp_n[1]);
                    pp_n = LA.matrixSubtract(I,LA.matrixMultiply(K_n,pm_n));
                    Log.i("PP", "PPn "+pp_n[0][0]+" VP "+pp_n[1][1]);
                    xp_e = LA.vectAdd(xm_e,LA.vectMatMultiply(K_e,LA.vectAdd(gps_e, xm_e,-1)),1);
                    Log.i("XP","XPe "+xp_e[0]+" VP "+xp_e[1]);
                    pp_e = LA.matrixSubtract(I,LA.matrixMultiply(K_e,pm_e));
                    Log.i("PP", "PPe "+pp_e[0][0]+" VP "+pp_e[1][1]);
                    try {
                        mutex.lock();
                        newGPSData = false;
                    } finally {
                        mutex.unlock();
                    }
                }

                //Loop thread every 5mS - 200Hz
                try {
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    };

    //Sensor listeners
    double[] acc_B = new double[3];
    double[] acc = new double[3];
    double accFilterConstant = 4;
    SensorEventListener accListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            acc[0] = sensorEvent.values[1];   //X-axis
            acc[1] = sensorEvent.values[0];   //Y-axis
            acc[2] = sensorEvent.values[2];   //Z-axis

            acc_B[0] += (acc[0]-acc_B[0])/accFilterConstant;
            acc_B[1] += (acc[1]-acc_B[1])/accFilterConstant;
            acc_B[2] += (acc[2]-acc_B[2])/accFilterConstant;
            if(acc_B[0] < 0.2 && acc_B[0] > -0.2) acc_B[0] = 0;
            if(acc_B[1] < 0.2 && acc_B[1] > -0.2) acc_B[1] = 0;
            if(acc_B[2] < 1 && acc_B[2] > -1) acc_B[2] = 0;
            Log.i("LinearAccelerometer", "x "+acc_B[0]+" y "+acc_B[1]+" z "+acc_B[2]);
        }
        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };

    SensorEventListener gravityListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            accR = Math.atan2(sensorEvent.values[0], sensorEvent.values[2])*rad2deg;    // atan2(y,z)
            accP = Math.atan2(-sensorEvent.values[1], Math.sqrt(Math.pow(sensorEvent.values[0],2)+(Math.pow(sensorEvent.values[2],2))))*rad2deg;    // atan2(-x,sqrt(y²+z²))
            Log.i("Accelerometer","Acc generated euler angles "+" R "+accR+" P "+accP);
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };

    double[] lastRates = new double[3];
    double[] lastGyro = new double[3];
    double gyroFilterConst = 2;
    SensorEventListener gyroListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            p = -sensorEvent.values[1]*rad2deg;  //X-axis
            q = -sensorEvent.values[0]*rad2deg;  //Y-axis
            r = -sensorEvent.values[2]*rad2deg;  //Z-axis

            lastRates[0] += (p-lastRates[0])/gyroFilterConst;
            lastRates[1] += (q-lastRates[1])/gyroFilterConst;
            lastRates[2] += (r-lastRates[2])/gyroFilterConst;

            lastGyro[0] += (p-lastGyro[0])/4;
            lastGyro[1] += (q-lastGyro[1])/4;
            lastGyro[2] += (r-lastGyro[2])/4;
            //vehicleState[3] = lastRates[0];
            //vehicleState[4] = lastRates[1];
            //vehicleState[5] = lastRates[2];
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };

    double mNorth;
    double mEast;
    double lastAccY;
    double magFilterConst = 4;
    double[] magData = new double[3];
    SensorEventListener magListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            mNorth = sensorEvent.values[1];
            mEast = sensorEvent.values[0];
            accY = Math.atan2(mEast,mNorth)*rad2deg;    //atan2(East, North) - atan2(S.values[1],S.values[0])
            lastAccY += (accY-lastAccY)/magFilterConst;

            magData[0] = sensorEvent.values[1];
            magData[1] = sensorEvent.values[0];
            magData[2] = -sensorEvent.values[2];

            //vehicleState[2] = lastAccY;
            //estAngles[2] = lastAccY;
            //Log.i("Magnetometer","Magnetometer generated Yaw "+accY+" Filtered "+lastAccY);
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };

    double h;
    double M = -0.02896;
    double R = 8.3143;
    double g = 9.82;
    double barAltitude;
    double pressFilterConst = 8;
    SensorEventListener presListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            h = -Math.log(sensorEvent.values[0]/groundlevelPresure) * ((R*ambientTemperature)/(-M*g));
            barAltitude += (h-barAltitude)/pressFilterConst;
            //vehicleState[8] = barAltitude;
            Log.i("Pressure","mBar "+sensorEvent.values[0]+" height "+barAltitude);
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };

    SensorEventListener orientListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            //Uses taylt-brian angles "z,y,x"
            //euler[0] = sensorEvent.values[2];
            //euler[1] = sensorEvent.values[1];
            //euler[2] = sensorEvent.values[0];   //Sensor event 0 is yaw "azimut"
            //Log.i("EulerAngles","Angels "+euler[0]+" "+euler[1]+" "+euler[2]);
            vehicleState[0] = sensorEvent.values[2];
            vehicleState[1] = sensorEvent.values[1];
            vehicleState[2] = sensorEvent.values[0];
            estAngles[0] = vehicleState[0];
            estAngles[1] = vehicleState[1];
            estAngles[2] = vehicleState[2];
            Log.i("EulerAngles", "Angles "+estAngles[0]+" "+estAngles[1]+" "+estAngles[2]);
            sendDataToActivity(vehicleState);
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };

    volatile boolean startOnce = false;
    double time = 0;
    double lastTime = 0;
    double[] LastUTM = {0,0,0};
    private BroadcastReceiver GPSReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            Bundle b = intent.getExtras();
            UTMPos = b.getDoubleArray("UTMCoordinates");
            UTMPos[2] = UTMPos[2]-groundlevelGPS;
            time = System.currentTimeMillis();
            vel_N[0] = (UTMPos[0]-LastUTM[0])/((time-lastTime)/1000);
            vel_N[1] = (UTMPos[1]-LastUTM[1])/((time-lastTime)/1000);
            vel_N[2] = (UTMPos[2]-LastUTM[2])/((time-lastTime)/1000);
            lastTime = time;
            LastUTM = UTMPos;

            try {
                mutex.lock();
                Log.i("Lock","Lock");
                newGPSData = true;
            }finally {
                mutex.unlock();
                Log.i("Lock", "unlock");
            }
            Log.i("UTMTest", "N "+UTMPos[0]+" E "+UTMPos[1]+" H "+UTMPos[2]+" V "+vel_N[0]+" "+vel_N[1]+" "+vel_N[2]);

            if(!startOnce) {
                kalmanThread.start();
                startOnce = true;
            }
        }
    };

    /*double[] quaternion = new double[4];
    double[] lastEuler = new double[3];
    SensorEventListener rotListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            quaternion[0] = sensorEvent.values[0];
            quaternion[1] = sensorEvent.values[1];
            quaternion[2] = sensorEvent.values[2];
            quaternion[3] = sensorEvent.values[3];
            // roll (x-axis rotation)

            double sinr_cosp = 2 * (quaternion[3] * quaternion[0] + quaternion[1] * quaternion[2]);
            double cosr_cosp = 1 - 2 * (quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1]);
            euler[1] = Math.atan2(sinr_cosp, cosr_cosp) * rad2deg;

            // pitch (y-axis rotation)
            double sinp = 2 * (quaternion[3] * quaternion[1] - quaternion[2] * quaternion[0]);
            if (Math.abs(sinp) >= 1) {
                euler[0] = Math.copySign (Math.PI / 2, sinp); // use 90 degrees if out of range
            }else
            euler[0] = Math.asin(sinp) * rad2deg;

            // yaw (z-axis rotation)
            double siny_cosp = 2 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1]);
            double cosy_cosp = 1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]);
            euler[2] = Math.atan2(siny_cosp, cosy_cosp) * rad2deg;

            euler[0] = (euler[0] -lastEuler[0]) / 2.0;
            euler[1] = (euler[1] -lastEuler[1]) / 2.0;
            euler[2] = (euler[2] -lastEuler[2]) / 2.0;
            lastEuler[0] = euler[0];
            lastEuler[1] = euler[1];
            lastEuler[2] = euler[2];
            sendDataToActivity(euler);
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };*/

}

    /*double[][] R_BN(double R, double P, double Y){
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
    }*/


/*
/*
    //Kalman
    double[] F = new double[14];
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

    private Runnable KalmanFilter = new Runnable() {
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
            double[] testVector = new double[3];

            // Initialize state with values, position set to gps, velocity set to 0, angles set to acc and mag angles, angular velocity set to 0
            x[0] = 0;       x[1] = 0;       x[2] = 0;       x[3] = 0;       x[4] = 0;       x[5] = 0;       x[6] = 0;
            x[7] = 0;       x[8] = 0;       x[9] = accR;    x[10] = accP;   x[11] = accY;   x[12] = 0;      x[13] = 0;

            // Initialize Q - process noise matrix and R sensor noise matrix
            for (int i = 0; i < 14; i++) {
                Q[i][i] = 0.1;
                R[i][i] = 0.1;
                I[i][i] = 1;
                P[i][i] = 0.05;
            }
            P[11][11] = 0.01;

            while (true) {
                //Load input vector u
                U[0] = uDot; U[1] = vDot; U[2] = wDot; U[3] = p; U[4] = q; U[5] = r;   //Initialize input with ned values
                //Predict
                x = stateTransistionFunction(x, U, 0.005);    //Calculate the priori estimate
                J = jacobian(x, U, 0.005);      //Calculate the jacobian of F
                Jt = LA.matrixTranspose(J);         //matrixTranspose(J);    //Take transpose of the jacobian
                P = LA.matrixAdd(LA.matrixMultiply(LA.matrixMultiply(J,P),Jt),Q);   //Calculate priori covariance

                //Update 1
                y[0] = 0;       y[1] = 0;       y[2] = 0;       y[3] = 0;       y[4] = 0;       y[5] = 0;       y[6] = 0;
                y[7] = 0;       y[8] = 0;       y[9] = 0;       y[10] = 0;      y[11] = 0;      y[12] = 0;      y[13] = 0;
                C[0][0] = 0;    C[1][1] = 0;    C[2][2] = 0;    C[3][3] = 0;    C[4][4] = 0;    C[5][5] = 0;    C[6][6] = 0;
                C[7][7] = 0;    C[8][8] = 0;    C[9][9] = 1;    C[10][10] = 1;  C[11][11] = 1;  C[12][12] = 0;  C[13][13] = 0;

                //Intermediate steps for clarity
                Ct = LA.matrixTranspose(C);    //Take inverse of C
                CPR = LA.matrixInverse(LA.matrixAdd(LA.matrixMultiply(LA.matrixMultiply(C,P),Ct),R));  //Intermediate step for simplicity
                K = LA.matrixMultiply(LA.matrixMultiply(P,Ct), CPR);   //calculate kalman gain
                x = LA.vectAdd(x,LA.vectMatMultiply(K,LA.vectAdd(y,LA.vectMatMultiply(C,x),-1)),1);   //calculate posterior estimate
                P = LA.matrixMultiply(LA.matrixSubtract(I,LA.matrixMultiply(K,C)), P);    //Update posterior covariance

                xVectorToController[0] = x[0];          xVectorToController[1] = x[1];              xVectorToController[2] = x[2];
                xVectorToController[3] = x[3];          xVectorToController[4] = x[4];              xVectorToController[5] = x[5];
                xVectorToController[6] = x[6];          xVectorToController[7] = x[7];              xVectorToController[8] = x[8];
                xVectorToController[9] = x[9]*rad2deg;  xVectorToController[10] = x[10]*rad2deg;    xVectorToController[11] = x[11]*rad2deg;
                xVectorToController[12] = x[12];        xVectorToController[13] = x[13];
                xVectorToController[14] = U[3];         xVectorToController[15] = U[4];             xVectorToController[16] = U[5];

                //sendDataToActivity(xVectorToController);
                testVector[0] = x[9]*rad2deg;   testVector[1] = x[10]*rad2deg;  testVector[2] = x[2]*rad2deg;
                sendDataToActivity(testVector);

                //Loop thread every 5mS - 200Hz
                try {
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    };
    //End
*/


/*
double[][] RMat = new double[3][3];
        double[] acc_N = new double[3];
        double[] gpsdata = new double[6];

        double[][] A = {{1,0,0,dt,0,0},{0,1,0,0,dt,0},{0,0,1,0,0,dt},{0,0,0,1,0,0},{0,0,0,0,1,0},{0,0,0,0,0,1}};
        double[][] B = {{0.5*dt*dt, 0, 0},{0, 0.5*dt*dt, 0},{0, 0, 0.5*dt*dt},{dt, 0, 0},{0, dt, 0},{0, 0, dt}};
        double[] xm = new double[6];
        double[] xp = new double[6];
        double[][] K = new double[6][6];
        double[][] Pm = new double[6][6];
        double[][] Pp = {{10, 0,0,0,0,0},{0, 10, 0,0,0,0},{0,0, 10, 0,0,0},{0,0,0, 10, 0,0},{0,0,0,0, 10, 0},{0,0,0,0,0, 10}};
        double[][] Q = {{0.2, 0,0,0,0,0},{0, 0.2, 0,0,0,0,0},{0,0, 0.2, 0,0,0},{0,0,0, 0.2, 0,0},{0,0,0,0, 0.2, 0},{0,0,0,0,0, 0.2}};
        double[][] R = {{0.2, 0,0,0,0,0},{0, 0.2, 0,0,0,0,0},{0,0, 0.2, 0,0,0},{0,0,0, 0.2, 0,0},{0,0,0,0, 0.2, 0},{0,0,0,0,0, 0.2}};
        double[][] I = {{1,0,0,0,0,0},{0,1,0,0,0,0},{0,0,1,0,0,0},{0,0,0,1,0,0},{0,0,0,0,1,0},{0,0,0,0,0,1}};
        double[][] H = {{1,0,0,0,0,0},{0,1,0,0,0,0},{0,0,1,0,0,0},{0,0,0,1,0,0},{0,0,0,0,1,0},{0,0,0,0,0,1}};

        @Override
        public void run() {
                xp[0] = UTMPos[0];
                xp[1] = UTMPos[1];
                xp[2] = UTMPos[2];
                xp[3] = 0;
                xp[4] = 0;
                xp[5] = 0;

            while (!threadInterrupt) {
                //Predict step
                //Transform acceleration
                RMat = R_BN(estAngles[0], estAngles[1], estAngles[2]);    //Creates the DCM from body to ned
                acc_N = LA.vectMatMultiply(RMat, acc_B);

                //Calculate a priori estimate
                xm = LA.vectAdd(LA.vectMatMultiply(A, xp), LA.vectMatMultiply(B, acc_N), 1);
                Pm = LA.matrixAdd(LA.matrixMultiply(LA.matrixMultiply(A, Pp), LA.matrixTranspose(A)), Q);

                if (newGPSData) {
                    //Update step
                    gpsdata[0] = UTMPos[0]; gpsdata[1] = UTMPos[1]; gpsdata[2] = UTMPos[2];
                    gpsdata[3] = vel_N[0];  gpsdata[4] = vel_N[1];  gpsdata[5] = vel_N[2];

                    //Calculate Kalman gain
                    //K = LA.matrixMultiply(LA.matrixMultiply(Pm, LA.matrixTranspose(H)), LA.matrixInverse(LA.matrixAdd(LA.matrixMultiply(H, LA.matrixMultiply(Pm, LA.matrixTranspose(H))), R)));
                    K = LA.matrixMultiply(Pm,LA.matrixInverse(LA.matrixAdd(Pm, R)));
                    //Calculate a posteori estimate
                    xp = LA.vectAdd(xm, LA.vectMatMultiply(K, LA.vectAdd(gpsdata, xm, -1)), 1);
                    //Pp = LA.matrixMultiply(LA.matrixSubtract(I, LA.matrixMultiply(K, H)), Pm);
                    Pp = LA.matrixMultiply(LA.matrixSubtract(I,K), Pm);
                    try {
                        mutex.lock();
                        newGPSData = false;
                    } finally {
                        mutex.unlock();
                    }
                }
                Log.d("PM","Pm1 "+Pm[0][0]+" Pm2 "+Pm[1][1]+" Pm3 "+Pm[2][2]+" Pm4 "+Pm[3][3]+" Pm5 "+Pm[4][4]+" Pm6 "+Pm[5][5]);
                Log.d("PP","Pp1 "+Pp[0][0]+" Pp2 "+Pp[1][1]+" Pp3 "+Pp[2][2]+" Pp4 "+Pp[3][3]+" Pp5 "+Pp[4][4]+" Pp6 "+Pp[5][5]);
                //Log.d("KalmanGain","K1 "+K[0][0]+" K2 "+K[1][1]+" K3 "+K[2][2]+" K4 "+K[3][3]+" K5 "+K[4][4]+" K6 "+K[5][5]);
                Log.d("KalmanGain", "Gain "+K[0][0]+" "+K[0][1]+" "+K[0][2]+" "+K[0][3]+" "+K[0][4]+" "+K[0][5]+" "+K[1][0]+" "+K[1][1]+" "+K[1][2]+" "+K[1][3]+" "+K[1][4]+" "+K[1][5]+" "+K[2][0]+" "+K[2][1]+" "+K[2][2]+" "+K[2][3]+" "+K[2][4]+" "+K[2][5]+" "+K[3][0]+" "+K[3][1]+" "+K[3][2]+" "+K[3][3]+" "+K[3][4]+" "+K[3][5]+" "+K[4][0]+" "+K[4][1]+" "+K[4][2]+" "+K[4][3]+" "+K[4][4]+" "+K[4][5]+" "+K[5][0]+" "+K[5][1]+" "+K[5][2]+" "+K[5][3]+" "+K[5][4]+" "+K[5][5]);
                Log.i("XM", "xm1 " + xm[0] + " xm2 " + xm[1] + " xm3 " + xm[2] + " xm4 " + xm[3] + " xm5 " + xm[4] + " xm6 " + xm[5]);    //Before update
                Log.i("XP", "Pn " + xp[0] + " Pe " + xp[1] + " Ph " + xp[2] + " Vn " + xp[3] + " Ve " + xp[4]);   //After update

                //Loop thread every 5mS - 200Hz
                try {
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
 */


/*
    double[] lastAcc = new double[3];
    double[] thisAcc = new double[3];
    double[] lastVel = new double[3];
    double[] thisVel = new double[3];
    double[] thisPos = new double[3];
    double[] lastPos = new double[3];
    int count = 0;
    int count2 = 10;
    double currentTime;
    double t0;
    double deltaT;
    //double[][] rotationMat = new double[3][3];
    //double[] axis = new double[3];
    SensorEventListener accListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            //uDot = sensorEvent.values[1];
            //vDot = sensorEvent.values[0];
            //wDot = sensorEvent.values[2];
            //accR = Math.atan2(sensorEvent.values[0], sensorEvent.values[2]);
            //accP = Math.atan2(sensorEvent.values[1], Math.sqrt(Math.pow(sensorEvent.values[0],2)+(Math.pow(sensorEvent.values[2],2))));
            /*
            uDot = sensorEvent.values[1];
            vDot = sensorEvent.values[0];
            wDot = sensorEvent.values[2];
            lastAcc[0] = (uDot-lastAcc[0])/accFilterConst;
            lastAcc[1] = (vDot-lastAcc[1])/accFilterConst;
            lastAcc[2] = (wDot-lastAcc[2])/accFilterConst;
            if(lastAcc[0] < 0.1 && lastAcc[0] > -0.1) lastAcc[0] = 0;
            if(lastAcc[1] < 0.1 && lastAcc[1] > -0.1) lastAcc[1] = 0;
            if(lastAcc[2] < 0.2 && lastAcc[2] > -0.2) lastAcc[2] = 0;
            test2 = test2 + lastAcc[0];
            test = test + test2*0.005 + 0.5*(lastAcc[0])*0.005;   //This might actually work!!

            if(test < 0.00001 && test > -0.00001) test = 0;
             */
            /*
            thisAcc[0] = sensorEvent.values[1];
            thisAcc[1] = sensorEvent.values[0];
            thisAcc[2] = sensorEvent.values[2];

            if (thisAcc[0] < 0.1 && thisAcc[0] > -0.1) thisAcc[0] = 0;
            if (thisAcc[1] < 0.1 && thisAcc[1] > -0.1) thisAcc[1] = 0;
            if (thisAcc[2] < 0.1 && thisAcc[2] > -0.1) thisAcc[2] = 0;

            thisVel[0] = lastVel[0] + lastAcc[0] + (thisAcc[0] - lastAcc[0]) / 2;
            thisPos[0] = lastPos[0] + lastVel[0] + (thisVel[0] - lastVel[0]) / 2;

            lastAcc[0] = thisAcc[0];
            lastAcc[1] = thisAcc[1];
            lastAcc[2] = thisAcc[2];
            lastVel[0] = thisVel[0];
            lastVel[1] = thisVel[1];
            lastVel[2] = thisVel[2];
            lastPos[0] = thisPos[0];
            lastPos[1] = thisPos[1];
            lastPos[2] = thisPos[2];

            if (thisAcc[0] == 0) {
                count++;
            } else {
                count = 0;
            }
            if (count >= 25) {
                thisVel[0] = 0;
                lastVel[0] = 0;
                count = 0;
            }
            * /

            //thisAcc[0] = (-sensorEvent.values[1]-thisAcc[0])/2;
            thisAcc[0] = -sensorEvent.values[1];
            if(thisAcc[0] < 0.2 && thisAcc[0] > -0.2) thisAcc[0] = 0;
            thisVel[0] = lastVel[0] + thisAcc[0]*0.005;
            //if(thisVel[0] < 0.05 && thisVel[0] > -0.05) thisVel[0] = 0;
            thisPos[0] = lastPos[0] + thisVel[0] + 0.5*thisAcc[0]*0.005*0.005;
            lastVel[0] = thisVel[0];
            lastPos[0] = thisPos[0];

            Log.i("DeltaT","Dt"+deltaT);
            Log.i("X-axis data", "acc "+thisAcc[0]+" vel "+thisVel[0]+" pos "+thisPos[0]);
            //Log.i("Accelerometer","x "+lastAcc[0]+" pos "+thisPos[0]+" y "+lastAcc[1]+" z "+lastAcc[2]);
        }
        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };

*/