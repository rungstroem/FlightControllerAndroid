package com.example.flightcontrolproof;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;


public class Controller {
    // Variables
    Context mContext;
    SensorManager mSensorManager;
    HandlerThread sensorCallbackThread;
    Thread sensorDataThread;

    LocationManager mGPSManager;
    HandlerThread gpsCallbackThread;
    Thread gpsThread;

    //Constructor
    public Controller(Context mContext) {
        this.mContext = mContext;
    }

    private SensorEventListener accListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            accX = sensorEvent.values[0];
            accY = sensorEvent.values[1];
            accZ = sensorEvent.values[2];
            T = sensorEvent.timestamp;
            //Log.i("Listener", "Hello from AccListener");
        }
        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {
            //Do nothing for now
        }
    };
    private SensorEventListener gyrListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            gyrR = sensorEvent.values[0];
            gyrP = sensorEvent.values[1];
            gyrY = sensorEvent.values[2];
            //Log.i("Listener", "Hello from GyroListener");
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {
            //Do nothing for now
        }
    };
    private LocationListener gpsListener = new LocationListener() {
        @Override
        public void onLocationChanged(@NonNull Location location) {
            lon = location.getLongitude();
            lat = location.getLatitude();
            alt = location.getAltitude();
            //Log.i("Listener", "Hello from GPSListener");
        }
    };

    int RATE = 400; //sample rate in µS
    int gpsRATE = 200000;
    long T;
    float accX;
    float accY;
    float accZ;
    float gyrR;
    float gyrP;
    float gyrY;
    double lon;
    double lat;
    double alt;

    public void setup() {
        this.initSensors();
        //this.initGPS();
        //gpsThread.start();
        sensorDataThread.start();
    }
    public void destroy() {
        //Clean-up function
        mSensorManager.unregisterListener(accListener);
        mSensorManager.unregisterListener(gyrListener);
        this.sensorCallbackThread.quit();
        this.sensorDataThread.interrupt();
        this.gpsCallbackThread.quit();
        this.gpsThread.interrupt();
    }
    private void initSensors() {
        mSensorManager = (SensorManager) mContext.getSystemService(Context.SENSOR_SERVICE);
        Sensor accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Sensor gyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        sensorCallbackThread = new HandlerThread("Sensor callback thread");
        sensorCallbackThread.start();
        Looper sensorThreadLooper = sensorCallbackThread.getLooper();
        Handler sensorThreadHandler = new Handler(sensorThreadLooper);
        mSensorManager.registerListener(accListener, accelerometer, RATE, sensorThreadHandler);  //Use "SensorManager.SENSOR_DELAY_FASTEST" instead of "RATE" to get maximum speed.
        mSensorManager.registerListener(gyrListener, gyroscope, RATE, sensorThreadHandler);      //Use RATE for user-defined update rate - set in µS
        //Thread to do something with the sensor data...
        sensorDataThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!sensorDataThread.isInterrupted()) {
                    Log.i("GyroData", "R " + gyrR + " P " + gyrP + " Y " + gyrY);
                    Log.i("AccData", "x " + accX + " y " + accY + " z " + accZ + " Time : "+T);
                }
            }
        });
    }

    private void initGPS() {
        mGPSManager = (LocationManager) mContext.getSystemService(mContext.LOCATION_SERVICE);
        gpsCallbackThread = new HandlerThread("GPS callback thread");
        gpsCallbackThread.start();
        Looper gpsThreadLooper = gpsCallbackThread.getLooper();
        Handler gpsThreadHandler = new Handler(gpsThreadLooper);
        if (ActivityCompat.checkSelfPermission(mContext, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(mContext, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            return;
        }
        mGPSManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, gpsRATE, 0, gpsListener, gpsThreadLooper);
        gpsThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(!gpsThread.isInterrupted()) {
                    Log.i("GPSData", "Lat " + lat + " Lon " + lon + " Alt " + alt);
                }
            }
        });
    }
}



    //This is the code that was originally use to have a thread handle the sensor callbacks.
    //whether this code works is not known currently known
        /*new Thread(new Runnable(){
           @Override
           public void run(){
               //Looper.prepare();
               Handler sensorThreadHandler = new Handler();
               mSensorManager.registerListener(accListener, accelerometer, RATE, sensorThreadHandler);
               mSensorManager.registerListener(gyrListener, gyroscope, RATE, sensorThreadHandler);
               while(true){
                   Log.i("GyroData", "R "+gyrR+" P "+gyrP+" Y "+gyrY);
                   Log.i("AccData", "x "+accX+" y "+accY+" z "+accZ);
               }
               //Looper.loop();
           }
        }).start();*/
