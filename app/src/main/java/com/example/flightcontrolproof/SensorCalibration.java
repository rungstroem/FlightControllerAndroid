package com.example.flightcontrolproof;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import android.Manifest;
import android.annotation.SuppressLint;
import android.content.Context;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

public class SensorCalibration extends AppCompatActivity implements View.OnClickListener {
    SensorManager sManager;
    HandlerThread sHandlerThread;
    Handler sThreadHandler;
    Looper sThreadLooper;

    Sensor mPressureSensor;
    double pressureCalibration;
    double pressureSamples = 0;

    Sensor mAccelerometerSensor;
    double[] accelerometerCalibration = new double[3];
    double accelerometerSamples = 0;

    Sensor mTemperatureSensor;
    double temperature;
    double tempSamples = 0;

    LocationManager mLocationManager;
    double GPSCalibration;
    double GPSSamples = 0;

    SharedPreferences mSharedPref;

    Button mPressureCalibrationButton;
    Button mAccelerometerCalibrationButton;
    Button mGPSCalibrationButton;

    TextView GPSView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_sensor_calibration);

        //Sensor thread create
        sHandlerThread = new HandlerThread("Calibration thread");

        //Sensor create
        sManager = (SensorManager) this.getSystemService(Context.SENSOR_SERVICE);
        mPressureSensor = sManager.getDefaultSensor(Sensor.TYPE_PRESSURE);
        mAccelerometerSensor = sManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        mTemperatureSensor = sManager.getDefaultSensor(Sensor.TYPE_AMBIENT_TEMPERATURE);

        mLocationManager = (LocationManager) this.getSystemService(Context.LOCATION_SERVICE);

        //Start temperature sensor immediately
        sManager.registerListener(temperatureListener, mTemperatureSensor, 100000);  //10Hz sampling

        //Init buttons
        mPressureCalibrationButton = findViewById(R.id.pressureCalibrationButton);
        mPressureCalibrationButton.setOnClickListener(this);
        mAccelerometerCalibrationButton = findViewById(R.id.accelerometerCalibrationButton);
        mAccelerometerCalibrationButton.setOnClickListener(this);
        mGPSCalibrationButton = findViewById(R.id.GPScalibrationButton);
        mGPSCalibrationButton.setOnClickListener(this);
        GPSView = findViewById(R.id.GPSData);
        mSharedPref = this.getSharedPreferences("CalibrationData", this.MODE_PRIVATE);  //Set share name and mode

        //Sensor thread handle and looper
        sHandlerThread.start();
        sThreadLooper = sHandlerThread.getLooper();
        sThreadHandler = new Handler(sThreadLooper);

        Log.i("SystemState", "Calibration started");
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        Log.i("SystemState", "Calibration stopped");
        sThreadLooper.quit();
        sHandlerThread.quit();
    }

    private SensorEventListener pressListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            pressureCalibration += sensorEvent.values[0];
            pressureSamples++;
            Log.i("Calibration","BarCal "+pressureCalibration);

            if (pressureSamples > 100) {
                sManager.unregisterListener(pressListener);
                pressureCalibration = pressureCalibration / pressureSamples;

                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(getApplicationContext(), "Barometer calibration: " + pressureCalibration, Toast.LENGTH_LONG).show();
                    }
                });

                SharedPreferences.Editor editor = mSharedPref.edit();
                editor.putLong("BarometerCalibration", Double.doubleToRawLongBits(pressureCalibration)).apply();
            }
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };

    private SensorEventListener accelerometerListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            accelerometerCalibration[0] = sensorEvent.values[0];
            accelerometerCalibration[1] = sensorEvent.values[1];
            accelerometerCalibration[2] = sensorEvent.values[2];
            accelerometerSamples++;

            if (accelerometerSamples > 1000) {
                sManager.unregisterListener(accelerometerListener);

                accelerometerCalibration[0] = accelerometerCalibration[0] / accelerometerSamples;
                accelerometerCalibration[1] = accelerometerCalibration[1] / accelerometerSamples;
                accelerometerCalibration[2] = accelerometerCalibration[2] / accelerometerSamples;

                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(getApplicationContext(), "Accelerometer calibration: " + accelerometerCalibration[0] + " " + accelerometerCalibration[1] + " " + accelerometerCalibration[2], Toast.LENGTH_LONG).show();
                    }
                });

                SharedPreferences.Editor editor = mSharedPref.edit();
                editor.putLong("AccelerometerCalibrationX", Double.doubleToRawLongBits(accelerometerCalibration[1]));
                editor.putLong("AccelerometerCalibrationY", Double.doubleToRawLongBits(accelerometerCalibration[0]));
                editor.putLong("AccelerometerCalibrationZ", Double.doubleToRawLongBits(accelerometerCalibration[2]));
                editor.apply();
            }
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };

    private SensorEventListener temperatureListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            temperature += sensorEvent.values[0];
            tempSamples++;
            if (tempSamples > 10) {
                sManager.unregisterListener(temperatureListener);
                temperature = temperature / tempSamples;

                SharedPreferences.Editor editor = mSharedPref.edit();
                editor.putLong("AmbientTemperature", Double.doubleToRawLongBits(temperature)).apply();
            }
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };

private LocationListener GPSlistener = new LocationListener() {
    @Override
    public void onLocationChanged(Location location) {
        Log.i("Calibration", "GPSCal " + location.getAltitude());
        GPSView.setText("Altitude "+location.getAltitude());
        if(location.getAltitude() == 0){

        }else {
            GPSCalibration += location.getAltitude();
            GPSSamples++;

            if (GPSSamples > 20) {
                GPSCalibration = GPSCalibration / GPSSamples;
                mLocationManager.removeUpdates(GPSlistener);

                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(getApplicationContext(), "GPS calibration " + GPSCalibration, Toast.LENGTH_LONG).show();
                    }
                });

                SharedPreferences.Editor editor = mSharedPref.edit();
                editor.putLong("GPSAltitudeCalibration", Double.doubleToRawLongBits(GPSCalibration)).apply();
            }
        }
    }
    @Override
    public void onStatusChanged(String provider, int status, Bundle extras){
        //Do nothing for now -- May be called on older android versions but not on version Q and above
    }
};

    @SuppressLint("MissingPermission")
    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.pressureCalibrationButton:
                Toast.makeText(getApplicationContext(), "Started barometer calibration", Toast.LENGTH_LONG).show();
                pressureSamples = 0;
                pressureCalibration = 0;
                sManager.registerListener(pressListener, mPressureSensor, 5000, sThreadHandler);
                break;

            case R.id.accelerometerCalibrationButton:
                Toast.makeText(getApplicationContext(), "Started accelerometer calibration", Toast.LENGTH_LONG).show();
                accelerometerCalibration[0] = 0;
                accelerometerCalibration[1] = 0;
                accelerometerCalibration[2] = 0;
                accelerometerSamples = 0;
                sManager.registerListener(accelerometerListener, mAccelerometerSensor, 5000, sThreadHandler);
                break;
            case R.id.GPScalibrationButton:
                Toast.makeText(getApplicationContext(), "Started GPS height calibration", Toast.LENGTH_LONG).show();
                GPSCalibration = 0;
                GPSSamples = 0;
                mLocationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, GPSlistener);
                break;

        }
    }
}