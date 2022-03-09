package com.example.flightcontrolproof;

import androidx.appcompat.app.AppCompatActivity;

import android.content.SharedPreferences;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

public class SensorCalibration extends AppCompatActivity implements View.OnClickListener {
    SensorManager Smanager;
    HandlerThread sHandlerThread;
    Handler sThreadHandler;
    Looper sThreadLooper;

    Sensor pressure;
    double pressureCalibration = 0.0;
    double pressureSamples = 0;
    SharedPreferences mSharedPref;
    SharedPreferences.Editor editor;

    Button pressureCalibrationButton;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_sensor_calibration);
        pressureCalibrationButton = findViewById(R.id.pressureCalibrationButton);
        pressureCalibrationButton.setOnClickListener(this);

        //Sensor thread create
        sHandlerThread = new HandlerThread("Calibrateion thread");
    }

    @Override
    protected void onStart() {
        super.onStart();
        //Sensor thread handle and looper
        sHandlerThread.start();
        sThreadLooper = sHandlerThread.getLooper();
        sThreadHandler = new Handler(sThreadLooper);

        pressure = Smanager.getDefaultSensor(Sensor.TYPE_PRESSURE);
        mSharedPref = this.getPreferences(this.MODE_PRIVATE);

    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        Smanager.unregisterListener(pressListener);
        sThreadLooper.quit();
        sHandlerThread.quit();
    }

    private SensorEventListener pressListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            if(pressureSamples < 1000){
                pressureCalibration += sensorEvent.values[0];
                pressureSamples++;
            }
            if(pressureSamples == 1000){
                pressureCalibration = pressureCalibration/pressureSamples;  //Mean of 1000 samples
                //editor = mSharedPref.edit();
                //editor.putLong("PressureCalibration", Double.doubleToRawLongBits(pressureCalibration));
                //editor.apply();
                Toast.makeText(getApplicationContext(), "Pressure sensor calibrated",Toast.LENGTH_SHORT);
                pressureSamples++;
            }
            else{
                //Waste cycles
            }
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };

    @Override
    public void onClick(View view) {
        switch (view.getId()){
            case R.id.pressureCalibrationButton:
                Smanager.registerListener(pressListener,pressure,5000, sThreadHandler);

        }
    }
}