package com.example.flightcontrolproof;

import androidx.appcompat.app.AppCompatActivity;

import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.widget.TextView;

import java.util.List;

public class ListSensors extends AppCompatActivity {
    SensorManager mManager;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_list_sensors);
        mManager = (SensorManager) getSystemService(SENSOR_SERVICE);
    }
    @Override
    protected void onStart(){
        List<Sensor> SensorDevices = mManager.getSensorList(Sensor.TYPE_ALL);
        TextView mTextview = findViewById(R.id.SensorsTV);
        for(Sensor S : SensorDevices){
            mTextview.append(S.toString() + "\n\n");
        }
        super.onStart();
    }
}