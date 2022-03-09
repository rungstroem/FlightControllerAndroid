package com.example.flightcontrolproof;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;


import com.google.android.gms.common.ConnectionResult;
import com.google.android.gms.common.GoogleApiAvailability;

import java.util.ArrayList;

public class MainActivity extends AppCompatActivity implements View.OnClickListener {

    private Button mButton1;
    private Button mButton2;
    private Button mButton3;
    private Button mButton4;
    private Button mButton5;
    private Button mButton6;
    private TextView mTextView;
    Intent intent;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        //Get location consent
        getGPSConsent();

        //Start GPS updata service - Can't be started here until the Maps activity can accept updates from the service.
        //intent = new Intent(this, GPSService.class);
        //startService(intent);

        //Set layout and buttons
        set_layout();
        setContentView(R.layout.activity_main);
        mTextView = findViewById(R.id.heading);
        set_button_layout();
        mButton1.setOnClickListener(this);
        mButton2.setOnClickListener(this);
        mButton3.setOnClickListener(this);
        mButton4.setOnClickListener(this);
        mButton5.setOnClickListener(this);
        mButton6.setOnClickListener(this);
    }
    private void getGPSConsent(){
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.ACCESS_FINE_LOCATION}, 1);
            return;
        }
    }
    protected void set_layout(){
        this.requestWindowFeature(Window.FEATURE_NO_TITLE);
        getSupportActionBar().hide(); //Shows the app actionbar "name"
        this.getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,WindowManager.LayoutParams.FLAG_FULLSCREEN);

    }
    protected void set_button_layout(){
        ArrayList<Integer> size = new ArrayList<Integer>();
        int buttonSize = 400;
        mButton1 = (Button) findViewById(R.id.button1);
        mButton2 = (Button) findViewById(R.id.button2);
        mButton3 = (Button) findViewById(R.id.button3);
        mButton4 = (Button) findViewById(R.id.button4);
        mButton5 = (Button) findViewById(R.id.button5);
        mButton6 = (Button) findViewById(R.id.button6);
        mButton1.setWidth(buttonSize);
        mButton2.setWidth(buttonSize);
        mButton3.setWidth(buttonSize);
        mButton4.setWidth(buttonSize);
        mButton5.setWidth(buttonSize);
        mButton6.setWidth(buttonSize);
    }
    @Override
    public void onClick(View v){
        switch(v.getId()){
            case R.id.button1:
                Intent intent1 = new Intent(this, MapsActivity.class);
                //myIntent.putExtra("key", value); // - Transfer data to new activity
                startActivity(intent1);
                break;
            case R.id.button2:
                //Toast toast2 = Toast.makeText(getApplicationContext(), "You pushed the How To button. Page is under construction", Toast.LENGTH_LONG);
                //toast2.show();
                Intent intent2 = new Intent(this, MotorTestActivity.class);
                startActivity(intent2);
                break;
            case R.id.button3:
                //Toast toast3 = Toast.makeText(getApplicationContext(), "You pushed the Guides button. Page is under construction", Toast.LENGTH_LONG);
                //toast3.show();
                Intent intent3 = new Intent(this, AutopilotPID.class);
                startActivity(intent3);
                break;
            case R.id.button4:
                //Toast toast4 = Toast.makeText(getApplicationContext(), "You pushed the Calculator button. Page is under construction", Toast.LENGTH_LONG);
                //toast4.show();
                Intent intent4 = new Intent(this, SensorCalibration.class);
                startActivity(intent4);
                break;
            case R.id.button5:
                Intent intent5 = new Intent(this, USBDevices.class);
                startActivity(intent5);
                break;
            case R.id.button6:
                Intent intent6 = new Intent(this, ListSensors.class);
                startActivity(intent6);
            default:
                break;
        }
    }
}