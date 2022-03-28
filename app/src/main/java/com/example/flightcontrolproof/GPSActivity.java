package com.example.flightcontrolproof;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.location.LocationRequest;
import android.os.Bundle;
import android.os.HandlerThread;
import android.os.Looper;
import android.util.Log;
import android.widget.TextView;

public class GPSActivity extends AppCompatActivity {
    TextView GPSview;
    TextView oldGPSView;
    TextView UTMView;
    TextView oldUTMView;
    LocationManager mLocationManager;
    HandlerThread GPSUpdateThread;
    Looper GPSLooper;

    double[] GPSLocation = new double[2];
    double[] oldGPSLocation = new double[2];
    double[] UTM;
    double[] oldUTM = new double[2];

    UTMConverterClass UTMConv;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_gpsactivity);
        GPSUpdateThread = new HandlerThread("GPS handler thread");
        GPSUpdateThread.start();
        GPSLooper = GPSUpdateThread.getLooper();

        GPSview = findViewById(R.id.GPSTextView);
        oldGPSView = findViewById(R.id.OldGPSTextView);
        UTMView = findViewById(R.id.UTMTextView);
        oldUTMView = findViewById(R.id.oldUTMTextView);

        UTMConv = new UTMConverterClass();
        mLocationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
    }

    @Override
    protected void onStart() {
        super.onStart();
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.ACCESS_FINE_LOCATION}, 1);
            return;
        }
        mLocationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, GPSListener, GPSLooper);
    }
    @Override
    protected void onStop(){
        super.onStop();
        mLocationManager.removeUpdates(GPSListener);
        GPSUpdateThread.quit();
    }

    LocationListener GPSListener = new LocationListener() {
        @Override
        public void onLocationChanged(@NonNull Location location) {
            GPSLocation[0] = location.getLatitude(); GPSLocation[1] = location.getLongitude();
            UTM = UTMConv.getUTM(GPSLocation[0],GPSLocation[1]);
            Log.i("GPS","GPS "+GPSLocation[0]+" "+GPSLocation[1]);
            Log.i("GPS","GPS "+oldGPSLocation[0]+" "+oldGPSLocation[1]);
            Log.i("GPS","UTM "+UTM[0]+" "+UTM[1]);
            Log.i("GPS","UTM "+oldUTM[0]+" "+oldUTM[1]);
            Log.i("GPSAccuracy","Accuracy "+location.getAccuracy());
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    GPSview.setText(GPSLocation[0]+" "+GPSLocation[1]);
                    oldGPSView.setText(oldGPSLocation[0]+" "+oldGPSLocation[1]);
                    UTMView.setText(UTM[0]+" "+UTM[1]);
                    oldUTMView.setText(oldUTM[0]+" "+oldUTM[1]);
                }
            });
            oldGPSLocation = GPSLocation;
            oldUTM = UTM;
        }
        @Override
        public void onStatusChanged(String provider, int status, Bundle extras){
            //Do nothing for now -- May be called on older android versions but not on version Q and above
        }
    };
}