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
import android.widget.TextView;

public class GPSActivity extends AppCompatActivity {
    TextView mTextview;
    LocationManager mLocationManager;
    HandlerThread GPSUpdateThread;
    Looper GPSLooper;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_gpsactivity);
        GPSUpdateThread = new HandlerThread("GPS handler thread");
        GPSUpdateThread.start();
        GPSLooper = GPSUpdateThread.getLooper();
        mTextview = findViewById(R.id.GPSTextView);
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
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    mTextview.setText("Lat "+location.getLatitude()+" Lon "+location.getLongitude()+" Alt "+location.getAltitude());
                }
            });
        }
    };
}