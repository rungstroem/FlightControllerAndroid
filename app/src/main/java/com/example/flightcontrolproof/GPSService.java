package com.example.flightcontrolproof;

import android.Manifest;
import android.annotation.SuppressLint;
import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.nfc.Tag;
import android.os.Binder;
import android.os.Bundle;
import android.os.HandlerThread;
import android.os.IBinder;
import android.os.IInterface;
import android.os.Looper;
import android.os.Parcel;
import android.os.RemoteException;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.core.content.ContextCompat;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import java.io.FileDescriptor;
import java.util.List;

public class GPSService extends Service {
    LocationManager mLocationManager;
    HandlerThread GPSHandlerThread;
    Looper GPSLooper;

    double[] UTMPosition = new double[3];
    double[] UTMTemp = new double[2];
    UTMConverterClass UTMconv;

    public GPSService() {
    }
    @Override
    public IBinder onBind(Intent intent) {
        throw new UnsupportedOperationException("Not yet implemented");
    }
    @Override
    public void onCreate(){
        GPSHandlerThread = new HandlerThread("GPS handler thread");
        UTMconv = new UTMConverterClass();
    }
    @SuppressLint("MissingPermission")
    @Override
    public int onStartCommand(Intent intent, int flags, int startID){
        GPSHandlerThread.start();
        GPSLooper = GPSHandlerThread.getLooper();
        mLocationManager = (LocationManager) this.getSystemService(Context.LOCATION_SERVICE);
        mLocationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER,0,0,GPSListener, GPSLooper);

        Log.i("SystemState","GPS service started");
        return START_STICKY;
    }

    public void sendDataToActivity(Location location){
        Intent intent = new Intent("GPSLocationUpdates");
        Bundle b = new Bundle();
        b.putParcelable("Location", location);
        intent.putExtra("Location",b);
        LocalBroadcastManager.getInstance(this).sendBroadcast(intent);
    }

    public void sendUTMtoActivity(Location location){
        UTMTemp = UTMconv.getUTM(location.getLatitude(),location.getLongitude());
        UTMPosition[0] = UTMTemp[0]; UTMPosition[1] = UTMTemp[1]; UTMPosition[2] = location.getAltitude();
        Intent intent = new Intent("GPSLocationUpdate");
        intent.putExtra("UTMCoordinates", UTMPosition);
    }

    public final LocationListener GPSListener = new LocationListener() {
        @Override
        public void onLocationChanged(@NonNull Location location) {
            Log.i("GPSUpdateService","Lat "+location.getLatitude()+" Lon "+location.getLongitude()+" Alt "+location.getAltitude());
            //sendDataToActivity(location);
            sendUTMtoActivity(location);
        }
    };

    public void onDestroy(){
        mLocationManager.removeUpdates(GPSListener);
        GPSLooper.quit();
        GPSHandlerThread.quit();
        Log.i("SystemState","GPS service stopped");
    }
}