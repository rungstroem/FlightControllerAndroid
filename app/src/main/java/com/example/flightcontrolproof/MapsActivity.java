package com.example.flightcontrolproof;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.fragment.app.FragmentActivity;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import android.Manifest;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.ActionMode;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.Toast;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.example.flightcontrolproof.databinding.ActivityMapsBinding;
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;

import java.util.ArrayList;
import java.util.Locale;

public class MapsActivity extends FragmentActivity implements OnMapReadyCallback {
    private GoogleMap mMap;
    private ActivityMapsBinding binding;
    private int waypointNR = 0;
    LocationManager mLocationManager;
    double curLat;
    double curLon;
    double curAlt;
    float zoomLevel = 17.0f;
    int lineThickness = 3;
    ArrayList<LatLng> waypoints;
    Button mButton1;
    Button mButton2;

    SharedPreferences mSharedPref;
    UTMConverterClass UTMConv;
    double[] UTM = new double[2];

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        //mSharedPref = this.getSharedPreferences("WaypointList",this.MODE_PRIVATE);

        //Create waypoint list for waypoint navigation
        waypoints = new ArrayList<LatLng>();
        getLocation();

        //Initialize UTM converter class
        UTMConv = new UTMConverterClass();

        //Shared preference
        mSharedPref = this.getSharedPreferences("WaypointList", this.MODE_PRIVATE);

        set_layout();
        binding = ActivityMapsBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        mButton1 = findViewById(R.id.RouteFinished);
        mButton2 = findViewById(R.id.DeleteLastPoint);

        SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager().findFragmentById(R.id.map);
        mapFragment.getMapAsync(this);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
    }

    protected void set_layout() {
        this.requestWindowFeature(Window.FEATURE_NO_TITLE);
        this.getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
    }

    void getLocation() {
        mLocationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.ACCESS_FINE_LOCATION}, 1);
            return;
        }
        mLocationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, new LocationListener() {
            @Override
            public void onLocationChanged(@NonNull Location location) {
                curLat = location.getLatitude();
                curLon = location.getLongitude();
                curAlt = location.getAltitude();
            }
            @Override
            public void onStatusChanged(String provider, int status, Bundle extras){
                //Do nothing for now -- May be called on older android versions but not on version Q and above
            }
        });
        Location location = mLocationManager.getLastKnownLocation(LocationManager.GPS_PROVIDER);
        curLat = location.getLatitude();
        curLon = location.getLongitude();
        curAlt = location.getAltitude();
    }


    /**
     * Manipulates the map once available.
     * This callback is triggered when the map is ready to be used.
     * This is where we can add markers or lines, add listeners or move the camera. In this case,
     */

    @Override
    public void onMapReady(GoogleMap googleMap) {
        LatLng initLatLon = new LatLng(curLat, curLon);
        waypoints.add(initLatLon);
        mMap = googleMap;
        mMap.animateCamera(CameraUpdateFactory.newLatLngZoom(new LatLng(initLatLon.latitude, initLatLon.longitude), zoomLevel));
        if (mMap != null) {
            //Add current location marker to map
            mMap.addMarker(new MarkerOptions().position(initLatLon).title("Current location"+" Lat: "+initLatLon.latitude+" Lon: "+initLatLon.longitude));
            //Create new markers on map
            mMap.setOnMapClickListener(new GoogleMap.OnMapClickListener() {
                @Override
                public void onMapClick(LatLng latLng) {
                    waypointNR++;
                    UTM = UTMConv.getUTM(latLng.latitude,latLng.longitude);
                    //mMap.addMarker(new MarkerOptions().position(latLng).title("Waypoint "+waypointNR+" Lat: "+latLng.latitude+" Lon: "+ latLng.longitude));
                    mMap.addMarker(new MarkerOptions().position(latLng).title("Waypoint "+waypointNR+" Lat: "+UTM[0]+" Lon: "+ UTM[1]));
                    mMap.addPolyline(new PolylineOptions().add(waypoints.get(waypointNR-1), latLng).width(lineThickness));
                    waypoints.add(latLng);
                }
            });
        }

        mButton1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                SharedPreferences.Editor editor = mSharedPref.edit();
                editor.clear(); //Clears old data
                editor.commit();

                double[] UTMTemp = new double[2];
                // !!! You ended here kenneth !!!
                for(int i=0; i<waypoints.size();i++){   //i=1 to ignore initLatLon at i=0
                    UTMTemp = UTMConv.getUTM(waypoints.get(i).latitude,waypoints.get(i).longitude);
                    editor.putLong("UTMLatitude"+i,Double.doubleToRawLongBits(UTMTemp[0]));
                    editor.putLong("UTMLongitude"+i, Double.doubleToRawLongBits(UTMTemp[1]));
                }
                editor.apply();
                Toast.makeText(getApplicationContext(), "Waypoint list saved", Toast.LENGTH_LONG).show();


                //Intent intent1 = new Intent(MapsActivity.this, controlActivity.class);
                //intent1.putExtra("waypointList", waypoints); // - Transfer data to new activity
                //startActivity(intent1);
            }
        });
        mButton2.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                waypointNR=0;
                waypoints.clear();
                mMap.clear();
                mMap.addMarker(new MarkerOptions().position(initLatLon).title("Current location"+" Lat: "+initLatLon.latitude+" Lon: "+initLatLon.longitude));
                waypoints.add(initLatLon);
            }
        });
    }
}