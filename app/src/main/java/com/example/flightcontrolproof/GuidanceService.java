package com.example.flightcontrolproof;

import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.IBinder;
import android.util.Log;

import androidx.localbroadcastmanager.content.LocalBroadcastManager;

public class GuidanceService extends Service {
    //Usefull variables
    double rad2deg = 180.0/Math.PI;
    volatile boolean threadInterrupt = false;

    //Altitude guidance
    Thread altitudeThread;
    PIDController altitudeGuidanceController;
    double[] altitudeGuidanceGains = {2.0, 0, 0};
    double altitudeSP = 3; //Just a fixed 3m altitude

    //Heading guidance
    Thread guidanceThread;
    double[][] waypointList;
    double rAcceptance = 50; //5m radius of acceptance

    //Vehicle state from estimator service
    double[] vehicleState = new double[10];  //[Roll Pitch Yaw p q r Px Py Pz Vt]

    public GuidanceService() {
    }

    @Override
    public IBinder onBind(Intent intent) {
        // TODO: Return the communication channel to the service.
        throw new UnsupportedOperationException("Not yet implemented");
    }

    @Override
    public void onCreate() {
        super.onCreate();
        //Register receiver for Kalman data
        LocalBroadcastManager.getInstance(this).registerReceiver(vehicleStateReceiver, new IntentFilter("attitudeUpdate"));
        //Register receiver for altitude controller gain update
        LocalBroadcastManager.getInstance(this).registerReceiver(altitudeGuidanceGainUpdate, new IntentFilter("AltitudeGuidanceGainUpdate"));

        //Shared preference - waypoints
        SharedPreferences mSharedPref = getSharedPreferences("WaypointList", Context.MODE_PRIVATE);
        waypointList = getPreference(mSharedPref);

        //Altitude controller
        altitudeGuidanceController = new PIDController(altitudeGuidanceGains[0], altitudeGuidanceGains[1], altitudeGuidanceGains[2]);
        altitudeGuidanceController.setSaturation(45,-45);   //This is angle output so set saturation to maximum pitch angle

        //Guidance threads
        guidanceThread = new Thread(guidanceCC);
        guidanceThread.start();
        altitudeThread = new Thread(guidanceAltitude);
        altitudeThread.start();
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        return START_STICKY;
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        threadInterrupt = true;
        Log.i("SystemState","Guidance service stopped");
    }

    double[][] getPreference(final SharedPreferences prefs){
        double[][] waypointList = new double[10][2];
        int iterator;
        for(int i = 0; i<10; i++){
            iterator = i+1;
            if(prefs.contains("UTMLatitude"+iterator)){
                waypointList[i][0] = Double.longBitsToDouble(prefs.getLong("UTMLatitude"+iterator,0));
                waypointList[i][1] = Double.longBitsToDouble(prefs.getLong("UTMLongitude"+iterator,0));
            }else{
                break;
            }
        }
        return waypointList;
    }

    double steadyZone(double setpoint, double u, double deadZone){
        return ( (setpoint-deadZone < u) ? ( (u < setpoint+deadZone) ? setpoint : u) : u);
    }

    BroadcastReceiver vehicleStateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            Bundle b = intent.getExtras();
            vehicleState = b.getDoubleArray("stateVector");

        }
    };

    BroadcastReceiver altitudeGuidanceGainUpdate = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            Bundle b = intent.getExtras();
            altitudeGuidanceGains = b.getDoubleArray("data");
            altitudeGuidanceController.setGains(altitudeGuidanceGains[0], altitudeGuidanceGains[1], altitudeGuidanceGains[2]);
        }
    };

    public void sendDataToActivity(double data, String intentIdentifier){
        Intent intent = new Intent(intentIdentifier);
        intent.putExtra("data", data);
        LocalBroadcastManager.getInstance(this).sendBroadcast(intent);
    }

    private Runnable guidanceAltitude = new Runnable() {
        double pitchCommand;
        @Override
        public void run() {
            Log.i("SystemState", "Altitude guidance thread started");
            while(!threadInterrupt){
                pitchCommand = -altitudeGuidanceController.control(altitudeSP,steadyZone(altitudeSP,vehicleState[8],1),0.2);

                Log.i("Guidance","Altitude "+vehicleState[8]+" setpoint "+pitchCommand+" Pitch "+vehicleState[1]);
                sendDataToActivity(pitchCommand, "AltitudeGuidanceUpdate");

                //Loop the thread - 5Hz
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            Log.i("SystemState","Altitude guidance thread shutting down");
        }
    };

    private Runnable guidanceCC = new Runnable() {
        double UAVx, UAVy, UAVyaw;
        double LookAhead = 5;   //5m look-ahead
        double pLOS;
        double crossTrackErrorAngle;
        double heading;
        int waypointPointer = 0;

        double RollCommand;
        double LOSangle;
        double UAVangle;
        double distance;
        double missDistance;
        double k = 0.1;

        @Override
        public void run() {
            Log.i("SystemState","Guidance thread started");

            while(!threadInterrupt){
                UAVx = vehicleState[6]; UAVy = vehicleState[7];

                if(vehicleState[2] < 180){
                    UAVyaw = vehicleState[2];
                }else{
                    UAVyaw = -(360-vehicleState[2]);
                }

                LOSangle = Math.atan2(waypointList[waypointPointer+1][1]-waypointList[waypointPointer][1],waypointList[waypointPointer+1][0]-waypointList[waypointPointer][0])*rad2deg;
                UAVangle = Math.atan2(UAVy-waypointList[waypointPointer][1],UAVx-waypointList[waypointPointer][0])*rad2deg;
                distance = Math.sqrt(Math.pow(UAVx-waypointList[waypointPointer][0],2)+Math.pow(UAVy-waypointList[waypointPointer][1],2));  //Distance between UAV and current waypoint W_i
                pLOS = Math.sqrt(Math.pow(LookAhead,2)+Math.pow(distance*Math.sin(LOSangle-UAVangle),2));
                crossTrackErrorAngle = Math.asin(distance*Math.sin(LOSangle-UAVangle)/pLOS)*rad2deg;
                heading = crossTrackErrorAngle+LOSangle;
                missDistance = Math.sqrt(Math.pow(waypointList[waypointPointer+1][0]-UAVx,2)+Math.pow(waypointList[waypointPointer+1][1]-UAVy,2));
                RollCommand = -k*(heading-UAVyaw);  //Can also include air speed Va - setPoint=k*(heading-UAVyaw)*Va

                //Change to next waypoint
                if(missDistance < rAcceptance){
                    waypointPointer++;
                    if(waypointList.length -1 < waypointPointer) waypointPointer = waypointList.length-1;
                }

                sendDataToActivity(RollCommand, "GuidanceUpdate");
                Log.i("Guidance","LOS angle "+LOSangle+" Roll setpoint "+RollCommand+" Yaw "+UAVyaw+" missDistance "+missDistance+" waypoint "+waypointList[waypointPointer+1][0]+" "+waypointList[waypointPointer+1][1]+" WaypointNR "+waypointPointer+" CurrentLocation "+UAVx+" "+UAVy);

                //Loop the thread - 5Hz
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            Log.i("SystemState","Guidance thread shutting down");
        }
    };

    private final Runnable guidanceLOS = new Runnable() {
        double k1 = 0.1, k2 = 0.1;
        double UAVx, UAVy, UAVyaw;
        double crossTrackError;
        int waypointPointer = 0;

        double RollCommand;
        double LOSangle;
        double UAVangle;
        double distance;
        double missDistance;

        @Override
        public void run() {
            Log.i("SystemState", "Guidance thread started");
            while (!threadInterrupt) {
                UAVx = vehicleState[6]; UAVy = vehicleState[7];
                if(vehicleState[2] < 180){
                    UAVyaw = vehicleState[2];
                }else{
                    UAVyaw = -(360-vehicleState[2]);
                }
                LOSangle = Math.atan2(waypointList[waypointPointer + 1][1] - waypointList[waypointPointer][1], waypointList[waypointPointer + 1][0] - waypointList[waypointPointer][0]) * rad2deg;      //Angle from vector W_i+1 - W_i to world - Theta = atan2(y_i+1-y_i, x_i+1-x_i)
                UAVangle = Math.atan2(UAVy - waypointList[waypointPointer][1], UAVx - waypointList[waypointPointer][0]) * rad2deg;     //Angle between UAV and current waypoint W_i
                distance = Math.sqrt(Math.pow(UAVx - waypointList[waypointPointer][0], 2) + Math.pow(UAVy - waypointList[waypointPointer][1], 2));  //Distance between UAV and current waypoint W_i
                crossTrackError = distance * Math.sin(LOSangle - UAVangle);
                missDistance = Math.sqrt(Math.pow(waypointList[waypointPointer + 1][0] - UAVx, 2) + Math.pow(waypointList[waypointPointer + 1][1] - UAVy, 2));
                RollCommand = k1 * (LOSangle - UAVyaw) + k2 * crossTrackError;
                //Change to next waypoint
                if (missDistance < rAcceptance) {
                    waypointPointer++;
                    if (waypointList.length - 1 < waypointPointer) {
                        //waypointPointer = waypointList.length - 1;
                        waypointPointer = 0;    //This is just to see if the plane can circle around waypoints
                    }
                }

                //sendDataToActivity(RollCommand, "GuidanceUpdate");
                Log.i("Guidance", "LOS angle " + LOSangle + " UAV angle " + UAVangle + " Roll setpoint " + RollCommand + " Yaw " + UAVyaw + " CrossError " +crossTrackError);

                //Loop the thread - 5Hz
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            Log.i("SystemState", "Guidance thread shutting down");
        }
    };

    private Runnable guidancePurePursuit = new Runnable() {
        double UAVyaw;
        double UAVx;
        double UAVy;
        double LOSangle;
        int waypointPointer = 0;
        double RollCommand;
        double k1 = 0.1;
        double missDistance;

        @Override
        public void run() {
            Log.i("SystemState","Guidance thread started");
            while(!threadInterrupt){
                UAVx = vehicleState[6]; UAVy = vehicleState[7];
                if(vehicleState[2] < 180){
                    UAVyaw = vehicleState[2];
                }else{
                    UAVyaw = -(360-vehicleState[2]);
                }
                LOSangle = Math.atan2(waypointList[waypointPointer + 1][1] - waypointList[waypointPointer][1], waypointList[waypointPointer + 1][0] - waypointList[waypointPointer][0]) * rad2deg;      //Angle from vector W_i+1 - W_i to world - Theta = atan2(y_i+1-y_i, x_i+1-x_i)
                missDistance = Math.sqrt(Math.pow(waypointList[waypointPointer + 1][0] - UAVx, 2) + Math.pow(waypointList[waypointPointer + 1][1] - UAVy, 2));
                RollCommand = k1*(LOSangle-UAVyaw);
                //Change to next waypoint
                if (missDistance < rAcceptance) {
                    waypointPointer++;
                    if (waypointList.length - 1 < waypointPointer) {
                        //waypointPointer = waypointList.length - 1;
                        waypointPointer = 0;    //This is just to see if the plane can circle around waypoints
                    }
                }

                Log.i("Guidance","LOS angle "+LOSangle+" Roll setpoint "+RollCommand+" Yaw "+UAVyaw+" missDistance "+missDistance+" waypoint "+waypointList[waypointPointer+1][0]+" "+waypointList[waypointPointer+1][1]+" WaypointNR "+waypointPointer+" CurrentLocation "+UAVx+" "+UAVy);
                //Loop the thread - 5Hz
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            Log.i("SystemState","Guidance thread shutting down");
        }
    };



}