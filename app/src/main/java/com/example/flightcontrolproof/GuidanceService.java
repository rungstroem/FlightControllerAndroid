package com.example.flightcontrolproof;

import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.os.IBinder;
import android.util.Log;

import androidx.localbroadcastmanager.content.LocalBroadcastManager;

public class GuidanceService extends Service {
    Thread guidanceThread;
    Thread altitudeThread;
    volatile boolean threadInterrupt = false;

    //Altitude PID controller
    double[] altitudeGuidanceGains = new double[3];
    PIDController altitudeGuidanceController;

    //Usefull variables
    double rad2deg = 180.0/Math.PI;
    double rAcceptance = 5; //5m radius of acceptance

    double[][] waypointList;
    double[] vehicleState = new double[9];  //[Roll Pitch Yaw p q r Px Py Pz]
    double altitudeSP;



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
        LocalBroadcastManager.getInstance(this).registerReceiver(KalmanReceiver, new IntentFilter("KalmanUpdate"));
        //Register receiver for altitude controller gain update
        LocalBroadcastManager.getInstance(this).registerReceiver(altitudeGuidanceGainUpdate, new IntentFilter("AltitudeGuidanceGainUpdate"));

        //Test waypoint list
        waypointList = new double[2][2];
        waypointList[0][0] = 2; waypointList[0][1] = 3;
        waypointList[1][0] = 4; waypointList[1][1] = 8;

        //Altitude controller
        altitudeGuidanceGains = new double[3];
        altitudeGuidanceGains[0] = 2.0;  altitudeGuidanceGains[1] = 0.0;  altitudeGuidanceGains[2] = 0.0;
        altitudeGuidanceController = new PIDController(altitudeGuidanceGains[0], altitudeGuidanceGains[1],altitudeGuidanceGains[2]);

        //Guidance threads
        guidanceThread = new Thread(guidanceLOS);
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

    double steadyZone(double setpoint, double u, double deadZone){
        return ( (setpoint-deadZone < u) ? ( (u < setpoint+deadZone) ? setpoint : u) : u);
    }

    BroadcastReceiver KalmanReceiver = new BroadcastReceiver() {
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
            altitudeGuidanceGains = b.getDoubleArray("gainVector");
            altitudeGuidanceController.setGains(altitudeGuidanceGains[0],altitudeGuidanceGains[1],altitudeGuidanceGains[2]);
        }
    };

    public void sendDataToActivity(double data, String intentIdentifier){
        Intent intent = new Intent(intentIdentifier);
        intent.putExtra("data", data);
        LocalBroadcastManager.getInstance(this).sendBroadcast(intent);
    }

    double setPoint;
    private Runnable guidanceCC = new Runnable() {
        @Override
        public void run() {
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

            Log.i("SystemState","Guidance thread started");

            while(!threadInterrupt){
                UAVx = vehicleState[6]; UAVy = vehicleState[7]; UAVyaw = vehicleState[2];
                LOSangle = Math.atan2(waypointList[waypointPointer+1][1]-waypointList[waypointPointer][1],waypointList[waypointPointer+1][0]-waypointList[waypointPointer][0])*rad2deg;
                UAVangle = Math.atan2(UAVy-waypointList[waypointPointer][1],UAVx-waypointList[waypointPointer][0])*rad2deg;
                distance = Math.sqrt(Math.pow(UAVx-waypointList[waypointPointer][0],2)+Math.pow(UAVy-waypointList[waypointPointer][1],2));  //Distance between UAV and current waypoint W_i
                pLOS = Math.sqrt(Math.pow(LookAhead,2)+Math.pow(distance*Math.sin(LOSangle-UAVangle),2));
                crossTrackErrorAngle = Math.asin(distance*Math.sin(LOSangle-UAVangle)/pLOS)*rad2deg;
                heading = crossTrackErrorAngle+LOSangle;
                missDistance = Math.sqrt(Math.pow(waypointList[waypointPointer+1][0]-UAVx,2)+Math.pow(waypointList[waypointPointer+1][1]-UAVy,2));
                setPoint = k*(heading-UAVyaw);  //Can also include air speed Va - setPoint=k*(heading-UAVyaw)*Va

                //Change to next waypoint
                if(missDistance < rAcceptance){
                    waypointPointer++;
                    if(waypointList.length -1 < waypointPointer) waypointPointer = waypointList.length-1;
                }

                Log.i("Guidance","LOS angle "+LOSangle+" UAV angle "+UAVangle+" Roll setpoint "+setPoint+" Yaw "+vehicleState[2]);

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
        @Override
        public void run() {
            double k1 = 0.1, k2 = 0.1;
            double UAVx, UAVy, UAVyaw;
            double crossTrackError;
            int waypointPointer = 0;

            double RollCommand;
            double LOSangle;
            double UAVangle;
            double distance;
            double missDistance;

            Log.i("SystemState", "Guidance thread started");
            while (!threadInterrupt) {
                UAVx = vehicleState[6];
                UAVy = vehicleState[7];
                UAVyaw = vehicleState[2];
                LOSangle = Math.atan2(waypointList[waypointPointer + 1][1] - waypointList[waypointPointer][1], waypointList[waypointPointer + 1][0] - waypointList[waypointPointer][0]) * rad2deg;      //Angle from vector W_i+1 - W_i to world - Theta = atan2(y_i+1-y_i, x_i+1-x_i)
                UAVangle = Math.atan2(UAVy - waypointList[waypointPointer][1], UAVx - waypointList[waypointPointer][0]) * rad2deg;     //Angle between UAV and current waypoint W_i
                distance = Math.sqrt(Math.pow(UAVx - waypointList[waypointPointer][0], 2) + Math.pow(UAVy - waypointList[waypointPointer][1], 2));  //Distance between UAV and current waypoint W_i
                crossTrackError = distance * Math.sin(LOSangle - UAVangle);
                missDistance = Math.sqrt(Math.pow(waypointList[waypointPointer + 1][0] - UAVx, 2) + Math.pow(waypointList[waypointPointer + 1][1] - UAVy, 2));
                RollCommand = k1 * (LOSangle - UAVyaw) + k2 * crossTrackError;

                //Change to next waypoint
                if (missDistance < rAcceptance) {
                    waypointPointer++;
                    if (waypointList.length - 1 < waypointPointer)
                        waypointPointer = waypointList.length - 1;
                }

                sendDataToActivity(RollCommand, "GuidanceUpdate");
                Log.i("Guidance", "LOS angle " + LOSangle + " UAV angle " + UAVangle + " Roll setpoint " + setPoint + " Yaw " + vehicleState[2]);

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
        @Override
        public void run() {
            double bearing;
            double heading;
            double k = 0.1;
            int waypointPointer = 0;
            double missDistance;

            Log.i("SystemState","Guidance thread started");
            while(!threadInterrupt){
                bearing = Math.acos(waypointList[waypointPointer][0]/Math.sqrt(Math.pow(waypointList[waypointPointer][0],2)+Math.pow(waypointList[waypointPointer][1],2)))*rad2deg;
                heading = bearing-vehicleState[2]*k;
                missDistance = Math.sqrt(Math.pow(waypointList[waypointPointer+1][0]-vehicleState[6],2)+Math.pow(waypointList[waypointPointer+1][1]-vehicleState[7],2));
                if(missDistance < rAcceptance){
                    waypointPointer++;
                    if(waypointList.length -1 < waypointPointer) waypointPointer = waypointList.length-1;
                }

                Log.i("Guidance","Heading"+heading+" bearing "+bearing+" Yaw "+vehicleState[2]);
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

    private Runnable guidanceAltitude = new Runnable() {
        @Override
        public void run() {
            double pitchCommand;

            Log.i("SystemState", "Altitude guidance thread started");
            while(!threadInterrupt){
                pitchCommand = altitudeGuidanceController.control(altitudeSP,steadyZone(altitudeSP,vehicleState[8],1),0.1);

                Log.i("Guidance","Altitude "+vehicleState[8]+" setpoint "+pitchCommand);
                sendDataToActivity(pitchCommand, "AltitudeGuidanceUpdate");
                //Loop the thread - 10Hz
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            Log.i("SystemState","Altitude guidance thread shutting down");
        }
    };

}