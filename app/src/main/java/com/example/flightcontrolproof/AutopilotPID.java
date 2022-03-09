package com.example.flightcontrolproof;

import androidx.appcompat.app.AppCompatActivity;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.os.PowerManager;
import android.security.keystore.SecureKeyImportUnavailableException;
import android.text.TextUtils;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;


public class AutopilotPID extends AppCompatActivity implements View.OnClickListener{
    EditText pitchKpInput;
    EditText pitchKiInput;
    EditText pitchKdInput;
    EditText rollKpInput;
    EditText rollKiInput;
    EditText rollKdInput;
    EditText pDampKpInput;
    EditText pDampKiInput;
    EditText pDampKdInput;
    EditText qDampKpInput;
    EditText qDampKiInput;
    EditText qDampKdInput;
    EditText heightKpInput;
    EditText heightKiInput;
    EditText heightKdInput;
    EditText throttleKpInput;
    EditText throttleKiInput;
    EditText throttleKdInput;
    Button pitchOKButtonInput;
    Button rollOkButtonInput;
    Button heightOKButtonInput;
    Button throttleOKButtonInput;
    Button pDampOKButtonInput;
    Button qDampOKButtonInput;

    private double pitchKp = 2;
    private double pitchKi = 0.3;
    private double pitchKd = 0.5;

    private double rollKp = 2;
    private double rollKi = 0.5;
    private double rollKd = 0.1;

    private double qRateKp = 0;
    private double qRateKi = 0.05;
    private double qRateKd = 0;

    private double pRateKp = 0;
    private double pRateKi = 0.05;
    private double pRateKd = 0;

    private double heightKp = 2.0;
    private double heightKi = 0.0;  //Because barometer is unstable I think P controller is best.
    private double heightKd = 0.0;

    private double throttleKp = 0.0;
    private double throttleKi = 0.0;
    private double throttleKd = 0.0;

    private double rad2deg = 180/Math.PI;
    volatile boolean threadInterrupt = false;
    private double[] vehicleState = new double[9];  //[Roll Pitch Yaw p q r Px Py Pz]
    private double[][] waypointList;

    PowerManager powerManager;
    PowerManager.WakeLock wakeLock;
    Serialconnection serialconnection;

    Intent KalmanIntent;
    Intent GuidanceIntent;
    Thread controllerThread;
    Thread guidanceThread;
    Thread altitudeThread;
    LinearAlgebra LA;

    //PID controllers
    PIDController pitchController;
    PIDController qRateController;
    PIDController heightController;
    PIDController rollController;
    PIDController pRateController;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_autopilot);

        pitchKpInput = findViewById(R.id.PitchKp);
        pitchKiInput = findViewById(R.id.PitchKi);
        pitchKdInput = findViewById(R.id.PitchKd);

        rollKpInput = findViewById(R.id.RollKp);
        rollKiInput = findViewById(R.id.RollKi);
        rollKdInput = findViewById(R.id.RollKd);

        heightKpInput = findViewById(R.id.HeightKp);
        heightKiInput = findViewById(R.id.HeightKi);
        heightKdInput = findViewById(R.id.HeightKd);

        throttleKpInput = findViewById(R.id.ThrottleKp);
        throttleKiInput = findViewById(R.id.ThrottleKi);
        throttleKdInput = findViewById(R.id.ThrottleKd);

        qDampKpInput = findViewById(R.id.qDampKp);
        qDampKiInput = findViewById(R.id.qDampKi);
        qDampKdInput = findViewById(R.id.qDampKd);

        pDampKpInput = findViewById(R.id.pDampKp);
        pDampKiInput = findViewById(R.id.pDampKi);
        pDampKdInput = findViewById(R.id.pDampKd);

        pitchOKButtonInput = findViewById(R.id.PitchOKButton);
        pitchOKButtonInput.setOnClickListener(this);

        rollOkButtonInput = findViewById(R.id.RollOKButton);
        rollOkButtonInput.setOnClickListener(this);

        heightOKButtonInput = findViewById(R.id.HeightOKButton);
        heightOKButtonInput.setOnClickListener(this);

        throttleOKButtonInput = findViewById(R.id.ThrottleOKButton);
        throttleOKButtonInput.setOnClickListener(this);

        pDampOKButtonInput = findViewById(R.id.pDampOKButton);
        pDampOKButtonInput.setOnClickListener(this);

        qDampOKButtonInput = findViewById(R.id.qDampOKButton);
        qDampOKButtonInput.setOnClickListener(this);

        //Initialize serial connection
        serialconnection = new Serialconnection(9600, 8, this);

        //Wake-lock to stop CPU from turning off
        powerManager = (PowerManager) getSystemService(POWER_SERVICE);
        wakeLock = powerManager.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "Controller::WakeLock");
        wakeLock.acquire();

        //Initialize Linear algebra library
        LA = new LinearAlgebra();

        //Start Kalman service
        KalmanIntent = new Intent(this, KalmanEstimatorService.class);
        startService(KalmanIntent);
        //Start guidance service
        GuidanceIntent = new Intent(this, GuidanceService.class);
        startService(GuidanceIntent);

        //Register receiver for Kalman data
        LocalBroadcastManager.getInstance(this).registerReceiver(KalmanReceiver, new IntentFilter("KalmanUpdate"));
        //Register receivers for guidance data
        LocalBroadcastManager.getInstance(this).registerReceiver(altitudeReceiver, new IntentFilter("AltitudeGuidanceUpdate"));
        LocalBroadcastManager.getInstance(this).registerReceiver(guidanceReceiver, new IntentFilter("GuidanceUpdate"));

        //Initialize PID controllers
        pitchController = new PIDController(pitchKp, pitchKi, pitchKd);
        pitchController.setSaturation(45,-35);
        qRateController = new PIDController(qRateKp,qRateKi,qRateKd);
        qRateController.setSaturation(45,-35);
        heightController = new PIDController(heightKp,heightKi,heightKd);
        heightController.setSaturation(45,-45); //This is angle output so set saturation to maximum pitch angle
        rollController = new PIDController(rollKp,rollKi, rollKd);
        rollController.setSaturation(35,-35);   //I don't like an un even value here...
        pRateController = new PIDController(qRateKp,qRateKi,qRateKd);
        pRateController.setSaturation(35,-35);

        waypointList = new double[2][2];
        waypointList[0][0] = 2; waypointList[0][1] = 3;
        waypointList[1][0] = 4; waypointList[1][1] = 8;

        //guidanceThread = new Thread(guidanceLOS);
        //guidanceThread.start();

        //altitudeThread = new Thread(guidanceAltitude);
        //altitudeThread.start();

        controllerThread = new Thread(controller);
        controllerThread.start();

        Log.i("SystemState","Autopilot started");
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        wakeLock.release(); //CPU can throttle down again - saves battery
        stopService(KalmanIntent);
        stopService(GuidanceIntent);
        threadInterrupt = true;
        serialconnection.finalize();

        Log.i("SystemState","Autopilot shutting down");
    }

    private BroadcastReceiver KalmanReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            Bundle b = intent.getExtras();
            vehicleState = b.getDoubleArray("stateVector");
            Log.i("Kalman","Roll "+vehicleState[0]+" Pitch "+vehicleState[1]+" Yaw "+vehicleState[2]+" p "+vehicleState[3]+" q "+vehicleState[4]+" r "+vehicleState[5]);
        }
    };

    private BroadcastReceiver altitudeReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            Bundle b = intent.getExtras();
            PitchSetpoint = b.getDouble("data");
        }
    };

    private BroadcastReceiver guidanceReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            Bundle b = intent.getExtras();
            RollSetpoint = b.getDouble("data");
        }
    };

    double[] limitAndOffsetOutput(double[] inputVector){
        if(inputVector[0] < -35) inputVector[0] = -35;
        if(inputVector[1] < -35) inputVector[1] = -35;
        if(inputVector[0] > 45) inputVector[0] = 45;
        if(inputVector[1] > 45) inputVector[1] = 45;
        inputVector[0] += 35;
        inputVector[1] += 35;
        return inputVector;
    }
    double steadyZone(double setpoint, double u, double deadZone){
        return ( (setpoint-deadZone < u) ? ( (u < setpoint+deadZone) ? setpoint : u) : u);
    }

    double[] AE = new double[2];
    double[] LR = new double[2];
    double[][] Mixing = new double[2][2];
    byte[] csData = new byte[4];
    double RollSetpoint = 0;
    double PitchSetpoint = 0;
    int sampleTime = 0;
    private Runnable controller = new Runnable() {
        @Override
        public void run() {
            Log.i("SystemState","Controller thread started");
            //Aileon    -   Elevator    -   Aileon      -       Elevator
            Mixing[0][0] = 1;   Mixing[0][1] = -1;  Mixing[1][0] = -1;  Mixing[1][1] = -1;
            //sendData.start();
            while(!threadInterrupt){
                //Speed controller here
                AE[0] = rollController.control(RollSetpoint,steadyZone(RollSetpoint,vehicleState[0],1),0.03);     //dAileron
                AE[1] = pitchController.control(PitchSetpoint,steadyZone(PitchSetpoint,vehicleState[1],1),0.03);   //dElevator
                AE[0] -= pRateController.control(0,steadyZone(0,vehicleState[3],1),0.03); //Roll rate damper  (Negative feedback)
                AE[1] -= qRateController.control(0,steadyZone(0,vehicleState[4],1),0.03); //Pitch rate damper (Negative feedback)

                Log.i("Controller", "dElevator "+AE[1]+" dAileron "+AE[0]);
                LR = LA.vectMatMultiply(Mixing,AE);
                LR = limitAndOffsetOutput(LR);
                Log.i("Autopilot","LElevon "+LR[0]+" RElevon "+LR[1]+" RollState "+vehicleState[0]+" PitchState "+vehicleState[1]+" RollSetpoint "+RollSetpoint+" PitchSetpoint "+PitchSetpoint+" Ts "+sampleTime);
                sampleTime++;

                csData[0] = 0x02;
                csData[1] = (byte) ((int)(LR[0]) & 0xFF);
                csData[2] = 0x03;
                csData[3] = (byte) ((int)(LR[1]) & 0xFF);
                if(serialconnection.USBDeviceOK()) {
                    serialconnection.tx_data(csData);
                }
                //Loop the thread - roughly 30Hz
                try {
                    Thread.sleep(30);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            Log.i("SystemState","Controller thread shutting down");
        }
    };

    @Override
    public void onClick(View v){
        switch (v.getId()){
            case R.id.PitchOKButton:
                if( !TextUtils.isEmpty( pitchKpInput.getText().toString() ) ) pitchKp = Double.parseDouble(pitchKpInput.getText().toString());
                if( !TextUtils.isEmpty( pitchKiInput.getText().toString() ) ) pitchKi = Double.parseDouble(pitchKiInput.getText().toString());
                if( !TextUtils.isEmpty( pitchKdInput.getText().toString() ) ) pitchKd = Double.parseDouble(pitchKdInput.getText().toString());
                Toast.makeText(getApplicationContext(),"Kp "+pitchKp+" Ki "+pitchKi+" Kd "+pitchKd, Toast.LENGTH_LONG).show();
                pitchKpInput.setText("");
                pitchKiInput.setText("");
                pitchKdInput.setText("");
                pitchController.setGains(pitchKp,pitchKi,pitchKd);
                break;

            case R.id.RollOKButton:
                if( !TextUtils.isEmpty( rollKpInput.getText().toString() ) ) rollKp = Double.parseDouble(rollKpInput.getText().toString());
                if( !TextUtils.isEmpty( rollKiInput.getText().toString() ) ) rollKi = Double.parseDouble(rollKiInput.getText().toString());
                if( !TextUtils.isEmpty( rollKdInput.getText().toString() ) ) rollKd = Double.parseDouble(rollKdInput.getText().toString());
                Toast.makeText(getApplicationContext(),"Kp "+rollKp+" Ki "+rollKi+" Kd "+rollKd, Toast.LENGTH_LONG).show();
                rollKpInput.setText("");
                rollKiInput.setText("");
                rollKdInput.setText("");
                rollController.setGains(rollKp,rollKi,rollKd);
                break;

            case R.id.pDampOKButton:
                if( !TextUtils.isEmpty( pDampKpInput.getText().toString() ) ) pRateKp = Double.parseDouble(pDampKpInput.getText().toString());
                if( !TextUtils.isEmpty( pDampKiInput.getText().toString() ) ) pRateKi = Double.parseDouble(pDampKiInput.getText().toString());
                if( !TextUtils.isEmpty( pDampKdInput.getText().toString() ) ) pRateKd = Double.parseDouble(pDampKdInput.getText().toString());
                Toast.makeText(getApplicationContext(),"Kp "+pRateKp+" Ki "+pRateKi+" Kd "+pRateKd, Toast.LENGTH_LONG).show();
                pitchKpInput.setText("");
                pitchKiInput.setText("");
                pitchKdInput.setText("");
                pRateController.setGains(pRateKp,pRateKi,pRateKd);
                break;

            case R.id.qDampOKButton:
                if( !TextUtils.isEmpty( qDampKpInput.getText().toString() ) ) qRateKp = Double.parseDouble(qDampKpInput.getText().toString());
                if( !TextUtils.isEmpty( qDampKiInput.getText().toString() ) ) qRateKi = Double.parseDouble(qDampKiInput.getText().toString());
                if( !TextUtils.isEmpty( qDampKdInput.getText().toString() ) ) qRateKd = Double.parseDouble(qDampKdInput.getText().toString());
                Toast.makeText(getApplicationContext(),"Kp "+qRateKp+" Ki "+qRateKi+" Kd "+qRateKd, Toast.LENGTH_LONG).show();
                pitchKpInput.setText("");
                pitchKiInput.setText("");
                pitchKdInput.setText("");
                qRateController.setGains(qRateKp,qRateKi,qRateKd);
                break;

            case R.id.HeightOKButton:
                if( !TextUtils.isEmpty( heightKpInput.getText().toString() ) ) heightKp = Double.parseDouble(heightKpInput.getText().toString());
                if( !TextUtils.isEmpty( heightKiInput.getText().toString() ) ) heightKi = Double.parseDouble(heightKiInput.getText().toString());
                if( !TextUtils.isEmpty( heightKdInput.getText().toString() ) ) heightKd = Double.parseDouble(heightKdInput.getText().toString());
                Toast.makeText(getApplicationContext(),"Kp "+heightKp+" Ki "+heightKi+" Kd "+heightKd, Toast.LENGTH_LONG).show();
                heightKpInput.setText("");
                heightKiInput.setText("");
                heightKdInput.setText("");
                heightController.setGains(heightKp,heightKi,heightKd);
                break;

            case R.id.ThrottleOKButton:
                if( !TextUtils.isEmpty( throttleKpInput.getText().toString() ) ) throttleKp = Double.parseDouble(throttleKpInput.getText().toString());
                if( !TextUtils.isEmpty( throttleKiInput.getText().toString() ) ) throttleKi = Double.parseDouble(throttleKiInput.getText().toString());
                if( !TextUtils.isEmpty( throttleKdInput.getText().toString() ) ) throttleKd = Double.parseDouble(throttleKdInput.getText().toString());
                Toast.makeText(getApplicationContext(),"Kp "+throttleKp+" Ki "+throttleKi+" Kd "+throttleKd, Toast.LENGTH_LONG).show();
                throttleKpInput.setText("");
                throttleKiInput.setText("");
                throttleKdInput.setText("");
                break;
        }
    }

    double rAcceptance = 5;  //5m radius of acceptance
    double setPoint;
    int waypointPointer = 0;
    private Runnable guidanceCC = new Runnable() {
        @Override
        public void run() {
            double UAVx, UAVy, UAVyaw;
            double LOSangle;
            double UAVangle;
            double distance;
            double LookAhead = 5;   //5m look-ahead
            double pLOS;
            double crossTrackErrorAngle;
            double heading;
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
            double LOSangle;
            double UAVangle;
            double distance;
            double crossTrackError;
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
                RollSetpoint = k1 * (LOSangle - UAVyaw) + k2 * crossTrackError;
                if (missDistance < rAcceptance) {
                    waypointPointer++;
                    if (waypointList.length - 1 < waypointPointer)
                        waypointPointer = waypointList.length - 1;
                }

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
            double missDistance;
            double k = 0.1;
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
        double altitudeSP = 3;
        double test;
        @Override
        public void run() {
            while(!threadInterrupt){
                test = heightController.control(altitudeSP,steadyZone(altitudeSP,vehicleState[8],1),0.1);
                Log.i("Guidance","Altitude "+vehicleState[8]+" setpoint "+test);
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


    // This is a thread for sending serial data. It waits 50mS between sending commands. Use this if the µC cannot keep up
        /*Thread sendData = new Thread(new Runnable() {
        @Override
        public void run() {
            byte[] data = new byte[2];
            data[0] = 0x03;
            int i = 0;
            while(true){
                //data[1] = (byte) ((int)(dElevator+30) & 0xFF);
                //serialconnection.tx_data(data[i]);
                i++;
                i %= 2;
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    });*/


   // This is a thread for a simple guidance method. It calculates the difference between the Yaw angle and the angle towards the waypoint
    /*
    double[] waypoint = new double[3];
    double bearing;
    double heading;
    double rad2deg = 180/Math.PI;
    private Runnable guidance = new Runnable() {
        @Override
        public void run() {
            Log.i("SystemState","Guidance thread started");

            waypoint[0] = 2;    waypoint[1] = 3;
            while(!threadInterrupt){
                bearing = Math.acos(waypoint[0]/Math.sqrt(Math.pow(waypoint[0],2)+Math.pow(waypoint[1],2)))*rad2deg;
                heading = bearing-vehicleState[2];
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
    };*/


    /*
    Height to pitch
        - Input: Height command and actual height
        - Error = Height command - actual height
        - Output: Pitch command (in Euler)
        - Probably PI controller
    Pitch to elevator
        - Input: Pitch command and actual pitch
        - Error = Pitch command - actual pitch
        - Output: Elevator deflection
    Velocity to throttle
        - Input: Velocity command and actual velocity
        - Error = Velocity command - actual velocity
        - Output: throttle command
    Roll to aileron
        - Input: Roll command and actual Roll
        - Error = Roll command - actual Roll
        - Output: Aileron deflection
    Yaw damper - maybe ??

    Remember to add trim to all actuator deflections.
     */
}