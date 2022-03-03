package com.example.flightcontrolproof;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.security.keystore.SecureKeyImportUnavailableException;
import android.text.TextUtils;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;


public class AutopilotPID extends AppCompatActivity implements View.OnClickListener{
    EditText pitchKpInput;
    EditText pitchKiInput;
    EditText pitchKdInput;
    EditText heightKpInput;
    EditText heightKiInput;
    EditText heightKdInput;
    EditText throttleKpInput;
    EditText throttleKiInput;
    EditText throttleKdInput;
    Button pitchOKButtonInput;
    Button heightOKButtonInput;
    Button throttleOKButtonInput;

    private double pitchKp = 0.0;
    private double pitchKi = 0.0;
    private double pitchKd = 0.0;

    private double heightKp = 0.0;
    private double heightKi = 0.0;
    private double heightKd = 0.0;

    private double throttleKp = 0.0;
    private double throttleKi = 0.0;
    private double throttleKd = 0.0;

    Serialconnection serialconnection;
    PIDController pitchController;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_autopilot);
        pitchKpInput = findViewById(R.id.PitchKp);
        pitchKiInput = findViewById(R.id.PitchKi);
        pitchKdInput = findViewById(R.id.PitchKd);

        heightKpInput = findViewById(R.id.HeightKp);
        heightKiInput = findViewById(R.id.HeightKi);
        heightKdInput = findViewById(R.id.HeightKd);

        throttleKpInput = findViewById(R.id.ThrottleKp);
        throttleKiInput = findViewById(R.id.ThrottleKi);
        throttleKdInput = findViewById(R.id.ThrottleKd);

        pitchOKButtonInput = findViewById(R.id.PitchOKButton);
        pitchOKButtonInput.setOnClickListener(this);

        heightOKButtonInput = findViewById(R.id.HeightOKButton);
        heightOKButtonInput.setOnClickListener(this);

        throttleOKButtonInput = findViewById(R.id.ThrottleOKButton);
        throttleOKButtonInput.setOnClickListener(this);

        serialconnection = new Serialconnection(9600, 8, this);
        //serialconnection.tx_data(2);

        pitchController = new PIDController(pitchKp,pitchKi,pitchKd);
        pitchController.setSaturation(30,-30);
        Log.i("PIDOutput","Output "+pitchController.control(5,2,0.001));
    }

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
                break;

            case R.id.HeightOKButton:
                if( !TextUtils.isEmpty( heightKpInput.getText().toString() ) ) heightKp = Double.parseDouble(heightKpInput.getText().toString());
                if( !TextUtils.isEmpty( heightKiInput.getText().toString() ) ) heightKi = Double.parseDouble(heightKiInput.getText().toString());
                if( !TextUtils.isEmpty( heightKdInput.getText().toString() ) ) heightKd = Double.parseDouble(heightKdInput.getText().toString());
                Toast.makeText(getApplicationContext(),"Kp "+heightKp+" Ki "+heightKi+" Kd "+heightKd, Toast.LENGTH_LONG).show();
                heightKpInput.setText("");
                heightKiInput.setText("");
                heightKdInput.setText("");
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