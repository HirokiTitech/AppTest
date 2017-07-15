package jp.ac.titech.itpro.sdl.apptest;

import android.Manifest;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.net.Uri;
import android.os.Handler;
import android.os.Message;
import android.os.SystemClock;
import android.support.annotation.NonNull;
import android.support.annotation.Nullable;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.telecom.TelecomManager;
import android.telephony.TelephonyManager;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.widget.Toast;

import com.google.android.gms.common.ConnectionResult;
import com.google.android.gms.common.api.GoogleApiClient;
import com.google.android.gms.location.LocationListener;
import com.google.android.gms.location.LocationRequest;
import com.google.android.gms.location.LocationServices;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.MapFragment;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.MarkerOptions;

import java.text.SimpleDateFormat;
import java.util.Timer;
import java.util.TimerTask;
import java.util.zip.DataFormatException;

public class MainActivity extends AppCompatActivity implements SensorEventListener, GoogleApiClient.ConnectionCallbacks, GoogleApiClient.OnConnectionFailedListener, LocationListener, View.OnClickListener, Runnable, OnMapReadyCallback, GoogleMap.OnMapLongClickListener, Handler.Callback {

    private final static String TAG = "MainActivity";

    private String mConnectedDeviceName = null;
    // Bluetooth class object
    private BluetoothAdapter mBluetoothAdapter;
    private BluetoothService mBluetoothService = null;
    // Intent request codes
    private static final int REQUEST_CONNECT_DEVICE_SECURE = 1;
    private static final int REQUEST_CONNECT_DEVICE_INSECURE = 2;
    private static final int REQUEST_ENABLE_BT = 3;
    private static final int REQUEST_PREFERENCE = 4;

    // Serial Communication
    private static final byte START_POINT = 'S';
    private static final byte END_POINT = 'E';
    private byte[] mTXPacket = new byte[3];
    private byte[] mRXPacket = new byte[3];
    private int[] mData = new int[5000];
    private int mIndex;

    private Handler mHandler;

    // Sensors parameters
    private SensorManager sensorManager;
    private Sensor accelerometer, magnetometer, gravitymeter;
    private float[] attitudeDegree;
    private float[] acceleration, gravity, magnetism;
    private double stopAccelValue, preYaw;
    private float[] yawArrey;
    private float yawSum;
    private int arreyIndex;
    private static final int INDEX__NUM = 10;

    // GPS parameters
    private GoogleMap googleMap;
    private GoogleApiClient googleApiClient;
    private LocationRequest locationRequest;
    private final static String[] PERMISSIONS = {
            Manifest.permission.ACCESS_COARSE_LOCATION,
            Manifest.permission.ACCESS_FINE_LOCATION
    };
    private final static int REQCODE_PERMISSIONS = 1111;
    private double latitude, longitude, altitude;
    private double targetLatitude, targetLongitude;
    private Location oldLocation = null;



    private enum UpdatingState {STOPPED, REQUESTING, STARTED}

    private UpdatingState state = UpdatingState.STOPPED;

    // UI parameters
    private TextView rotationView, positionView;
    private TextView velocityView, distanceView, timeView, slopeView, decelerationView, turnView, mileageView;
    private TextView actionView;
    private int actionColor;
    private String actionText;
    private ProgressBar mileageBar;
    private int decelerationColor, turnColor;
    private Button startButton, holdButton, resetButton, calculateButton, connectButton;
    private int startColor, holdColor, resetColor;
    private SimpleDateFormat dateFormat = new SimpleDateFormat("HH:mm");

    private double distance, velocity,time, startTime, slope, slopeOffset;
    private double targetDistance, targetDirection, maxDistance;
    private int holdFlag = 0;

    // Telephone
    TelephonyManager telephonyManager;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Log.d(TAG, "onCreate");

        // Initialize sensors
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        if (accelerometer == null) {
            Toast.makeText(this, getString(R.string.toast_no_accel_error), Toast.LENGTH_SHORT).show();
            finish();
            return;
        }
        magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        if (magnetometer == null) {
            Toast.makeText(this, getString(R.string.toast_no_magneto_error), Toast.LENGTH_SHORT).show();
            finish();
            return;
        }
        gravitymeter = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
        if (gravitymeter == null) {
            Toast.makeText(this, getString(R.string.toast_no_gravity_error), Toast.LENGTH_SHORT).show();
            finish();
            return;
        }
        attitudeDegree = new float[3];
        yawArrey = new float[INDEX__NUM];
        arreyIndex = 0;

        // Initialize GPS
        MapFragment mapFragment = (MapFragment) getFragmentManager().findFragmentById(R.id.map_fragment);
        mapFragment.getMapAsync(this);


        googleApiClient = new GoogleApiClient.Builder(this)
                .addConnectionCallbacks(this)
                .addOnConnectionFailedListener(this)
                .addApi(LocationServices.API)
                .build();
        locationRequest = new LocationRequest();
        locationRequest.setInterval(1000);
        locationRequest.setFastestInterval(500);
        locationRequest.setPriority(LocationRequest.PRIORITY_BALANCED_POWER_ACCURACY);

        // Initialize UI components
        velocityView = (TextView) findViewById(R.id.velocityView);
        distanceView = (TextView) findViewById(R.id.distanceView);
        timeView = (TextView) findViewById(R.id.timeView);
        slopeView = (TextView) findViewById(R.id.slopeView);
        decelerationView = (TextView) findViewById(R.id.decelerationView);
        rotationView = (TextView) findViewById(R.id.rotationView);
        positionView = (TextView) findViewById(R.id.positionView);
        turnView = (TextView) findViewById(R.id.turnView);
        mileageView = (TextView) findViewById(R.id.mileageView);
        actionView = (TextView) findViewById(R.id.actionView);
        startButton = (Button) findViewById(R.id.startButton);
        startButton.setOnClickListener(this);
        holdButton = (Button) findViewById(R.id.holdButton);
        holdButton.setOnClickListener(this);
        resetButton = (Button) findViewById(R.id.resetButton);
        resetButton.setOnClickListener(this);
        calculateButton = (Button) findViewById(R.id.calculateButton);
        calculateButton.setOnClickListener(this);
        connectButton = (Button) findViewById(R.id.connectButton);
        connectButton.setOnClickListener(this);
        mileageBar = (ProgressBar) findViewById(R.id.mileageBar);
        mileageBar.setMax(1000);
        mileageBar.setProgress(1000);

        // Initialize bike parameters
        velocity = 0f;
        distance = 0f;
        targetLatitude = 35.6058926f;
        targetLongitude = 139.6809381f;
        maxDistance = 0;
        actionColor = Color.TRANSPARENT;
        actionText = getString(R.string.text_stop);

        // Initialize telephone
        telephonyManager = (TelephonyManager) getSystemService(Context.TELEPHONY_SERVICE);
        if (telephonyManager.getPhoneType() == TelephonyManager.PHONE_TYPE_NONE) {
            Toast.makeText(this, getString(R.string.toast_no_telephone_error), Toast.LENGTH_SHORT).show();
        }

        // Get local Bluetooth adapter
        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        // If the adapter is null, then Bluetooth is not supported
        if (mBluetoothAdapter == null) {
            Toast.makeText(this, "Bluetooth is not available", Toast.LENGTH_LONG).show();
            Log.v(TAG,  "Bluetooth is not available");
            finish();
        }
    }

    @Override
    protected void onStart() {
        Log.d(TAG, "onStart");
        super.onStart();
        if (!mBluetoothAdapter.isEnabled()) {
            Log.v(TAG, "Turn on Bluetooth if enable");
            Intent enableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableIntent, REQUEST_ENABLE_BT);
            // Otherwise, setup the chat session
        } else if (mBluetoothService == null) {
            mHandler = new Handler(this);
            mBluetoothService = new BluetoothService(this, mHandler);
        }
        googleApiClient.connect();
    }

    @Override
    protected void onResume() {
        Log.d(TAG, "onResume");
        super.onResume();

        sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_NORMAL);
        sensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_NORMAL);
        sensorManager.registerListener(this, gravitymeter, SensorManager.SENSOR_DELAY_NORMAL);

        if (state != UpdatingState.STARTED && googleApiClient.isConnected()) {
            startLocationUpdate(true);
        } else {
            state = UpdatingState.REQUESTING;
        }

        if (mBluetoothService != null) {
            // Only if the state is STATE_NONE, do we know that we haven't started already
            if (mBluetoothService.getState() == BluetoothService.STATE_NONE) {
                // Start the Bluetooth chat services
                mBluetoothService.start();
            }
        }
        clickHold();
        clickReset();
    }

    @Override
    protected void onPause() {
        Log.d(TAG, "onPause");
        super.onPause();

        sensorManager.unregisterListener(this);

        if (state == UpdatingState.STARTED) {
            stopLocationUpdate();
        }
    }

    @Override
    protected void onStop() {
        Log.d(TAG, "onStop");
        super.onStop();

        googleApiClient.disconnect();
    }

    @Override
    public void onDestroy() {
        Log.v(TAG, "onDestroy is called");
        super.onDestroy();
        if (mBluetoothService != null) {
            mBluetoothService.stop();
        }
    }

    @Override
    public void onActivityResult(int requestCode, int resultCode, Intent data) {
        switch (requestCode) {
            case REQUEST_CONNECT_DEVICE_SECURE:
                Log.v(TAG, "REQUEST_CONNECT_DEVICE_SECURE");
                // When DeviceListActivity returns with a device to connect
                if (resultCode == Activity.RESULT_OK) {
                    // Get the device MAC address
                    String address = data.getExtras()
                            .getString(DeviceListActivity.EXTRA_DEVICE_ADDRESS);
                    // Get the BluetoothDevice object
                    BluetoothDevice device = mBluetoothAdapter.getRemoteDevice(address);
                    // Attempt to connect to the device
                    mBluetoothService.connect(device, true);

                }
                break;
            case REQUEST_CONNECT_DEVICE_INSECURE:
                Log.d(TAG, "REQUEST_CONNECT_DEVICE_INSECURE");
                // When DeviceListActivity returns with a device to connect
                if (resultCode == Activity.RESULT_OK) {
                    // Get the device MAC address
                    String address = data.getExtras()
                            .getString(DeviceListActivity.EXTRA_DEVICE_ADDRESS);
                    // Get the BluetoothDevice object
                    BluetoothDevice device = mBluetoothAdapter.getRemoteDevice(address);
                    // Attempt to connect to the device
                    mBluetoothService.connect(device, false);
                    mTXPacket[0] = START_POINT;
                    mTXPacket[1] = 'A';
                    mTXPacket[2] = END_POINT;
                    mBluetoothService.write(mTXPacket);
                }
                break;
            case REQUEST_ENABLE_BT:
                Log.d(TAG, "REQUEST_ENABLE_BT");
                // When the request to enable Bluetooth returns
                if (resultCode == Activity.RESULT_OK) {
                    // Bluetooth is now enabled, so set up a chat session
                    mHandler = new Handler(this);
                    mBluetoothService = new BluetoothService(this, mHandler);
                } else {
                    // User did not enable Bluetooth or an error occurred
                    Log.d(TAG, "BT not enabled");
                    Toast.makeText(this, R.string.bt_not_enabled_leaving, Toast.LENGTH_SHORT).show();
                    finish();
                }
                break;
        }
    }
    @Override
    public void onRequestPermissionsResult(int reqCode,
                                           @NonNull String[] permissions, @NonNull int[] grants) {
        Log.d(TAG, "onRequestPermissionsResult");
        switch (reqCode) {
            case REQCODE_PERMISSIONS:
                startLocationUpdate(false);
                break;
        }
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.startButton:
                clickStart();
                break;
            case R.id.holdButton:
                clickHold();
                break;
            case R.id.resetButton:
                clickReset();
                break;
            case R.id.calculateButton:
                clickCalculate();
                break;
            case R.id.connectButton:
                clickConnect();
                break;
        }
        startButton.setBackgroundColor(startColor);
        holdButton.setBackgroundColor(holdColor);
        resetButton.setBackgroundColor(resetColor);
    }


    @Override
    public boolean handleMessage(Message msg) {

        switch (msg.what) {
            case Constants.MESSAGE_STATE_CHANGE:
                Log.d(TAG, "Change State");
                switch (msg.arg1) {
                    case BluetoothService.STATE_CONNECTED:
                        Log.d(TAG, "Connected");
                        Toast.makeText(this, R.string.title_connected, Toast.LENGTH_SHORT).show();
                        break;
                    case BluetoothService.STATE_CONNECTING:
                        Log.d(TAG, "Connecting");
                        Toast.makeText(this, R.string.title_connecting, Toast.LENGTH_SHORT).show();
                        break;
                    case BluetoothService.STATE_LISTEN:
                    case BluetoothService.STATE_NONE:
                        Log.d(TAG, "Fail");
                        Toast.makeText(this, R.string.title_not_connected, Toast.LENGTH_SHORT).show();
                        break;
                }
                break;
            case Constants.MESSAGE_READ:
                Log.d(TAG, "Read");
                process((byte[]) msg.obj);
                break;
            case Constants.MESSAGE_DEVICE_NAME:
                // save the connected device's name
                mConnectedDeviceName = msg.getData().getString(Constants.DEVICE_NAME);
                if (null != this) {
                    Toast.makeText(this, "Connected to "
                            + mConnectedDeviceName, Toast.LENGTH_SHORT).show();
                }
                break;
        }
        return false;
    }
    @Override
    public void run() {
        //time = SystemClock.currentThreadTimeMillis()-startTime;
        //timeView.setText((int) time);



        //positionView.setText(getString(R.string.location_format, latitude, longitude));
        //rotationView.setText(getString(R.string.posture_format, attitudeDegree[0], attitudeDegree[1], attitudeDegree[2]));
    }

    @Override
    public void onMapReady(GoogleMap map) {
        googleMap = map;
        googleMap.moveCamera(CameraUpdateFactory.zoomTo(15f));
        googleMap.setOnMapLongClickListener(this);
    }


    @Override
    public void onMapLongClick(LatLng latLng) {
        float[] result = new float[3];
        targetLatitude = latLng.latitude;
        targetLongitude = latLng.longitude;
        MarkerOptions options = new MarkerOptions();
        options.position(new LatLng(targetLatitude, targetLongitude));
        googleMap.addMarker(options);
        oldLocation.distanceBetween(oldLocation.getLatitude(), oldLocation.getLongitude(),targetLatitude,targetLongitude, result);
        maxDistance = result[0];
        Log.d(TAG,"TargetLatitude" + targetLatitude);
        Log.d(TAG,"TargetLongitude" + targetLongitude);
    }

    // Sensor method
    @Override
    public void onSensorChanged(SensorEvent event) {
        switch(event.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER:
                acceleration=event.values.clone();
                break;
            case Sensor.TYPE_MAGNETIC_FIELD:
                magnetism=event.values.clone();
                break;
            case Sensor.TYPE_GRAVITY:
                gravity=event.values.clone();
        }
        if(acceleration != null && magnetism != null && gravity != null) {
            float[] inR = new float[16];
            float[] outR = new float[16];
            float[] I = new float[16];
            SensorManager.getRotationMatrix(inR, I, acceleration, magnetism);
            SensorManager.remapCoordinateSystem(inR, SensorManager.AXIS_X, SensorManager.AXIS_Y, outR);
            SensorManager.getOrientation(outR, attitudeDegree);
            for (int i = 0; i < 3; i++) {
                attitudeDegree[i] = (float) (attitudeDegree[i] * 180 / Math.PI);
            }

            if(holdFlag == 0) {
                slope = -attitudeDegree[1] - slopeOffset;
                stopAccelValue = (float) ((acceleration[1] - gravity[1]) * Math.cos(slope * Math.PI / 180) -  (acceleration[2]-gravity[2]) * Math.sin(slope * Math.PI / 180));
                //Log.d(TAG,"stopAccelValue"+stopAccelValue);

                if(Math.abs(attitudeDegree[2]) > 75f) {
                    if(telephonyManager.getPhoneType() == TelephonyManager.PHONE_TYPE_NONE) {
                        //Toast.makeText(this, getString(R.string.toast_no_telephone_error), Toast.LENGTH_SHORT).show();
                    }else{
                        Log.d(TAG, "Intent Start");
                        Intent intent = new Intent(Intent.ACTION_DIAL, Uri.parse("tel:08052816923"));
                        startActivity(intent);
                    }
                }



                if(attitudeDegree[0] > yawSum + 40) {
                    actionColor = Color.GREEN;
                    actionText = getString(R.string.text_turn_right);
                    turnColor = Color.RED;
                    mTXPacket[1] = 'R';
                }else if(attitudeDegree[0] < yawSum - 40) {
                    actionColor = Color.CYAN;
                    actionText = getString(R.string.text_turn_left);
                    turnColor = Color.CYAN;
                    mTXPacket[1] = 'L';
                }else {
                    turnColor = Color.TRANSPARENT;
                    if(stopAccelValue > 3.0f) {
                        actionColor = Color.RED;
                        actionText = getString(R.string.text_deceleration);
                        decelerationColor = Color.RED;
                        mTXPacket[1] = 'D';
                    }else{
                        actionColor = Color.TRANSPARENT;
                        actionText = getString(R.string.text_running);
                        decelerationColor = Color.TRANSPARENT;
                        mTXPacket[1] = 'A';
                    }
                }




                slopeView.setText(getString(R.string.slope_format, (int)slope));
                //decelerationView.setBackgroundColor(decelerationColor);
                //turnView.setBackgroundColor(turnColor);
                actionView.setBackgroundColor(actionColor);
                actionView.setText(actionText);
                //runOnUiThread(this);
            }
            yawArrey[arreyIndex] = attitudeDegree[0];
            arreyIndex++;
            if(arreyIndex >= INDEX__NUM) {
                arreyIndex = 0;
            }
            for(int i = 0; i < INDEX__NUM; i++) {
                yawSum += yawArrey[i];
            }
            yawSum = yawSum/INDEX__NUM;
            //Log.d(TAG,"yawSum = " + yawSum);
            //Log.d(TAG,"yaw = " + attitudeDegree[0]);
            //Log.d(TAG,"accelX=" + accelValue[0]);
            //Log.d(TAG,"accelY=" + accelValue[1]);
            //Log.d(TAG,"accelZ=" + accelValue[2]);
            //Log.d(TAG,"magnetoX=" + magnetoValue[0]);
            //Log.d(TAG,"magnetoY=" + magnetoValue[1]);
            //Log.d(TAG,"magnetoZ=" + magnetoValue[2]);
            //Log.d(TAG,"Yaw=" + attitudeDegree[0]);
            //Log.d(TAG,"Pitch=" + attitudeDegree[1]);
            //Log.d(TAG,"Roll=" + attitudeDegree[2]);
            //Log.d(TAG,"stopAccelValue=" + stopAccelValue);
            preYaw = attitudeDegree[0];
            acceleration = null;
            magnetism = null;
            gravity = null;
            mTXPacket[0] = START_POINT;
            mTXPacket[2] = END_POINT;
            mBluetoothService.write(mTXPacket);
            Log.d(TAG, "Receive Packet" + mRXPacket);
            Log.d(TAG, "Send Packet" + mTXPacket);
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    @Override
    public void onConnected(@Nullable Bundle bundle) {
        Log.d(TAG, "onConnected");
        if (state == UpdatingState.REQUESTING) {
            startLocationUpdate(true);
        }
    }

    @Override
    public void onConnectionSuspended(int i) {
        Log.d(TAG, "onConnectionSuspended");
    }

    @Override
    public void onConnectionFailed(@NonNull ConnectionResult connectionResult) {
        Log.d(TAG, "onConnectionFailed");
    }

    @Override
    public void onLocationChanged(Location location) {
        Log.d(TAG, "location is changed");
        float[] result = new float[3];
        latitude = location.getLatitude();
        longitude = location.getLongitude();
        altitude = location.getAltitude();
        //Log.d(TAG,"Latitude=" + latitude);
        //Log.d(TAG,"Longitude=" + longitude);
        //Log.d(TAG,"Altitude=" + altitude);
        location.distanceBetween(latitude, longitude,targetLatitude,targetLongitude, result);
        targetDistance = result[0];
        targetDirection = result[1];
        if(holdFlag == 0) {
            distance += location.distanceTo(oldLocation);
            velocity = location.getSpeed() * 3600f / 1000f;
            //Log.d(TAG,"TargetDirection=" + targetDirection);
            velocityView.setText(getString(R.string.velocity_format, velocity));
            distanceView.setText(getString(R.string.distance_format, distance));
            googleMap.addMarker(new MarkerOptions().position(new LatLng(latitude, longitude)));
            googleMap.animateCamera(CameraUpdateFactory.newLatLng(new LatLng(latitude, longitude)));
            mileageView.setText(getString(R.string.distance_format, targetDistance));
            mileageBar.setProgress((int) (1000 * targetDistance / maxDistance));

        }
        oldLocation = location;


        //Log.d(TAG,"TargetDistance=" + targetDistance + "m");

    }

    private void startLocationUpdate(boolean reqPermission) {
        Log.d(TAG, "startLocationUpdate: " + reqPermission);
        for (String permission : PERMISSIONS) {
            if (ContextCompat.checkSelfPermission(this, permission)
                    != PackageManager.PERMISSION_GRANTED) {
                if (reqPermission)
                    ActivityCompat.requestPermissions(this, PERMISSIONS, REQCODE_PERMISSIONS);
                else
                    Toast.makeText(this, getString(R.string.toast_requires_permission, permission),
                            Toast.LENGTH_SHORT).show();
                return;
            }
        }
        LocationServices.FusedLocationApi.requestLocationUpdates(googleApiClient, locationRequest, this);
        state = UpdatingState.STARTED;
        oldLocation = LocationServices.FusedLocationApi.getLastLocation(googleApiClient);
        clickReset();
    }

    private void stopLocationUpdate() {
        Log.d(TAG, "stopLocationUpdate");
        LocationServices.FusedLocationApi.removeLocationUpdates(googleApiClient, this);
        state = UpdatingState.STOPPED;
    }

    private void initSettings() {

    }

    private void clickStart() {
        Log.d(TAG,"Hold is called");
        startColor = Color.DKGRAY;
        holdColor = Color.GREEN;
        resetColor = Color.RED;
        startTime = System.currentTimeMillis();
        holdFlag = 0;
        mTXPacket[0] = START_POINT;
        mTXPacket[1] = 'A';
        mTXPacket[2] = END_POINT;
        mBluetoothService.write(mTXPacket);
    }

    private void clickHold() {
        Log.d(TAG,"Hold is called");
        startColor = Color.CYAN;
        holdColor = Color.DKGRAY;
        resetColor = Color.RED;
        holdFlag = 1;
    }

    private void clickReset() {
        Log.d(TAG,"Reset is called");
        float[] result = new float[3];
        startColor = Color.CYAN;
        holdColor = Color.DKGRAY;
        resetColor = Color.DKGRAY;
        velocity = 0f;
        distance = 0f;
        time = 0f;
        if(oldLocation != null) {
            Log.d(TAG,"oldLocation is not null");
            googleMap.animateCamera(CameraUpdateFactory.newLatLng(new LatLng(oldLocation.getLatitude(), oldLocation.getLongitude())));
            oldLocation.distanceBetween(oldLocation.getLatitude(), oldLocation.getLongitude(),targetLatitude,targetLongitude, result);
            maxDistance = result[0];
        }

        startButton.setBackgroundColor(startColor);
        holdButton.setBackgroundColor(holdColor);
        resetButton.setBackgroundColor(resetColor);
    }

    private void clickCalculate() {
        Log.d(TAG,"Calculate is called");
        slopeOffset = -attitudeDegree[1];
    }

    private  void clickConnect() {
        Log.d(TAG, "Scan is called");
        Intent intent = new Intent(this, DeviceListActivity.class);
        startActivityForResult(intent, REQUEST_CONNECT_DEVICE_SECURE);
    }

    private void process(byte[] packet) {
        mRXPacket = packet;
        if(mRXPacket[0] == START_POINT) {
            mTXPacket[0] = START_POINT;
            mTXPacket[2] = END_POINT;
            mBluetoothService.write(mTXPacket);
            Log.d(TAG, "Receive Packet" + mRXPacket);
            Log.d(TAG, "Send Packet" + mTXPacket);
        }
    }
}
