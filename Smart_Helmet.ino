#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include <EEPROM.h>
#include <DHT.h>
#include <FirebaseESP32.h>

#define FIREBASE_HOST "https://posture-9f0bf-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "5WreA24ZnMvaInr2rrE89eXcKiACsY9oXBhhLQCC"

#define wifi_name "Airtel_9674319250"
#define password "air20400"

// Firebase Initialization
FirebaseData firebaseData;
FirebaseJson json;

// MPU6050
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;
float elapsedTime, currentTime, previousTime;

int mpu_x[15];
int mpu_y[15];

//MQ2 Gas Sensor
const int MQ2 = 34; //34 of NodeMCU is connected to Analog pin of MQ-02
float gas_2;
float calib_2;

//MQ135 Gas Sensor
const int MQ135 = 35; //35 of NodeMCU is connected to Analog pin of MQ-135
float gas_135;
float calib_135;

//DHT11 Sensor
const int DHT11PIN = 32;
float humi, temp;
DHT dht(DHT11PIN, DHT11);

// Helmet Status
int ir = 33; 

void setup()
{
    Serial.begin(115200);
    delay(10);
    Serial.println("Connecting to");
    Serial.print(wifi_name);
    WiFi.begin (wifi_name, password); 
    while(WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to the WiFi network");

    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
    Firebase.reconnectWiFi(true);

    Serial.println("Calibrating Sensors.... ");
    gas_sensor_calibrate();

    // MPU6050
    Wire.begin(21, 22, 100000); // sda, scl
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);

    for(int i=0; i<20; i++){
      mpu_x[i]=rand()*100;
      mpu_y[i]=rand()*100;
    }
    
    dht.begin(); //DHT Sensor
    
    Serial.println(" Calibration and Setup complete");
}

void loop()
{
  //MPU6050
  double Ax, Ay, Az, T, Gx, Gy, Gz;
  double accAngleX, accAngleY, accAngleZ, gyroAngleX, gyroAngleY;
  double pitch, roll, yaw;
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR);
  Wire.requestFrom(MPU_ADDR, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
//  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
//  GyX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
//  GyY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
//  GyZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  Ax = (double)AcX/AccelScaleFactor;
  Ay = (double)AcY/AccelScaleFactor;
  Az = (double)AcZ/AccelScaleFactor;
//  Gx = (double)GyX/GyroScaleFactor;
//  Gy = (double)GyY/GyroScaleFactor;
//  Gz = (double)GyZ/GyroScaleFactor;

  //Calculate Roll, Yaw, Pitch
  accAngleX = (180 * atan (-1*Ax/sqrt(Ay*Ay + Az*Az))/M_PI) + 90;
  accAngleY = (180 * atan (Ay/sqrt(Ax*Ax + Az*Az))/M_PI) + 90;

  Serial.print("Roll = ");
  Serial.print(accAngleX);
  Serial.print("\t");
  Serial.print("pitch = ");
  Serial.println(accAngleY);

  String mpu = check_mpu(accAngleX, accAngleY, 0);
  Firebase.setString(firebaseData, "/Hellmate/MPU", mpu);

  // Helmet Status
  helmet_status();
  
  // MQ2
  mq2();
  Serial.print("\t");

  // MQ 135
  mq_135();
  Serial.println();

  //DHT11 Sensor
  dht11();

  delay(10);
  Serial.println("------");
}

// Calibrate MQ2 and MQ135 Sensor
void gas_sensor_calibrate(){
  calib_2 = 0;
  calib_135 = 0;
  
  for (int i = 1; i<=300; i++){
    calib_2 += analogRead(MQ2);
    calib_135 += analogRead(MQ135);
    delay(10);
    Serial.print(".");
  }
  
  calib_2 /= 300;
  calib_135 /= 300;
  Serial.println();
  Serial.print("Calibrated MQ_2: ");
  Serial.print(calib_2);
  Serial.print("\t");
  Serial.print("Calibrated MQ_135: ");
  Serial.println(calib_135);
}

// Check if person is wearing helmet or not
void helmet_status(){
  String s = "";
  int stat = analogRead(ir);
  Serial.print("Helmet Status = ");
  if (stat < 2000)
    s = "Wearing";
  else
    s = "Not_Wearing";

  Serial.println(s);
  Firebase.setString(firebaseData, "/Hellmate/Status", s);
}

// MPU Manipulation
String check_mpu(int x, int y, int count){
  int tx, ty;
  
  for(int i=1; i<15; i++){
    mpu_x[i-1] = mpu_x[i];
    mpu_y[i-1] = mpu_y[i];
  }
  mpu_x[14] = x;
  mpu_y[14] = y;

  for(int i=1; i<15; i++){
    if ((abs(mpu_x[0] - mpu_x[i]) < 1.5) and (abs(mpu_y[0] - mpu_y[i]) < 1.5)){
      count += 1;
      Serial.println("Count = " + String(count));
    }
    else{
      count = 0;
      Serial.println("Count = " + String(count));
    }
  }
//  Serial.println("Count = " + String(count));
  if (count >= 10)
    return "Dead";
  else
    return "Alive";
}

// Check for safe environment using MQ2
void mq2(){
  gas_2 = analogRead(MQ2);
  String s = "";
  
  if (gas_2 < (calib_2+30))
    s = "Safe_Environment";
    
  else if ((gas_2 > (calib_2+30)) and (gas_2 < (calib_2+50)))
    s = "Moderate_Environment";
  
  else
    s = "Unsafe_Environment";
    
  Serial.println(s); 
  Serial.print("MQ_2: ");
  Serial.print(gas_2); 
  
  Firebase.setString(firebaseData, "/Hellmate/MQ2", s);
}

// Check for alcohol status using MQ135
void mq_135(){
  gas_135 = analogRead(MQ135);
  String s = "";
  
  if (gas_135 < (calib_135+10))
    s = "Non_Alcoholic";
  
  else
    s = "Alcoholic";
    
  Serial.println(s); 
  Serial.print("MQ_135: ");
  Serial.print(gas_135);
  
  Firebase.setString(firebaseData, "/Hellmate/MQ135", s);
}

// Get environment temperature and Humidity
void dht11(){
  humi = dht.readHumidity();
  temp = dht.readTemperature();
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.print("ºC ");
  Serial.print("Humidity: ");
  Serial.println(humi);
  
  Firebase.setFloat(firebaseData, "/Hellmate/Humidity", humi);
  Firebase.setFloat(firebaseData, "/Hellmate/Temperature", temp);
}
