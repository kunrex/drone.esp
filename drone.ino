#include <Wire.h>
#include <BLEUtils.h>
#include <BLEDevice.h>
#include <BLEServer.h>

constexpr const char* device_name = "david.drone";

constexpr std::string_view service_id = "12345678-1234-5678-1234-56789abcdef0";
constexpr std::string_view input_characteristic_id = "abcdef01-1234-5678-1234-56789abcdef0";

constexpr int MPU = 0x68;

constexpr unsigned int min_pwm = 0;
constexpr unsigned int max_pwm = 255;

constexpr unsigned int thrust_threshold = 20;

constexpr float epsilon = 1e-6;

typedef void (*Delegate)(const float(&)[4]);

class DroneCallBack : public BLECharacteristicCallbacks 
{
    private:
        const Delegate onCallDelegate;

    public:
        DroneCallBack(const Delegate onCallDelegate) : onCallDelegate(onCallDelegate) { }

    void onWrite(BLECharacteristic* const characteristic) 
    {
        float output[4];
        
        const auto value = characteristic->getValue();
        if (value.length() == sizeof(float) * 4) 
        {
            memcpy(output, value.c_str(), sizeof(output));
            onCallDelegate(output);
        }
    }
};

struct PIDWrapper 
{
    public:
        const float kp;
        const float ki;
        const float kd;

        PIDWrapper(const float kp, const float ki, const float kd) : kp(kp), ki(ki), kd(kd) { }
};

class Axis 
{
    private:
        float error;
        const PIDWrapper wrapper;
  
    public:
        Axis(float kp, float ki, float kd) : error(0), wrapper(kp, ki, kd) { }

        float calculate(const float input, const float sensor, const float dt) 
        {
            const auto e = input - sensor;
            return error = wrapper.kp * error + wrapper.ki * error * dt + wrapper.kd * e / (dt + epsilon);
        }
};

constexpr unsigned int scl = 6, sda = 7;
constexpr unsigned int front_right = 8, front_left = 9, back_right = 3, back_left = 2;
constexpr unsigned int stby1 = 10, stby2 = 1;

Axis roll(10, .1, .01), pitch(10, 1., .01), yaw(25, 1, .01);

float inRoll, inPitch, inYaw, inThrust;
float actRoll, actPitch;

float kRoll = 1, kPitch = 1, kYaw = 1;

unsigned long then;

void input(const float(&data)[4]) 
{
    inRoll = data[0];
    inPitch = data[1];
    inYaw = data[2];
    inThrust = data[3];
}

void motorBrake(const unsigned int motor)
{
    analogWrite(motor, 0);
}

void motorDrive(const unsigned int motor, const float motorSpeed)
{
    analogWrite(motor, (int)constrain(motorSpeed, min_pwm, max_pwm));
}

DroneCallBack callBack(input);

void setup() 
{
    BLEDevice::init(device_name);
    
    const auto server = BLEDevice::createServer();
    const auto service = server->createService(service_id.data());

    const auto characteristic = service->createCharacteristic(input_characteristic_id.data(), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    characteristic->setCallbacks(&callBack);
    service->start();
    
    const auto advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(service_id.data());
    advertising->setScanResponse(true);
    advertising->start();

    Wire.begin(scl, sda);

    // MPU6050 is on sleep mode by default
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU);
    Wire.write(0x1C);// connects to ACCEL_CONFIG register (1C hex)
    Wire.write(0x10);// sets register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU);
    Wire.write(0x1B);// connects to the GYRO_CONFIG register (1B hex)
    Wire.write(0x10);// sets the register bits as 00010000 (1000deg/s full scale)
    Wire.endTransmission(true);

    pinMode(front_right, OUTPUT);
    pinMode(front_left, OUTPUT);
    pinMode(back_right, OUTPUT);
    pinMode(back_left, OUTPUT);

    pinMode(stby1, OUTPUT);
    pinMode(stby2, OUTPUT);

    then = millis();
}

void sensorIn(const float dt) 
{
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);// connects to the accerlation out (3B hex)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // reads 6 registers, 2 each which correspond to accelerometer data along one axis. 
    
    //for a range of +-2g, we need to divide the raw values by 4096 (according to the datasheet)
    const auto accX = (Wire.read() << 8 | Wire.read()) / 4096.0;
    const auto accY = (Wire.read() << 8 | Wire.read()) / 4096.0;
    const auto accZ = (Wire.read() << 8 | Wire.read()) / 4096.0; 

    const auto accAngleX = (atan((accY / sqrt(pow(accX, 2) + pow(accZ, 2) + epsilon))) * 180 / PI) - 0.58; // error in accX is aprox 0.58 
    const auto accAngleY = (atan(-1 * (accX / sqrt(pow(accY, 2) + pow(accZ, 2) + epsilon))) * 180 / PI) + 1.58; // err in accY is aprox -1.58

    Wire.beginTransmission(MPU);
    Wire.write(0x43); // connects to the gyroscope out (43 hex)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 4, true); // reads 6 registers, 2 each which correspond to gyroscope data along one axis. Ignore z.

    // for a range of 10000deg/s, we have to divide raw values by 32.8 (according to the datasheet)
    const auto gyroX = (Wire.read() << 8 | Wire.read()) / 32.8 + 0.56; // error in gyroX is aprox -0.56
    const auto gyroY = (Wire.read() << 8 | Wire.read()) / 32.8 - 2; // error in gyroX is aprox 2

    // raw values are in degrees per seconds, so convert to degrees
    const auto gyroAngleX = actRoll + gyroX * dt;
    const auto gyroAngleY = actPitch + gyroY * dt;
      
    actRoll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    actPitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
}

void loop() 
{
    const auto now = millis();
    const auto dt = (now - then) / 1000.f;

    sensorIn(dt);

    const auto outRoll = kRoll * roll.calculate(inRoll, actRoll, dt);
    const auto outPitch = kPitch * pitch.calculate(inPitch, actPitch, dt);
    const auto outYaw = kYaw * yaw.calculate(inYaw, 0, dt);

    if (inThrust < thrust_threshold) 
    {
        motorBrake(front_right);
        motorBrake(front_left);
        motorBrake(back_right);
        motorBrake(back_left);

        digitalWrite(stby1, LOW);
        digitalWrite(stby2, LOW);
    }
    else 
    {
        digitalWrite(stby1, HIGH);
        digitalWrite(stby2, HIGH);
      
        motorDrive(front_right, inThrust - outRoll + outPitch + outYaw);
        motorDrive(front_left, inThrust + outRoll + outPitch - outYaw);
        motorDrive(back_right, inThrust - outRoll - outPitch - outYaw);
        motorDrive(back_left, inThrust + outRoll - outPitch + outYaw);
    }

    then = now;
}
