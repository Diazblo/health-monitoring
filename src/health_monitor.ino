#include <Arduino.h>

#define REPORTING_PERIOD_MS 1000
uint32_t tsLastReport = 0;

//********************************************
#include <Wire.h>
#include "MAX30105.h"
MAX30105 pox;

#include "heartRate.h"
const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
long irValue;

#include "spo2_algorithm.h"
int32_t bufferLength;    // data length
int32_t spo2;            // SPO2 value
int8_t validSPO2;        // indicator to show if the SPO2 calculation is valid
int32_t heartRate;       // heart rate value
int8_t validHeartRate;   // indicator to show if the heart rate calculation is valid
uint32_t irBuffer[100];  // infrared LED sensor data
uint32_t redBuffer[100]; // red LED sensor data

TaskHandle_t pox_task_handle;
//********************************************

//********************************************
#include <Adafruit_Sensor.h>
#include "DHT.h"
#define DHTPIN 18 // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

// TaskHandle_t DHT11_task_handle;
//********************************************

//********************************************
#define ECG_PIN 34
#define ECG_LOM_PIN 25
#define ECG_LOP_PIN 33
#define ECG_SDN_PIN 32

// TaskHandle_t DHT11_task_handle;
//********************************************

//********************************************

#include <OneWire.h>
#include <DallasTemperature.h>

// GPIO where the DS18B20 is connected to
const int oneWireBus = 23;

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature DS18B20(&oneWire);

// TaskHandle_t DALLAS_task_handle;
//********************************************

//********************************************
#include <WiFi.h>
#include <WebServer.h>

float temperature, humidity, BPM, SpO2, bodytemperature;

/*Put your SSID & Password*/
const char *ssid = " Palkar";        // Enter SSID here
const char *password = "Aasdv9196?"; // Enter Password here
// WebServer server(80);

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"

AsyncWebServer server(80);
AsyncEventSource events("/events");

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 500;

#include <Arduino_JSON.h>
// Json Variable to Hold Sensor Readings
JSONVar readings;

//********************************************

TaskHandle_t print_task_handle;

void print_task(void *pvParameters)
{
    for (;;)
    {
        // Print variables
        Serial.print(analogRead(ECG_PIN));

        Serial.print(F(", Hum="));
        Serial.print(humidity);

        Serial.print(F(", temp="));
        Serial.print(temperature);

        Serial.print(F(", Btemp="));
        Serial.print(bodytemperature);

        Serial.print(F(", HR="));
        Serial.print(heartRate, DEC);

        Serial.print(F(", HRvalid="));
        Serial.print(validHeartRate, DEC);

        Serial.print(F(", SPO2="));
        Serial.print(spo2, DEC);

        Serial.print(F(", SPO2Valid="));
        Serial.println(validSPO2, DEC);

        delay(500);
    }
}

void MAX30102_task(void *pvParameters)
{
    Serial.print("Initializing pulse oximeter..");
    if (!pox.begin(Wire, I2C_SPEED_FAST))
    {
        Serial.println(F("MAX30105 was not found. Please check wiring/power."));
        while (1)
            ;
    }
    else
    {
        Serial.println("SUCCESS");
        // pox.setOnBeatDetectedCallback(onBeatDetected);
    }

    byte ledBrightness = 60; // Options: 0=Off to 255=50mA
    byte sampleAverage = 4;  // Options: 1, 2, 4, 8, 16, 32
    byte ledMode = 2;        // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    byte sampleRate = 100;   // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 411;    // Options: 69, 118, 215, 411
    int adcRange = 4096;     // Options: 2048, 4096, 8192, 16384

    pox.setup(); // Configure sensor with these settings

    for (;;)
    {
        bufferLength = 100; // buffer length of 100 stores 4 seconds of samples running at 25sps

        // read the first 100 samples, and determine the signal range
        for (byte i = 0; i < bufferLength; i++)
        {
            while (pox.available() == false) // do we have new data?
                pox.check();                 // Check the sensor for new data

            redBuffer[i] = pox.getRed();
            irBuffer[i] = pox.getIR();
            pox.nextSample(); // We're finished with this sample so move to next sample
            // Serial.print(F("red="));
            // Serial.print(redBuffer[i], DEC);
            // Serial.print(F(", ir="));
            // Serial.println(irBuffer[i], DEC);
        }

        // calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
        maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

        // Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
        while (1)
        {
            // dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
            for (byte i = 25; i < 100; i++)
            {
                redBuffer[i - 25] = redBuffer[i];
                irBuffer[i - 25] = irBuffer[i];
            }

            // take 25 sets of samples before calculating the heart rate.
            for (byte i = 75; i < 100; i++)
            {
                while (pox.available() == false) // do we have new data?
                    pox.check();                 // Check the sensor for new data

                // digitalWrite(readLED, !digitalRead(readLED)); // Blink onboard LED with every data read

                redBuffer[i] = pox.getRed();
                irBuffer[i] = pox.getIR();
                pox.nextSample(); // We're finished with this sample so move to next sample
            }

            // After gathering 25 new samples recalculate HR and SP02
            maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
        }
        delay(1);
    }
}

String getSensorReadings()
{
    readings["sensor1"] = String(analogRead(ECG_PIN));
    readings["sensor2"] = String(temperature);
    readings["sensor3"] = String(BPM);
    readings["sensor4"] = String(SpO2);

    String jsonString = JSON.stringify(readings);
    return jsonString;
}

void setup()
{
    Serial.begin(115200);
    delay(100);

    initSPIFFS();

    // ECG *******************************************
    pinMode(ECG_PIN, INPUT);
    pinMode(ECG_LOM_PIN, INPUT);
    pinMode(ECG_LOP_PIN, INPUT);
    pinMode(ECG_SDN_PIN, INPUT);

    Serial.println("Connecting to ");
    Serial.println(ssid);

    // connect to your local wi-fi network
    WiFi.begin(ssid, password);

    // check wi-fi is connected to wi-fi network
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected..!");
    Serial.print("Got IP: ");
    Serial.println(WiFi.localIP());

    // Web Server *******************************************
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/index.html", "text/html"); });

    server.serveStatic("/", SPIFFS, "/");

    // Request for the latest sensor readings
    server.on("/readings", HTTP_GET, [](AsyncWebServerRequest *request)
              {
    String json = getSensorReadings();
    request->send(200, "application/json", json);
    json = String(); });

    events.onConnect([](AsyncEventSourceClient *client)
                     {
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000); });
    server.addHandler(&events);

    // Start server
    server.begin();

    // DHT11 *******************************************
    dht.begin();

    // DS18B20 *******************************************
    DS18B20.begin();

    // MAX30102 *******************************************
    xTaskCreate(MAX30102_task, "pox_task", 5000, NULL, 1, &pox_task_handle);
    configASSERT(pox_task_handle);
    delay(3000);

    xTaskCreate(print_task, "print_task", 5000, NULL, 1, &print_task_handle);
}

void loop()
{
    if ((millis() - lastTime) > timerDelay)
    {
            events.send("ping", NULL, millis());
            events.send(getSensorReadings().c_str(), "new_readings", millis());
        lastTime = millis();
    }

    // Assign all read values to server variables
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
    BPM = constrain(heartRate, -1, 300);
    SpO2 = constrain(spo2, -1, 100);

    DS18B20.requestTemperatures();
    bodytemperature = DS18B20.getTempCByIndex(0);

    delay(50);
}

// Initialize SPIFFS
void initSPIFFS()
{
    if (!SPIFFS.begin())
    {
        Serial.println("An error has occurred while mounting SPIFFS");
    }
    else
    {
        Serial.println("SPIFFS mounted successfully");
    }
}