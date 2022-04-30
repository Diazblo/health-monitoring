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

void pox_task(void *pvParameters)
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

    pox.setup(); // Configure sensor. Use 6.4mA for LED drive

    for (;;)
    {
        irValue = pox.getIR();

        if (checkForBeat(irValue) == true)
        {
            // We sensed a beat!
            long delta = millis() - lastBeat;
            lastBeat = millis();

            beatsPerMinute = 60 / (delta / 1000.0);

            if (beatsPerMinute < 255 && beatsPerMinute > 20)
            {
                rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
                rateSpot %= RATE_SIZE;                    // Wrap variable

                // Take average of readings
                beatAvg = 0;
                for (byte x = 0; x < RATE_SIZE; x++)
                    beatAvg += rates[x];
                beatAvg /= RATE_SIZE;
            }
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
    // while(1){
    //     Serial.println(analogRead(ECG_PIN));
    //     delay(100);
    // }

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
        // for (uint8_t bb = 0; bb < 20; bb++)
        // {
            events.send("ping", NULL, millis());
            events.send(getSensorReadings().c_str(), "new_readings", millis());
        // }
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

// void handle_OnConnect()
// {

//     // server.send(200, "text/html", SendHTML(temperature, humidity, BPM, SpO2, bodytemperature));
// }

// void handle_NotFound()
// {
//     // server.send(404, "text/plain", "Not found");
// }
/*
String SendHTML(float temperature, float humidity, float BPM, float SpO2, float bodytemperature)
{
    String ptr = "<!DOCTYPE html>";
    ptr += "<html>";
    ptr += "<script src=\"https://cdnjs.cloudflare.com/ajax/libs/Chart.js/2.9.4/Chart.js\"></script>";
    ptr += "<head>";
    ptr += "<title>Smart Health Syrveillance System</title>";
    ptr += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    ptr += "<link href='https://fonts.googleapis.com/css?family=Open+Sans:300,400,600' rel='stylesheet'>";
    ptr += "<style>";
    ptr += "html { font-family: 'Open Sans', sans-serif; display: block; margin: 0px auto; text-align: center;color: #444444;}";
    ptr += "body{margin: 0px;} ";
    ptr += "h1 {margin: 50px auto 30px;} ";
    ptr += ".side-by-side{display: table-cell;vertical-align: middle;position: relative;}";
    ptr += ".text{font-weight: 600;font-size: 19px;width: 200px;}";
    ptr += ".reading{font-weight: 300;font-size: 50px;padding-right: 25px;}";
    ptr += ".temperature .reading{color: #F29C1F;}";
    ptr += ".humidity .reading{color: #3B97D3;}";
    ptr += ".BPM .reading{color: #FF0000;}";
    ptr += ".SpO2 .reading{color: #955BA5;}";
    ptr += ".bodytemperature .reading{color: #F29C1F;}";
    ptr += ".superscript{font-size: 17px;font-weight: 600;position: absolute;top: 10px;}";
    ptr += ".data{padding: 10px;}";
    ptr += ".container{display: table;margin: 0 auto;}";
    ptr += ".icon{width:65px}";
    ptr += "</style>";
    ptr += "</head>";
    ptr += "<body>";
    ptr += "<h1>Smart Health Syrveillance System</h1>";
    ptr += "<h3>SSJCOE</h3>";
    ptr += "<div class='container'>";

    ptr += "<div class='data temperature'>";
    ptr += "<div class='side-by-side icon'>";
    ptr += "<svg enable-background='new 0 0 19.438 54.003'height=54.003px id=Layer_1 version=1.1 viewBox='0 0 19.438 54.003'width=19.438px x=0px xml:space=preserve xmlns=http://www.w3.org/2000/svg xmlns:xlink=http://www.w3.org/1999/xlink y=0px><g><path d='M11.976,8.82v-2h4.084V6.063C16.06,2.715,13.345,0,9.996,0H9.313C5.965,0,3.252,2.715,3.252,6.063v30.982";
    ptr += "C1.261,38.825,0,41.403,0,44.286c0,5.367,4.351,9.718,9.719,9.718c5.368,0,9.719-4.351,9.719-9.718";
    ptr += "c0-2.943-1.312-5.574-3.378-7.355V18.436h-3.914v-2h3.914v-2.808h-4.084v-2h4.084V8.82H11.976z M15.302,44.833";
    ptr += "c0,3.083-2.5,5.583-5.583,5.583s-5.583-2.5-5.583-5.583c0-2.279,1.368-4.236,3.326-5.104V24.257C7.462,23.01,8.472,22,9.719,22";
    ptr += "s2.257,1.01,2.257,2.257V39.73C13.934,40.597,15.302,42.554,15.302,44.833z'fill=#F29C21 /></g></svg>";
    ptr += "</div>";
    ptr += "<div class='side-by-side text'>Room Temperature</div>";
    ptr += "<div class='side-by-side reading'>";
    ptr += (int)temperature;
    ptr += "<span class='superscript'>&deg;C</span></div>";
    ptr += "</div>";

    ptr += "<div class='data humidity'>";
    ptr += "<div class='side-by-side icon'>";
    ptr += "<svg enable-background='new 0 0 29.235 40.64'height=40.64px id=Layer_1 version=1.1 viewBox='0 0 29.235 40.64'width=29.235px x=0px xml:space=preserve xmlns=http://www.w3.org/2000/svg xmlns:xlink=http://www.w3.org/1999/xlink y=0px><path d='M14.618,0C14.618,0,0,17.95,0,26.022C0,34.096,6.544,40.64,14.618,40.64s14.617-6.544,14.617-14.617";
    ptr += "C29.235,17.95,14.618,0,14.618,0z M13.667,37.135c-5.604,0-10.162-4.56-10.162-10.162c0-0.787,0.638-1.426,1.426-1.426";
    ptr += "c0.787,0,1.425,0.639,1.425,1.426c0,4.031,3.28,7.312,7.311,7.312c0.787,0,1.425,0.638,1.425,1.425";
    ptr += "C15.093,36.497,14.455,37.135,13.667,37.135z'fill=#3C97D3 /></svg>";
    ptr += "</div>";
    ptr += "<div class='side-by-side text'>Room Humidity</div>";
    ptr += "<div class='side-by-side reading'>";
    ptr += (int)humidity;
    ptr += "<span class='superscript'>%</span></div>";
    ptr += "</div>";

    ptr += "<div class='data Heart Rate'>";
    ptr += "<div class='side-by-side icon'>";
    ptr += "<svg enable-background='new 0 0 40.542 40.541'height=40.541px id=Layer_1 version=1.1 viewBox='0 0 40.542 40.541'width=40.542px x=0px xml:space=preserve xmlns=http://www.w3.org/2000/svg xmlns:xlink=http://www.w3.org/1999/xlink y=0px><g><path d='M34.313,20.271c0-0.552,0.447-1,1-1h5.178c-0.236-4.841-2.163-9.228-5.214-12.593l-3.425,3.424";
    ptr += "c-0.195,0.195-0.451,0.293-0.707,0.293s-0.512-0.098-0.707-0.293c-0.391-0.391-0.391-1.023,0-1.414l3.425-3.424";
    ptr += "c-3.375-3.059-7.776-4.987-12.634-5.215c0.015,0.067,0.041,0.13,0.041,0.202v4.687c0,0.552-0.447,1-1,1s-1-0.448-1-1V0.25";
    ptr += "c0-0.071,0.026-0.134,0.041-0.202C14.39,0.279,9.936,2.256,6.544,5.385l3.576,3.577c0.391,0.391,0.391,1.024,0,1.414";
    ptr += "c-0.195,0.195-0.451,0.293-0.707,0.293s-0.512-0.098-0.707-0.293L5.142,6.812c-2.98,3.348-4.858,7.682-5.092,12.459h4.804";
    ptr += "c0.552,0,1,0.448,1,1s-0.448,1-1,1H0.05c0.525,10.728,9.362,19.271,20.22,19.271c10.857,0,19.696-8.543,20.22-19.271h-5.178";
    ptr += "C34.76,21.271,34.313,20.823,34.313,20.271z M23.084,22.037c-0.559,1.561-2.274,2.372-3.833,1.814";
    ptr += "c-1.561-0.557-2.373-2.272-1.815-3.833c0.372-1.041,1.263-1.737,2.277-1.928L25.2,7.202L22.497,19.05";
    ptr += "C23.196,19.843,23.464,20.973,23.084,22.037z'fill=#26B999 /></g></svg>";
    ptr += "</div>";
    ptr += "<div class='side-by-side text'>Heart Rate</div>";
    ptr += "<div class='side-by-side reading'>";
    ptr += (int)BPM;
    ptr += "<span class='superscript'>BPM</span></div>";
    ptr += "</div>";

    ptr += "<div class='data Blood Oxygen'>";
    ptr += "<div class='side-by-side icon'>";
    ptr += "<svg enable-background='new 0 0 58.422 40.639'height=40.639px id=Layer_1 version=1.1 viewBox='0 0 58.422 40.639'width=58.422px x=0px xml:space=preserve xmlns=http://www.w3.org/2000/svg xmlns:xlink=http://www.w3.org/1999/xlink y=0px><g><path d='M58.203,37.754l0.007-0.004L42.09,9.935l-0.001,0.001c-0.356-0.543-0.969-0.902-1.667-0.902";
    ptr += "c-0.655,0-1.231,0.32-1.595,0.808l-0.011-0.007l-0.039,0.067c-0.021,0.03-0.035,0.063-0.054,0.094L22.78,37.692l0.008,0.004";
    ptr += "c-0.149,0.28-0.242,0.594-0.242,0.934c0,1.102,0.894,1.995,1.994,1.995v0.015h31.888c1.101,0,1.994-0.893,1.994-1.994";
    ptr += "C58.422,38.323,58.339,38.024,58.203,37.754z'fill=#955BA5 /><path d='M19.704,38.674l-0.013-0.004l13.544-23.522L25.13,1.156l-0.002,0.001C24.671,0.459,23.885,0,22.985,0";
    ptr += "c-0.84,0-1.582,0.41-2.051,1.038l-0.016-0.01L20.87,1.114c-0.025,0.039-0.046,0.082-0.068,0.124L0.299,36.851l0.013,0.004";
    ptr += "C0.117,37.215,0,37.62,0,38.059c0,1.412,1.147,2.565,2.565,2.565v0.015h16.989c-0.091-0.256-0.149-0.526-0.149-0.813";
    ptr += "C19.405,39.407,19.518,39.019,19.704,38.674z'fill=#955BA5 /></g></svg>";
    ptr += "</div>";
    ptr += "<div class='side-by-side text'>Blood Oxygen</div>";
    ptr += "<div class='side-by-side reading'>";
    ptr += (int)SpO2;
    ptr += "<span class='superscript'>%</span></div>";
    ptr += "</div>";

    ptr += "<div class='data Body Temperature'>";
    ptr += "<div class='side-by-side icon'>";
    ptr += "<svg enable-background='new 0 0 19.438 54.003'height=54.003px id=Layer_1 version=1.1 viewBox='0 0 19.438 54.003'width=19.438px x=0px xml:space=preserve xmlns=http://www.w3.org/2000/svg xmlns:xlink=http://www.w3.org/1999/xlink y=0px><g><path d='M11.976,8.82v-2h4.084V6.063C16.06,2.715,13.345,0,9.996,0H9.313C5.965,0,3.252,2.715,3.252,6.063v30.982";
    ptr += "C1.261,38.825,0,41.403,0,44.286c0,5.367,4.351,9.718,9.719,9.718c5.368,0,9.719-4.351,9.719-9.718";
    ptr += "c0-2.943-1.312-5.574-3.378-7.355V18.436h-3.914v-2h3.914v-2.808h-4.084v-2h4.084V8.82H11.976z M15.302,44.833";
    ptr += "c0,3.083-2.5,5.583-5.583,5.583s-5.583-2.5-5.583-5.583c0-2.279,1.368-4.236,3.326-5.104V24.257C7.462,23.01,8.472,22,9.719,22";
    ptr += "s2.257,1.01,2.257,2.257V39.73C13.934,40.597,15.302,42.554,15.302,44.833z'fill=#F29C21 /></g></svg>";
    ptr += "</div>";
    ptr += "<div class='side-by-side text'>Body Temperature</div>";
    ptr += "<div class='side-by-side reading'>";
    ptr += (int)bodytemperature;
    ptr += "<span class='superscript'>&deg;C</span></div>";
    ptr += "</div>";


    ptr += "<canvas id=\"myChart\" style=\"width:100%;max-width:600px\"></canvas>";
    ptr += "<script>";
    ptr += "var xValues = [50,60,70,80,90,100,110,120,130,140,150];";
    ptr += "var yValues = [7,8,8,9,9,9,10,11,14,14,15];";
    ptr +=
    "new Chart(\"myChart\", {\
    type: \"line\",\
    data: {\
        labels: xValues,\
        datasets: [{\
        fill: false,\
        lineTension: 0,\
        backgroundColor: \"rgba(0,0,255,1.0)\",\
        borderColor: \"rgba(0,0,255,0.1)\",\
        data: yValues\
        }]\
    },\
    options: {\
        legend: {display: false},\
        scales: {\
        yAxes: [{ticks: {min: 6, max:16}}],\
        }\
    }\
    });\
    </script>";

    ptr += "</div>";
    ptr += "</body>";
    ptr += "</html>";
    return ptr;
}
*/