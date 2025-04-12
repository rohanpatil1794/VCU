#include <SPI.h>
#include <CAN.h>
// SD
#include <SD.h>

#define TX_GPIO_NUM 17
#define RX_GPIO_NUM 16

#define HSPI_MOSI 13
#define HSPI_MISO 12
#define HSPI_SCK 14
#define HSPI_CS 32
#define RST 34
#define DI0 2

#define buzzer 4
#define RTD_LED 5
#define pwm 23
#define testled 40 
#define apps_led 25 
#define bspd_led 21 

#define apps1_pin 26
#define apps2_pin 27
#define BPS_pin 2
#define CS_pin 35
#define RTDB_pin 36
#define bps_scs_pin 18
#define ECU_SCS_pin 19
#define Air_State_pin 22

#define VCU_FILE "vcu.csv"
#define BUFFER_SIZE 128

// Thresholds 12-bit ADC [Formula => (v*4096)/3.3]
// #define lt1 1550//1500.00
// #define ht1 3376//2340.00
// #define lt2 900//850.00
// #define ht2 1936//1553.80
#define bps_th 100.00

#define lt1 1735
#define ht1 2650
#define lt2 942
#define ht2 1453

const int filterSize = 5;  // Filter size adjustment
int buffer[filterSize];
int bufferIndex = 0;

int rtdmode = 0, brakefault = 0, flag1 = 0, appsfault = 0;
int mapped_apps;
int rtdb_count = 0;

float a = 0.0f;
float b = 0.0f;
float apps1_analog = 0.0f;
float apps2_analog = 0.0f;
int bps = 0;
int cs = 0;

// Tasks
TaskHandle_t SD_Task = NULL;

void PWM_init();
void sdTask(void * pvParameters);
void appendVcuDataToSdFile();


void setup() {
  Serial.begin(115200);

  pinMode(buzzer, OUTPUT);
  pinMode(RTD_LED, OUTPUT);
  pinMode(pwm, OUTPUT);
  pinMode(testled, OUTPUT);
  pinMode(apps_led, OUTPUT);
  pinMode(bspd_led, OUTPUT);
  pinMode(apps1_pin, INPUT);
  pinMode(apps2_pin, INPUT);
  pinMode(BPS_pin, INPUT);
  pinMode(CS_pin, INPUT);
  analogReadResolution(12); // Set ADC resolution to 12-bit
  PWM_init();

  // Initialize SPI
  SPI.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, HSPI_CS);

  // Initialize SD card
  if (!SD.begin(HSPI_CS)) {
      Serial.println("Card failed, or not present");
      return;
  }
  Serial.println("SD card initialized.");

  xTaskCreatePinnedToCore(
                  sdTask,   /* USB and LoRa task */
                  "SD_Task",     /* name of task. */
                  10000,       /* Stack size of task */
                  NULL,        /* parameter of the task */
                  1,           /* priority of the task */
                  &SD_Task,      /* Task handle to keep track of created task */
                  0);          /* pin task to core 0 */
}

void loop() {
  ecuCode();
}

void ecuCode() {
  float apps1, apps2; 
  int rtdb, bps_scs, ECU_SCS, Air_State;

  rtdb = analogRead(RTDB_pin);
  bps = analogRead(BPS_pin);
  cs = analogRead(CS_pin);
  bps_scs = digitalRead(bps_scs_pin);
  ECU_SCS = digitalRead(ECU_SCS_pin);
  Air_State = analogRead(Air_State_pin);

  //Print the states of each pin
  // Serial.print("RTDB Pin State: ");
  // Serial.println(rtdb);
  // Serial.print("BPS Pin State: ");
  // Serial.println(bps);
  Serial.print("CS Pin State: ");
  Serial.println(cs);
  // Serial.print("ECU_SCS Pin State: ");
  // Serial.println(ECU_SCS);
  // Serial.print("Air_State Pin State: ");
  // Serial.println(Air_State);
    // Serial.println("RTDB = ");
    // Serial.println(rtdb);

  // if (bps >= bps_th && rtdb > 3000 && ECU_SCS == 1 && Air_State == 0 && rtdmode == 0 && brakefault == 0 && appsfault == 0) { // pending - bps scs check
  if (rtdmode == 0 && bps >= bps_th && rtdb > 3000) { 
    unsigned long startTime = millis();
    rtdb_count = 0;

    while (millis() - startTime <= 2000) { 
      digitalWrite(buzzer, HIGH);
    }

    digitalWrite(buzzer, LOW);
    Serial.println("Buzz... Buzz... Buzz...");
    rtdmode = 1;
  }

  if (rtdmode) { 
    rtdb_count++;
    digitalWrite(RTD_LED, HIGH);
    // Serial.println("RTD Mode Entered");
    bps = analogRead(BPS_pin);
    apps1_analog = analogRead(apps1_pin);
    apps2_analog = analogRead(apps2_pin);
    Serial.print("Apps1: ");
    apps1 = apps1_analog * 3.6 / 4095;
    Serial.println(apps1);
    apps2 = apps2_analog * 3.6 / 4095;
    Serial.print("Apps2: ");
    Serial.println(apps2);
    // Normalization, Range[0 to 1]
    a = ((float)(apps1_analog - lt1) / (float)(ht1 - lt1));
    b = ((float)(apps2_analog - lt2) / (float)(ht2 - lt2));
    // Serial.print("a: ");
    // Serial.println(a);
    // Serial.print("b: ");
    // Serial.println(b);

    mapped_apps = map(apps2_analog, lt2, ht2, 0, 255);
    
    // Store the current mapped value in the buffer
    // buffer[bufferIndex] = mapped_apps;
    // bufferIndex = (bufferIndex + 1) % filterSize;  // Increment the buffer index cyclically
    
    // Apply the median filter
    //int filteredValue = medianFilter(buffer, filterSize);
    
    analogWrite(pwm, mapped_apps);
    // analogWrite(pwm, filteredValue);
    Serial.print("Mapped apps value: ");
    Serial.println(mapped_apps);

    // ECU SCS check condition
    // while (ECU_SCS == 0) {
    //   ECU_SCS = digitalRead(ECU_SCS_pin);
    //   analogWrite(pwm, 0);
    //   digitalWrite(RTD_LED, LOW);
    //   rtdmode = 0;
    //   Serial.println("ECU SCS Check Failed... Exiting RTD Mode");
    //  }

    //  AIR state condition
    // while (Air_State >= 3000) {
    //   Air_State = analogRead(Air_State_pin);
    //   analogWrite(pwm, 0);
    //   digitalWrite(RTD_LED, LOW);
    //   rtdmode = 0;
    //   Serial.println("Air State High... Exiting RTD Mode");

    // }

    // BPS SCS check condition
    // while (bps_scs == 0) {
    //   bps_scs = digitalRead(bps_scs_pin);
    //   analogWrite(pwm, 0);
    //   digitalWrite(RTD_LED, LOW);
    //   rtdmode = 0;
    //   Serial.println("BPS SCS Check Failed... Exiting RTD Mode");
    // }

    // Brake fault condition
    // if (a > 0.25 && bps >= bps_th && brakefault == 0 && rtdmode == 1) {
    //   digitalWrite(bspd_led, HIGH);
    //   unsigned long startTime = millis();
    //   brakefault == 1;
    //   rtdmode = 0;
    //   Serial.println("BPS fault... BSPD LED On");

      // while (brakefault == 1) {
      //   // bps = analogRead(BPS_pin);
      //   // apps1_analog = analogRead(apps1_pin);
      //   // a = ((float)(apps1_analog - lt1) / (float)(ht1 - lt1));
      //   // b = ((float)(apps2_analog - lt2) / (float)(ht2 - lt2));

      //   Serial.println("Value a ");
      //   Serial.println(a);
      //   Serial.println("BPS value ");
      //   Serial.println(bps);
      //   // if (millis() - startTime < 2000 && ((a < 0.25) || bps < bps_th)) { // 500ms plausibility check
      //   if((a < 0.25) || bps < bps_th){
      //     digitalWrite(RTD_LED, HIGH);
      //     digitalWrite(bspd_led, LOW);
      //     brakefault = 0;
      //     Serial.println("RTD Mode Re-entered");
      //   }
      //   // if (millis() - startTime > 2000) {
      //     else{
      //     analogWrite(pwm, 0);
      //     digitalWrite(RTD_LED, LOW);
      //     Serial.println("Exiting RTD Mode");
      //     brakefault = 1;
      //     flag1 = 1;
      //   }
      // }

      // while (flag1 == 1) {
      //   apps1_analog = analogRead(apps1_pin);
      //   apps2_analog = analogRead(apps2_pin);
      //   digitalWrite(RTD_LED, LOW);
      //   Serial.println("RTD LED Off");
      //   rtdmode = 0;
      //   a = ((float)(apps1_analog - lt1) / (float)(ht1 - lt1));
      //   b = ((float)(apps2_analog - lt2) / (float)(ht2 - lt2));
      //   if (a > 0.05 || b > 0.05) {
      //     analogWrite(pwm, 0);
      //   }
      //   else if (a < 0.05 || b < 0.05) {
      //     analogWrite(pwm, map(apps1_analog, lt1, ht1, 0, 255));
      //     digitalWrite(RTD_LED, HIGH);
      //     digitalWrite(bspd_led, LOW);
      //     flag1 = 0;
      //     Serial.println("BSPD LED Off");
      //     Serial.println("RTD Mode Re-entered");
      //   }
      // }
    // }

    // APPS Fault condition
    //if ((abs(a - b) > 0.2 && rtdmode == 1) || apps1<0.81 || apps1>1.43 || apps2<0.75 || apps2>1.5) { // 10% implausibility
    // if ((abs(a - b) > 0.2 && rtdmode == 1)) { // 10% implausibility

    //   unsigned long startTime = millis(); 
    //   appsfault = 1;
    //   while (appsfault == 1) {
    //     apps1_analog = analogRead(apps1_pin);
    //     apps2_analog = analogRead(apps2_pin);
    //     digitalWrite(apps_led, HIGH);
    //     a = ((float)(apps1_analog - lt1) / (float)(ht1 - lt1));
    //     b = ((float)(apps2_analog - lt2) / (float)(ht2 - lt2));
    //     Serial.print("a: ");
    //     Serial.print(a);
    //     Serial.print("b: ");
    //     Serial.print(b);
    //     if ((millis() - startTime <= 500 && (abs(a - b) < 0.2)) && a>0 && a<1 && b>0 && b<1) { // 100ms condition
    //       digitalWrite(apps_led, LOW);
    //       Serial.println("Exiting APPS Fault... APPS LED Off");
    //       appsfault = 0;
    //     }
    //     if (millis() - startTime > 1000) { // 500ms plausibility check
    //       analogWrite(pwm, 0);
    //       digitalWrite(RTD_LED, LOW);
    //       digitalWrite(apps_led, HIGH);
    //       appsfault = 1;
    //       rtdmode = 0;
    //       Serial.println("Apps fault occurred. RTD LED turned LOW. APPS LED turned HIGH");
    //       delay(1000);
    //     }
    //   }
    // }

    // Mannualy exiting RTD mode
    // delay(5000);
    // Serial.println("RTD Button = ");
    // Serial.println(rtdb);
    if(rtdmode == 1 && rtdb > 3000 && rtdb_count > 1){
      analogWrite(pwm, 0);
      digitalWrite(RTD_LED, LOW);
      Serial.println("Manually exiting RTD Mode.");
      rtdmode = 0;
    }
  }
}


void PWM_init() {
  // For motor controller
  ledcSetup(0, 5000, 8); // Use channel 0, 5000 Hz PWM, 8-bit resolution
  ledcAttachPin(pwm, 0);   
}

int medianFilter(int *values, int size) {
    int tempArray[size];
    
    // Copy the values to the temporary array
    for (int i = 0; i < size; i++) {
        tempArray[i] = values[i];
    }
    
    // Sort the temporary array
    for (int i = 0; i < size - 1; i++) {
        for (int j = i + 1; j < size; j++) {
            if (tempArray[i] > tempArray[j]) {
                int temp = tempArray[i];
                tempArray[i] = tempArray[j];
                tempArray[j] = temp;
            }
        }
    }
    
    // Return the median value
    return tempArray[size / 2];
}


void sdTask(void * pvParameters) {
  Serial.print("usbTask running on core ");
  Serial.println(xPortGetCoreID());
  for(;;){
    appendVcuDataToSdFile();
    vTaskDelay(100);
  }
}


void appendVcuDataToSdFile()
{
  Serial.print("Appending data to file: ");
  Serial.println(VCU_FILE);

  // Open file for writing
  File dataFile = SD.open(VCU_FILE, FILE_APPEND);
  if (!dataFile) {
      Serial.println("Error opening file for writing!");
      return;
  }

  char floatBuffer[BUFFER_SIZE];  // Buffer for storing float values as strings
  char buffer[10];                // Adjust the size as needed

  String floatStr = "";

  unsigned long timeStamp = millis();
  sprintf(buffer, "%lu", timeStamp);

  floatStr += buffer;
  floatStr += ",";

  // Concatenate float values into a single string
  dtostrf(a, 4, 2, buffer);
  floatStr += buffer;
  floatStr += ",";

  dtostrf(b, 4, 2, buffer);
  floatStr += buffer;
  floatStr += ",";

  dtostrf(apps1_analog, 4, 2, buffer);
  floatStr += buffer;
  floatStr += ",";

  dtostrf(apps2_analog, 4, 2, buffer);
  floatStr += buffer;
  floatStr += ",";

  dtostrf(bps, 4, 2, buffer);
  floatStr += buffer;
  floatStr += ",";

  dtostrf(cs, 4, 2, buffer);
  floatStr += buffer;

  floatStr += "\n";  // Add a newline at the end of the row
  // Copy the contents of the const char* buffer into a non-const buffer
  floatStr.toCharArray(floatBuffer, BUFFER_SIZE);

  // Write the concatenated string to the file
  dataFile.write((const uint8_t*)floatBuffer, strlen(floatBuffer));

  dataFile.close();
  Serial.println("Done Appending!");
  return;
}