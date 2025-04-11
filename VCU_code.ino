#include <Ch376msc.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <LoRa.h>
#include <CAN.h>

#define TX_GPIO_NUM 17;
#define RX_GPIO_NUM 16;

#define HSPI_MOSI 13;
#define HSPI_MISO 12;
#define HSPI_SCK 14;
#define HSPI_CS 15;
#define RST 34;
#define DI0 2;

#define buzzer 4;
#define RTD_LED 5;
#define pwm 23;
#define testled 40;
#define apps_led 25; 
#define bspd_led 21;

#define apps1_pin 26;
#define apps2_pin 27;
#define BPS_pin 2;
#define CS_pin 35;
#define RTDB_pin 36;
#define bps_scs_pin 18;
#define ECU_SCS_pin 19;
#define Air_State_pin 22;

// Thresholds 12-bit ADC [Formula => (v*4096)/3.3]
// #define lt1 1550//1500.00
// #define ht1 3376//2340.00
// #define lt2 900//850.00
// #define ht2 1936//1553.80
#define bps_th 100.00;

#define lt1 1735;
#define ht1 2650;
#define lt2 942;
#define ht2 1453;

const int filterSize = 5;  // Filter size adjustment
int buffer[filterSize];
int bufferIndex = 0;

// float canData[8] = {0.0, 2.3, 2.9, 1.5, 1.6, 1.9, 1.8, 0.7}; // Initialize array to store CAN data
// uint8_t byteValue[8] = {100, 200, 255, 230, 150, 75, 63, 20};

int rtdmode = 0, brakefault = 0, flag1 = 0, appsfault = 0;
int mapped_apps;
int rtdb_count = 0;

SoftwareSerial mySerial(33, 32); // RX, TX pins on ESP32
Ch376msc flashDrive(mySerial);

void PWM_init();

// TaskHandle_t Task1;
// TaskHandle_t Task2;

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

    // mySerial.begin(9600);
    mySerial.begin(115200);
    flashDrive.init();
    Serial.print("Initialized CH376");
    delay(1000);
    if (flashDrive.checkIntMessage()) {
        if (flashDrive.getDeviceStatus()) {
            Serial.println(F("Flash drive attached!"));
        } else {
            Serial.println(F("Flash drive detached!"));
        }
        delay(1000);
    }
    //createAndWriteFile();

    // Serial.println("LoRa Sender");
    // SPI.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, HSPI_CS);
    // LoRa.setPins(HSPI_CS, RST, DI0);

    // while (!LoRa.begin(433E6)) {
    //     Serial.println("LoRa initialization failed. Retrying...");
    //     delay(1000);
    // }
    // LoRa.setSyncWord(0xF1);
    // Serial.println("LoRa Initializing Successful!");

    // Serial.println("CAN Receiver/Receiver");
    // CAN.setPins(RX_GPIO_NUM, TX_GPIO_NUM);
    // if (!CAN.begin(500E3)) {
    //     Serial.println("Starting CAN failed!");
    //     while (1);
    // } else {
    //     Serial.println("CAN Initialized");
    // }
    // xTaskCreatePinnedToCore(
    //                 Task1code,   /* Task function. */
    //                 "Task1",     /* name of task. */
    //                 10000,       /* Stack size of task */
    //                 NULL,        /* parameter of the task */
    //                 1,           /* priority of the task */
    //                 &Task1,      /* Task handle to keep track of created task */
    //                 0);          /* pin task to core 0 */                  
    // delay(500); 

    // //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
    // xTaskCreatePinnedToCore(
    //                   Task2code,   /* Task function. */
    //                   "Task2",     /* name of task. */
    //                   10000,       /* Stack size of task */
    //                   NULL,        /* parameter of the task */
    //                   1,           /* priority of the task */
    //                   &Task2,      /* Task handle to keep track of created task */
    //                   1);          /* pin task to core 1 */
    // delay(500); 
}
// void Task1code( void * pvParameters ){
//   Serial.print("Task1 running on core ");
//   Serial.println(xPortGetCoreID());

//   for(;;){
   
//     //LoRasend();
//     ecuCode();
//     delay(100);
//   } 
// }

//Task2code: blinks an LED every 700 ms
// void Task2code( void * pvParameters ){
//   Serial.print("Task2 running on core ");
//   Serial.println(xPortGetCoreID());

//   for(;;){

//     delay(100);
//   }
// }
void loop() {
  //canReceiver();
  //appendDataToFile();
  ecuCode();
}

void ecuCode() {
  float a, b, apps1, apps2, apps1_analog, apps2_analog;
  int rtdb, bps_scs, ECU_SCS, Air_State, bps, cs;

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
    // Serial.print("Apps1: ");
    apps1 = apps1_analog * 3.6 / 4095;
    // Serial.println(apps1);
    apps2 = apps2_analog * 3.6 / 4095;
    // Serial.print("Apps2: ");
    // Serial.println(apps2);
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
    // Serial.print("Mapped apps value: ");
    // Serial.println(mapped_apps);

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

 void createAndWriteFile() {
    Serial.print("Creating and writing to file: TEST1.CSV");
    flashDrive.setFileName("TEST1.CSV");
    flashDrive.openFile();

    char header[] = "sensor 1, sensor 2, sensor 3, sensor 4, sensor 5, sensor 6, sensor 7, sensor 8\n";
    flashDrive.writeFile(header, strlen(header));

    flashDrive.closeFile();
    Serial.println("Done!");
}

// void appendDataToFile() {
//     Serial.print("Appending data to file: TEST1.CSV");
//     flashDrive.setFileName("TEST1.CSV");
//     if (flashDrive.openFile() == ANSW_USB_INT_SUCCESS) {
//         flashDrive.moveCursor(CURSOREND);

//         char buffer[10];  // Adjust the size as needed

//         for (int i = 0; i < 8; i++) {
//             dtostrf(canData[i], 4, 2, buffer);  // Convert float to string with 2 decimal places
//             flashDrive.writeFile(buffer, strlen(buffer));
//             if (i < 7) {
//                 flashDrive.writeFile(",", 1);
//             }
//         }

//         flashDrive.writeFile("\n", 1);  // Add a newline at the end of the row

//         flashDrive.closeFile();
//         Serial.println("Done Appending!");
//     }
// }

// void LoRasend() {
//   Serial.println("Sending packet");

//   // Send the byte array via LoRa
//   LoRa.beginPacket();
//   LoRa.write(byteValue, sizeof(byteValue));
//   LoRa.endPacket();

//   if (LoRa.endPacket()) {
//     Serial.println("Packet sent successfully");
//   } else {
//     Serial.println("Error sending packet");
//   }

//   delay(100);
// }

// void filter() {
//     unsigned long packetId = CAN.packetId();
//     unsigned long targetId = 0x12;

//     if (packetId == targetId) {
//         Serial.println("Data from Master 0x");
//         Serial.println(packetId, HEX);
//     } else {
//         Serial.print("Data from CAN Node 0x");
//         Serial.println(packetId, HEX);
//     }
// }

// void canReceiver() {
//     int packetSize = CAN.parsePacket();

//     if (packetSize) {
//         Serial.print("Received ");

//         if (CAN.packetExtended()) {
//             Serial.print("extended ");
//         }

//         if (CAN.packetRtr()) {
//             Serial.print("RTR ");
//         }

//         unsigned long packetId = CAN.packetId();

//         if (packetId) {
//             if (!CAN.packetRtr()) {
//                 Serial.print(" and length ");
//                 Serial.println(packetSize);
//                 filter();

//                 for (int i = 0; i < 8; i++) {
//                     byteValue[i] = CAN.read();
//                     canData[i] = (float)byteValue[i] / 76;
//                     Serial.println(canData[i], 2);
//                 }
                
//             }
//         } else {
//             Serial.print(" and requested length ");
//             Serial.println(CAN.packetDlc());
//         }

//         Serial.println();
//     }
// }

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