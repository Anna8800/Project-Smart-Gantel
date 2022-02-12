/* Edge Impulse Arduino */


/* Includes ---------------------------------------------------------------- */
#include <Anna-project-1_inferencing.h>
#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal


BLEService fitnessBLEsense("1826");    // GATT service - Fitness Machine
BLEIntCharacteristic counterChar("2AD3", BLERead | BLENotify);  // GATT characteristic and obect type 0x2ad3 Training status
int counter=4;
//
/**
* @brief      Arduino setup function
*/
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("Edge Impulse Inferencing Demo");

    if (!IMU.begin()) {
        ei_printf("Failed to initialize IMU!\r\n");
    }
    else {
        ei_printf("IMU initialized\r\n");
    }

    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
        ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
        return;
    }

   pinMode(LED_BUILTIN, OUTPUT);

   if (!BLE.begin()) {
      Serial.println("BLE failed to Initiate");
      delay(500);
      while (1);
   }

   // read all the sensor values
   
   BLE.setLocalName("ArduinoFitnessBLEsense");
   BLE.setAdvertisedService(fitnessBLEsense);
   fitnessBLEsense.addCharacteristic(counterChar);
   BLE.addService(fitnessBLEsense);
   counterChar.writeValue(counter); 
   BLE.advertise();

   Serial.println("Bluetooth device is now active, waiting for connections...");

}

/**
* @brief      Printf function uses vsnprintf and output using Arduino Serial
*
* @param[in]  format     Variable argument list
*/
void ei_printf(const char *format, ...) {
   static char print_buf[1024] = { 0 };

   va_list args;
   va_start(args, format);
   int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
   va_end(args);

   if (r > 0) {
       Serial.write(print_buf);
   }
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop()
{
    ei_printf("\nStarting inferencing in 1 seconds...\n");

    delay(1000);

    ei_printf("Sampling...\n");

    // Allocate a buffer here for the values we'll read from the IMU
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
        // Determine the next tick (and then sleep later)
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

        IMU.readAcceleration(buffer[ix], buffer[ix + 1], buffer[ix + 2]);

        buffer[ix + 0] *= CONVERT_G_TO_MS2;
        buffer[ix + 1] *= CONVERT_G_TO_MS2;
        buffer[ix + 2] *= CONVERT_G_TO_MS2;

        delayMicroseconds(next_tick - micros());
    }

    // Turn the raw buffer in a signal which we can the classify
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // print the predictions
    float max_predication=0.0;
    int result_predication=0;
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        if(result.classification[ix].value>max_predication)
           {
            result_predication=ix+1;
            max_predication=result.classification[ix].value;
           }
        ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    }
   Serial.print("result_predication=");Serial.println(result_predication);
   Serial.print("max_predication=");Serial.println  (max_predication);
   BLEDevice central = BLE.central();
   if (central) {
      Serial.print("Connected to central: ");
      Serial.println(central.address());
      digitalWrite(LED_BUILTIN, HIGH);
      if   (central.connected()) {
          if(max_predication>0.7) {
            counter=result_predication;
            counterChar.writeValue(counter); 
            //
            Serial.println("fitness BLEsense data:");
            Serial.print("counter=");Serial.print(counter);
            Serial.println("");   
            //            
          }
      }
   digitalWrite(LED_BUILTIN, LOW);
   Serial.print("Disconnected from central: ");
   }

//}

    
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_ACCELEROMETER
#error "Invalid model for current sensor"
#endif
