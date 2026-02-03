#include <M5Core2.h>



// --- Global Variables ---
float accX, accY, accZ, gyroX, gyroY, gyroZ;
float pitch, roll, yaw; // Euler Angles
float temp;

// Smoothing (Low Pass Filter)
float alpha = 0.2; // Smoothing factor (0.0 - 1.0). Lower = smoother but more lag.
float f_accX, f_accY, f_accZ; 



void setup() {
    M5.begin(); // Init Core2 (Screen, SD, UART)
    M5.IMU.Init();
    
    M5.Lcd.setTextSize(2);
    M5.Lcd.print("Ready. Press A to Record.");
}

void loop() {
    M5.update();
    
    // 1. Read Raw Data
    M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
    M5.IMU.getAccelData(&accX, &accY, &accZ);

    // 2. Apply Smoothing (Low Pass Filter)
    // Formula: Output = alpha * NewInput + (1-alpha) * OldOutput
    f_accX = alpha * accX + (1.0 - alpha) * f_accX;
    f_accY = alpha * accY + (1.0 - alpha) * f_accY;
    f_accZ = alpha * accZ + (1.0 - alpha) * f_accZ;

    // 3. Convert to Euler Angles (using internal Mahony Filter)
    // The M5 library handles the quaternion math internally here
    M5.IMU.getAhrsData(&pitch, &roll, &yaw);

    // 4. Stream Data to Serial (For Visualization)
    // Format: "Variable:Value" for Arduino Serial Plotter
    Serial.print("AccelX:"); Serial.print(f_accX);
    Serial.print(",Pitch:"); Serial.print(pitch);
    Serial.print(",Roll:"); Serial.println(roll);

    // 5. Check Button for Recording
    if (M5.BtnA.wasPressed()) {
        recordData();
    }
    
    delay(10); // Simple loop pacing
}

void recordData() {
    M5.Lcd.fillScreen(RED);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.print("RECORDING...");
    
    // Create Files
    File imuFile = SD.open("/imu_data.csv", FILE_WRITE);
    
    imuFile.println("timestamp,ax,ay,az,pitch,roll,yaw");

    unsigned long startTime = millis();
    size_t bytesRead;
    uint8_t i2s_read_buff[1024];
    int16_t *samples;
    
    while (millis() - startTime < (RECORD_TIME_SECONDS * 1000)) {
        // --- Capture IMU (Snapshot) ---
        M5.IMU.getAccelData(&accX, &accY, &accZ);
        M5.IMU.getAhrsData(&pitch, &roll, &yaw);
        
        imuFile.print(millis() - startTime); imuFile.print(",");
        imuFile.print(accX); imuFile.print(",");
        imuFile.print(accY); imuFile.print(",");
        imuFile.print(accZ); imuFile.print(",");
        imuFile.print(pitch); imuFile.print(",");
        imuFile.print(roll); imuFile.print(",");
        imuFile.println(yaw);
    }

    imuFile.close();
    
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.print("Saved to SD!");
    delay(1000);
}