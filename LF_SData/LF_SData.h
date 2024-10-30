#ifndef LF_SDATA_H
#define LF_SDATA_H

#include <Arduino.h>
#include <EEPROM.h>

class LF_SData {
public:
    void setupDistanceSensor(uint8_t distancePin);
    void setupLineSensors(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3, uint8_t sig);
    void calibrateSensors(bool fullCalibration);
    void getLiveSerialPrint(bool printSensors, bool printDistance);
    long getLinePosition();
    int16_t getDistance();

private:
    uint8_t distanceSensorPin;
    uint8_t S0, S1, S2, S3, SIG;
    const int numSensors = 16;
    int sensorValues[16];
    long sensorMin[16];
    long sensorMax[16];
    long total;
    long weightedSum;
    long max_value;
    long lastLinePosition = 0;
    bool serialInitialized = false;
    const int noLineThreshold = 150;  // Threshold for determining no line detected

    const int positionValues[16] = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 11000, 12000, 13000, 14000, 15000};

    int readMultiplexer(int channel);
    void loadCalibration();
    void saveCalibration();
    void initializeSerial();
    bool isSerialConnected();
};

void LF_SData::setupDistanceSensor(uint8_t distancePin) {
    distanceSensorPin = distancePin;
    pinMode(distanceSensorPin, INPUT);
}

void LF_SData::setupLineSensors(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3, uint8_t sig) {
    S0 = s0;
    S1 = s1;
    S2 = s2;
    S3 = s3;
    SIG = sig;

    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);

    digitalWrite(S0, LOW);
    digitalWrite(S1, LOW);
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
}

void LF_SData::calibrateSensors(bool fullCalibration) {
    if (fullCalibration) {
        // Initialize calibration values
        for (int i = 0; i < numSensors; i++) {
            sensorMin[i] = 1023;
            sensorMax[i] = 0;
        }

        // Perform calibration for 5 seconds
        initializeSerial();
        Serial.println("Calibrating...");
        unsigned long startTime = millis();
        while (millis() - startTime < 5000) {
            for (int i = 0; i < numSensors; i++) {
                sensorValues[i] = readMultiplexer(i);
                sensorMin[i] = min(sensorMin[i], sensorValues[i]);
                sensorMax[i] = max(sensorMax[i], sensorValues[i]);
            }
            delay(10);
        }
        Serial.println("Calibration complete");

        saveCalibration();
    } else {
        loadCalibration();
    }
}

void LF_SData::loadCalibration() {
    for (int i = 0; i < numSensors; i++) {
        EEPROM.get(i * sizeof(long), sensorMin[i]);
        EEPROM.get((i + numSensors) * sizeof(long), sensorMax[i]);
    }
}

void LF_SData::saveCalibration() {
    for (int i = 0; i < numSensors; i++) {
        EEPROM.put(i * sizeof(long), sensorMin[i]);
        EEPROM.put((i + numSensors) * sizeof(long), sensorMax[i]);
    }
}

void LF_SData::getLiveSerialPrint(bool printSensors, bool printDistance) {
    total = 0;
    weightedSum = 0;

    if (!serialInitialized && isSerialConnected()) {
        initializeSerial();
    }

    if (serialInitialized) {
        bool lineDetected = false;

        if (printSensors) {
            // Print values
            Serial.print("| ");
            for (int i = 0; i < numSensors; i++) {
                sensorValues[i] = readMultiplexer(i);
                long normalizedValue = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
                weightedSum += normalizedValue * positionValues[i];
                total += normalizedValue;

                if (sensorValues[i] > noLineThreshold) {
                    lineDetected = true;
                }

                Serial.print(sensorValues[i]);
                Serial.print(" | ");
            }

            long linePosition = getLinePosition();
            if (!lineDetected) {
                linePosition = lastLinePosition > 8000 ? 15000 : 0;
            } else {
                lastLinePosition = linePosition;
            }
            Serial.print(" pos = ");
            Serial.print(linePosition);
            Serial.print(" | ");
        }

        if (printDistance) {
            int16_t distance = getDistance();
            Serial.print("Distance: ");
            Serial.print(distance);
            Serial.print(" mm");
        }

        Serial.println();
    }
}

long LF_SData::getLinePosition() {
    total = 0;
    weightedSum = 0;
    max_value = 0;

    for (int i = 0; i < numSensors; i++) {
        sensorValues[i] = readMultiplexer(i);
        long normalizedValue = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
        weightedSum += normalizedValue * positionValues[i];
        total += normalizedValue;
        max_value = max(max_value, sensorValues[i]);
    }

    long linePosition = (total > 0) ? weightedSum / total : -1;
    if (max_value <= 150 || linePosition == -1) {
        // No line detected
        linePosition = lastLinePosition > 8000 ? 15000 : 0;
    } else {
        lastLinePosition = linePosition;
    }

    return linePosition;
}

int16_t LF_SData::getDistance() {
    int16_t t = pulseIn(distanceSensorPin, HIGH);

    if (t == 0) {
        // pulseIn() did not detect the start of a pulse within 1 second.
        if (!serialInitialized) {
            initializeSerial();
        }
        Serial.println("timeout");
        return -1;
    } else if (t > 1850) {
        // No detection.
        return -1;
    } else {
        // Valid pulse width reading. Convert pulse width in microseconds to distance in millimeters.
        int16_t d = (t - 1000) * 3 / 4;

        // Limit minimum distance to 0.
        if (d < 0) {
            d = 0;
        }

        return d;
    }
}

int LF_SData::readMultiplexer(int channel) {
    int controlPin[] = { S0, S1, S2, S3 };

    int muxChannel[16][4] = {
        { 0, 0, 0, 0 },  //channel 0
        { 1, 0, 0, 0 },  //channel 1
        { 0, 1, 0, 0 },  //channel 2
        { 1, 1, 0, 0 },  //channel 3
        { 0, 0, 1, 0 },  //channel 4
        { 1, 0, 1, 0 },  //channel 5
        { 0, 1, 1, 0 },  //channel 6
        { 1, 1, 1, 0 },  //channel 7
        { 0, 0, 0, 1 },  //channel 8
        { 1, 0, 0, 1 },  //channel 9
        { 0, 1, 0, 1 },  //channel 10
        { 1, 1, 0, 1 },  //channel 11
        { 0, 0, 1, 1 },  //channel 12   
        { 1, 0, 1, 1 },  //channel 13
        { 0, 1, 1, 1 },  //channel 14
        { 1, 1, 1, 1 }   //channel 15
    };  

    for (int i = 0; i < 4; i++) {
        digitalWrite(controlPin[i], muxChannel[channel][i]);
    }

    return analogRead(SIG);
}

void LF_SData::initializeSerial() {
    Serial.begin(115200);
    serialInitialized = true;
}

bool LF_SData::isSerialConnected() {
    return Serial;  // Returns true if Serial is connected
}

#endif // LF_SDATA_H
