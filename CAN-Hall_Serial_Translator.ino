#include <mcp_can_2.h>
#include <SPI.h>
#include <math.h>

#define BITS_PER_BYTE 8
#define CAN_INT 2    // Set INT to pin 2

#define M_TWOPI (2 * M_PI)

#define X 0
#define Y 1
#define Z 2
#define XY 0
#define XZ 1
#define YZ 2

typedef union
{
  int number;
  uint8_t bytes[2];
} INT_UNION;

typedef union
{
  uint16_t number;
  uint8_t bytes[2];
} UINT_UNION;

typedef struct _sensor_data {

  uint16_t group_id;
  bool axis_set[3];

  long last_send;

  // Raw sensor value
  INT_UNION raw[3];

  //  // Gauss value
  //  float mx;
  //  float my;
  //  float mz;
  //
  //  // radian values
  //  float rx;
  //  float ry;
  //  float rz;
  //
  //  // Use a four quadrant Arc Tan to convert 2
  //  // axis to an angle (which is in radians) then
  //  // convert the angle from radians to degrees
  //  // for display.
  //  float angleXY;
  //  float angleXZ;
  //  float angleYZ;

  uint8_t i2c_error;

} SENSOR_DATA;

SENSOR_DATA data[32] = {0};

void setup()
{
  Serial.begin(250000);
  Serial.println("Serial Ready!");
  for (int i = 10; i <= 13; i++) {
    pinMode(i, INPUT);
  }
  if (CAN.begin(CAN_500KBPS) == CAN_OK) { // init can bus : baudrate = 500k
    Serial.println("CAN Ready!");
  }

}

void loop()
{
  if (!digitalRead(CAN_INT))  // The CAN interrupt pin is active LOW
  {
    // CAN RX Variables
    long unsigned int rxId;
    unsigned char len;
    unsigned char rxBuf[8];
    // Serial Output String Buffer
    char msgString[128];

    CAN.readMsgBuf(&rxId, &len, rxBuf);              // Read data: len = data length, buf = data byte(s)
    int last_digit = rxId % 10;
    int sensor_index = rxId - 1;
    // Only read from valid IDs
    if (rxId >= 1 && rxId <= 32) {

      //      // Print the ID
      //      sprintf(msgString, "0x%.3lX", rxId);
      //      Serial.print(msgString);
      //
      //      // Print the buffer contents
      //      for (byte i = 0; i < len; i++) {
      //        sprintf(msgString, " 0x%.2X", rxBuf[i]);
      //        Serial.print(msgString);
      //      }
      //
      //      Serial.println();

      UINT_UNION group_id;
      group_id.bytes[0] = rxBuf[0];
      group_id.bytes[1] = rxBuf[0];
      uint8_t axis = rxBuf[2];
      SENSOR_DATA* this_data = &data[sensor_index];

      // If this is a new group number, reset the axes
      if (group_id.number != data[sensor_index].group_id) {
        data[sensor_index].group_id = group_id.number;
        for (int i = 0; i < 3; i++) {
          this_data->axis_set[i] = false;
        }
      }

      // Save the reading into the raw values array
      this_data->raw[axis].bytes[0] = rxBuf[3];
      this_data->raw[axis].bytes[1] = rxBuf[4];
      this_data->axis_set[axis] = true;

      // Check to see if we have all the axes from this particular reading
      bool data_complete = true;
      for (int i = 0; i < 3; i++) {
        if (this_data->axis_set[i] == false) {
          data_complete = false;
        }
      }

      // If the data group is complete, do something useful with it
      if (data_complete) {


        float gauss[3];
        float rads[3];
        float angle[3];


        for (int i = 0; i < 3; i++) {
          // Look at the datasheet for the sensitivity of the part used.
          // In this case, full scale range is 500 gauss
          // Sensitivity of 500 gauss = 4.0 lsb/g
          gauss[i] = (float)this_data->raw[i].number / 4.0;  // 4.0
          rads[i] = (float)this_data->raw[i].number / 4096.0 * M_TWOPI;
        }

        angle[XY] = atan2f(rads[Y], rads[X]) * 180.0 / M_PI;
        angle[XZ] = atan2f(rads[Z], rads[X]) * 180.0 / M_PI;
        angle[YZ] = atan2f(rads[Z], rads[Y]) * 180.0 / M_PI;

        //        Serial.print("\t1: ");
        //        Serial.print(data[sensor_index].raw[X].bytes[0]);
        //        Serial.print("\t2: ");
        //        Serial.println(data[sensor_index].raw[X].bytes[1]);

        if (abs(data[sensor_index].raw[X].number) >= 3 ||
            abs(data[sensor_index].raw[Y].number) >= 3 ||
            abs(data[sensor_index].raw[Z].number) >= 3) {

          //          Serial.print("ID: ");
          Serial.print(rxId);
          Serial.print(",");
          Serial.print(angle[XZ]);

          Serial.print(",");
          Serial.print(data[sensor_index].raw[X].number);
          Serial.print(",");
          Serial.print(data[sensor_index].raw[Y].number);
          Serial.print(",");
          Serial.println(data[sensor_index].raw[Z].number);
          //
          //          Serial.print("\tXY: ");
          //          Serial.print(angle[XY]);
          //          Serial.print("\tXZ: ");
          //          Serial.print(angle[XZ]);
          //          Serial.print("\tYZ: ");
          //          Serial.println(angle[YZ]);
          //
          //          Serial.print(",");
          //          Serial.print(angle[XY]);

          //          Serial.print(",");
          //          Serial.println(angle[YZ]);
          for (int i = 0; i < 3; i++) {
            this_data->axis_set[i] = false;
          }
//          long update_time = micros() - data[sensor_index].last_send;
//          Serial.println(update_time);
//          data[sensor_index].last_send = micros();
        }
      }


    }
  }
}
