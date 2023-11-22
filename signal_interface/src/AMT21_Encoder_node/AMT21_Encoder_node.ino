#include <ros.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle nh;

sensor_msgs::JointState joint_state_msg;
ros::Publisher joint_state_pub("joint_states", &joint_state_msg);

/* Serial rates for UART */
#define USB_BAUDRATE         115200
#define RS485_BAUDRATE       115200

/* We will use this define macro so we can write code once compatible with 12 or 14 bit encoders */
#define RESOLUTION            14


#define RS485_RESET           0x75
#define RS485_ZERO            0x5E
#define RS485_ENC1            0x40
#define RS485_ENC2            0x44
#define RS485_ENC3            0x48
#define RS485_ENC4            0x50
#define RS485_ENC5            0x54

#define RS485_POS             0x00 //this is unnecessary to use but it helps visualize the process for using the other modifiers
#define RS485_TURNS           0x01
#define RS485_EXT             0x02

#define RS485_T_RE            8
#define RS485_T_DE            9
#define RS485_T_DI            18
#define RS485_T_RO            19

/* For ease of reading, we will create a helper function to set the mode of the transceiver. We will send that funciton
 * these arguments corresponding with the mode we want.
 */
#define RS485_T_TX            0 //transmit: receiver off, driver on
#define RS485_T_RX            1 //receiver: driver off, transmit on
#define RS485_T_2             2 //unused
#define RS485_T_3             3 //unused


void setup()
{
  //Set the modes for the RS485 transceiver
  pinMode(RS485_T_RE, OUTPUT);
  pinMode(RS485_T_DE, OUTPUT);
  pinMode(RS485_T_RO, INPUT_PULLUP); //enable pull-up on RX pin so that it is not left floating when the transceiver is in transmit-mode

  //Initialize the UART serial connection to the PC
  Serial.begin(USB_BAUDRATE);

  //Initialize the UART link to the RS485 transceiver
  Serial1.begin(RS485_BAUDRATE);

  // Initialize ROS
  nh.initNode();
  nh.advertise(joint_state_pub);

  // Initialize JointState message
  joint_state_msg.name_length = 5; // For 2 joints
  joint_state_msg.position_length = 5;
  joint_state_msg.name = new char*[5]{"joint_1", "joint_2", "joint_3", "joint_4", "joint_5"};
  joint_state_msg.position = new float[5];

}

void loop()
{
  //create an array of encoder addresses so we can use them in a loop
  uint8_t addresses[5] = {RS485_ENC1, RS485_ENC2, RS485_ENC3, RS485_ENC4, RS485_ENC5};

  for(int encoder = 0; encoder < sizeof(addresses); ++encoder)
  {
    //clear whatever is in the read buffer in case there's nonsense in it
    while (Serial1.available()) Serial1.read();

    setStateRS485(RS485_T_TX); //put the transciver into transmit mode
    delayMicroseconds(10);   //IO operations take time, let's throw in an arbitrary 10 microsecond delay to make sure the transeiver is ready

    //send the command to get position. All we have to do is send the node address, but we can use the modifier for consistency
    Serial1.write(addresses[encoder] | RS485_POS);

    //With an AMT21xE (115.2 kbps), we expect a response from the encoder to begin after 11 microseconds. Each byte sent has a start and stop bit,
    //so each 8-bit byte transmits 10 bits total. So for the AMT21 operating at 115.2 kbps, transmitting the full 20 bit response will take about 174µs.
    //We expect the response to start after 11µs totalling 185 microseconds from the time we've finished sending data. So we need to put the transceiver
    //into receive mode within 11µs, but we want to make sure the data has been fully transmitted before we do that or we could cut it off mid transmission.
    //This code has been tested and optimized for this; porting this code to another device must take all this timing into account.

    //Here we will make sure the data has been transmitted and then toggle the pins for the transceiver
    //Here we are accessing the avr library to make sure this happens very fast. We could use Serial.flush() which waits for the output to complete
    //but it takes about 2 microseconds, which gets pretty close to our 3 microsecond window. Instead we want to wait until the serial transmit flag USCR1A completes.
    while (!(UCSR1A & _BV(TXC1)));

    setStateRS485(RS485_T_RX); //set the transceiver back into receive mode for the encoder response

    //We need to give the encoder enough time to respond, but not too long. In a tightly controlled application we would want to use a timeout counter
    //to make sure we don't have any issues, but for this demonstration we will just have an arbitrary delay before checking to see if we have data to read.
    delayMicroseconds(40000000/RS485_BAUDRATE);
    
    //Response from encoder should be exactly 2 bytes
    int bytes_received = Serial1.available();
    if (bytes_received == 2)
    {
      uint16_t currentPosition = Serial1.read(); //low byte comes first
      currentPosition |= Serial1.read() << 8;    //high byte next, OR it into our 16 bit holder but get the high bit into the proper placeholder

      if (verifyChecksumRS485(currentPosition))
      {
        //we got back a good position, so just mask away the checkbits
        currentPosition &= 0x3FFF;

        float currentPositionRadians = (currentPosition / 16383.0) * (2 * PI);

        // Assign the current position in radians to the joint_state_msg for this encoder
        joint_state_msg.position[encoder] = currentPositionRadians;

        Serial.print("Encoder #");
        Serial.print(encoder, DEC);
        Serial.print(" position in radians: ");
        //Serial.print(" position: ");
        Serial.println(currentPositionRadians, 3); //print the position in radians with 3 decimal places
        //Serial.println(currentPosition, DEC); //print the position in decimal format
      }
      else
      {
        Serial.print("Encoder #");
        Serial.print(encoder, DEC);
        Serial.println(" error: Invalid checksum.");
      }
    }
    else
    {
      Serial.print("Encoder #");
      Serial.print(encoder, DEC);
      Serial.print(" error: Expected to receive 2 bytes. Actually received ");
      Serial.print(bytes_received, DEC);
      Serial.println(" bytes.");
    }

    //flush the received serial buffer just in case anything extra got in there
    while (Serial1.available()) Serial1.read();

    // Publish the JointState message after both positions are updated
    joint_state_pub.publish(&joint_state_msg);
    
    // Handle ROS communications
    nh.spinOnce();
  }

  //For the purpose of this demo we don't need the position returned that quickly so let's wait a half second between reads
  delay(1);
}

bool verifyChecksumRS485(uint16_t message)
{
  //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent.
  //checksum is invert of XOR of bits, so start with 0b11, so things end up inverted
  uint16_t checksum = 0x3;
  for(int i = 0; i < 14; i += 2)
  {
    checksum ^= (message >> i) & 0x3;
  }
  return checksum == (message >> 14);
}

/*
 * This function sets the state of the RS485 transceiver. We send it that state we want. Recall above I mentioned how we need to do this as quickly
 * as possible. To be fast, we are not using the digitalWrite functions but instead will access the avr io directly. I have shown the direct access
 * method and left commented the digitalWrite method.
 */
void setStateRS485(uint8_t state)
{
  //switch case to find the mode we want
  switch (state)
  {
    case RS485_T_TX:
      PORTH |= 0b01100000;
      //digitalWrite(RS485_RE, HIGH); //ph5
      //digitalWrite(RS485_DE, HIGH); //ph6
      break;
    case RS485_T_RX:
      PORTH &= 0b10011111;
      //digitalWrite(RS485_RE, LOW);
      //digitalWrite(RS485_DE, LOW);
      break;
    case RS485_T_2:
      PORTH = (PORTH & 0b11011111) | 0b01000000;
      //digitalWrite(RS485_RE, LOW);
      //digitalWrite(RS485_DE, HIGH);
      break;
    case RS485_T_3:
    default:
      PORTH = (PORTH & 0b10111111) | 0b00100000;
      //digitalWrite(RS485_RE, HIGH);
      //digitalWrite(RS485_DE, LOW);
      break;
  }
}