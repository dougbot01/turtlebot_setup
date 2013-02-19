/* 
 * rosserial ADC Voltage Example
 * 
 * This can be used to return analog voltage values from Arduino
 * as a ROS message.
 */

#include <ros.h>
#include <turtlebot_node/LaptopChargeStatus.h>

ros::NodeHandle nh;
turtlebot_node::LaptopChargeStatus batt_msg;
ros::Publisher p("turtlebot_lipo", &batt_msg);

float Vcc;
float Vmax = 16.8;
float Vmin = 15.0;
float Vmul = 4.2983192926;
float Icap = 5.0;
float prevCharge;
bool firstRun;

// This code is from the SecretVoltmeter project. See
// http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
float readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return (result/1000.0);
}

// Average 10 analog readings to elminate some of the noise
// and normalize the reading from 0.0 to 1.0
float averageAnalog(int pin){
  int v=0;
  for(int i=0; i<10; i++) v+= analogRead(pin);
  return (v/1023.0)/10.0;
}

void setup()
{ 
  // Initialize the ROS node and advertise the message
  nh.initNode();
  nh.advertise(p);
  
  firstRun = true;
}

void loop()
{
  // Read the supply voltage
  Vcc = readVcc();
  
  // Add a time stamp for the message header
  batt_msg.header.stamp = nh.now();
  
  // Assign the voltages to the message
  // Multiply the average by Vcc to get volts
  batt_msg.present = true;
  batt_msg.charge_state = batt_msg.DISCHARGING;
  batt_msg.capacity = Icap;
  batt_msg.design_capacity = Icap;

  batt_msg.voltage = averageAnalog(0)*Vcc*Vmul;
  batt_msg.charge = Icap*(batt_msg.voltage-Vmin)/(Vmax-Vmin);
  batt_msg.percentage = 100 *(batt_msg.charge/Icap);

  if (firstRun) {
    batt_msg.rate = -1.0;
    firstRun = false;
  } else {
    batt_msg.rate = batt_msg.charge - prevCharge;
  }
  prevCharge = batt_msg.charge;
  
  // Publish the ROS message  
  p.publish(&batt_msg);
  nh.spinOnce();
  
  // Delay 1 second between messages
  delay(1000);
}

