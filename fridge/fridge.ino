#include <Servo.h>
#include <PID_v1.h>

#define THERM_PIN 0
#define JAG_PIN 9
#define Kp 1
#define Ki 0
#define Kd 0

Servo jag;  // create servo object to control a servo
const double vref = 5.222; //Reference voltage
const double pullup1 = 99.7*1000; //Pullup resistor 1
const double pullup2 = 99.9*1000; //Pullup resistor 2

double temp1 = 25; //Thermistor 1
double temp2 = 25; //Thermistor 2
double PIDOutput = 0;
double setpoint = 25;
String incomingString = "";
PID tempController(&temp1, &PIDOutput, &setpoint, Kp, Ki, Kd, REVERSE);

long out[] = {150,145,140,135,130,125,120,115,110,105,100,95,90,85,80,75,70,65,60,55,50,45,40,35,30,25,20,15,10,5,0,-5,-10,-15,-20,-25,-30,-35,-40};
long in[] = {1618,1827,2070,2350,2676,3057,3503,4026,4643,5372,6238,7269,8504,9988,11780,13951,16597,19835,23820,28749,34879,42548,52200,64422,80003,100000,125851,159522,203723,262229,340346,445602,588793,785573,1058901,1442861,1988706,2774565,3921252};
const int ARR_LEN = 39;

void setup()
{
  jag.attach(JAG_PIN);  //Jag is connected to a pin
  tempController.SetMode(AUTOMATIC);
  Serial.begin(19200);
  Serial.println("Startup");
}

void loop()
{
  if (temp2 <= 50)
  {
    tempController.Compute();
    
    incomingString = "";
    while (Serial.available() > 0)
      incomingString += (char)(Serial.read());
      
    if(incomingString.substring(0, 4) == "SET:")
    {
      setpoint = incomingString.substring(4).toInt();
      Serial.println("Setpoint set at "+String(setpoint));
    }
    else if(incomingString != "")
      Serial.println("INVALID COMMAND!");
    
    temp1 = readTherm(0, 40, pullup1, vref);
    temp2 = readTherm(1, 40, pullup2, vref);
    
    //jag.write(map(constrain(PIDOutput, -100, 100), -100, 100, -180, 180));
    jag.write(180);
    
    Serial.print("Cold: "+String(temp1)+", "+String((analogRead(0)/1023.0)*vref));
    Serial.print("\t");
    Serial.print("Hot: "+String(temp2)+", "+String((analogRead(1)/1023.0)*vref));
    Serial.print("\t");
    Serial.println(jag.read());
  }
  else
  {
    temp2 = readTherm(1, 40, pullup2, vref);
    Serial.println("!!TEMPERATURE TOO HIGH!! ("+String(temp2)+")");
    jag.write(90);
  }
}

int readTherm(int port, int samples, double pullup_res, double v_ref)
{
  long tempTotal = 0;
  
  for(int C = 0;C < samples;C++)
  {
    tempTotal += (pullup_res*(v_ref-((analogRead(port)/1023.0)*v_ref)))/((analogRead(port)/1023.0)*v_ref);
    delay(1);
  }
  long avg = tempTotal/(double)(samples);
  return multiMap((long)(avg), in, out, ARR_LEN);
}

int multiMap(long val, long* _in, long* _out, uint8_t sizee)
{
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]){Serial.print("VALUE TOO SMALL:  "); Serial.println(String(val)+" <= "+String(_in[0])); return _out[0];}
  if (val >= _in[sizee-1]){Serial.print("VALUE TOO LARGE  "); Serial.println(String(val)+" >= "+String(_in[0])); return _out[sizee-1];}

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}
