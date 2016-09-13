// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  pinMode(obLED, OUTPUT);
  pinMode(jsBut, INPUT);
  pinMode(jsSupplyVcc , OUTPUT);
  digitalWrite(jsSupplyVcc ,HIGH);
  pinMode(jsSupplyVss , OUTPUT);
  digitalWrite(jsSupplyVss , LOW);

  // attache each servo to it's pin
  for (int i = 0; i < qn; i++) 
    servos[i].attach(servoPin[i]);
  

  //Startup variables
  long currentTime = millis();
  lUserInputScan_L = currentTime;
  lUserSerialInfoScan_L = currentTime;
  ijsCoordSys = 0;
  lKineReCalc_L = currentTime;


  //Calculate velocity and acceleration constants
  fMaxManualJointFeedPerServoCycle = fMaxManualJointVelocity * lUpdateServos / 1000;
  fMaxManualLinFeedPerServoCycle = fMaxManualLinVelocity * lUpdateServos / 1000;
  fMaxManualRotFeedPerServoCycle = fMaxManualRotVelocity * lUpdateServos / 1000;
  fManualJointFeedVariationPerServoCycle = fManualJointAccel *6.0  * lUpdateServos / 1000;
  fManualLinFeedVariationPerServoCycle = fManualLinAccel * lUpdateServos / 1000; 
  fManualRotFeedVariationPerServoCycle = fManualRotAccel *6.0  * lUpdateServos / 1000;


  // 
  ijsCoordSys =0;
  ijsHighestAxInd = qn;
  ijsAxisControlled[0] = 1;

  delay(1500); //Delay 1,5sec before changing the USB serial settings
  Serial.begin(iBaudRate);
  Serial3.begin(iBaudRateSerial3);
  Serial.println("Booting Up!");
  //Serial.println((String)Timer.getAvailable());
  delay(1500);

  //SetupTFT();


#ifdef ServiceServoUpdateWithInterrupt
  // Setup timer and interrupt to cyclicly update servos
  Timer7.attachInterrupt(InturruptBasedServo).start(lUpdateServos*1000); //1000 factor to conver mSec into uSec
#endif

 
}

// the loop function runs over and over again forever
void loop(){
  currentTime = millis();


  //should be before recalc Kinematics and before updating servo positions
  //Recalc End effector position if joints changed position
  if (!EndEffectorUpToDate)
  {
    CalculateEndEffector();
  }

  // Recalc Kinematics
  if (abs(currentTime - lKineReCalc_L) >= lKineReCalc) {
    //Serial.println("Effective Kine Cycle=" + String(currentTime - lKineReCalc_L));
    lKineReCalc_L = currentTime;
    CalculateEndEffTransfMatrix();
    CalculateJacobian();
    InvertJacobian();
    CalculateDelta();
  }
  
  // Update servo positions if this function is not interrupt driven
#ifndef ServiceServoUpdateWithInterrupt
  if (abs(currentTime - lUpdateServos_L) >= lUpdateServos) {
    lUpdateServos_L = currentTime;
    UpdateServoPositions();
  }
#endif

  // Service user interface
  if (abs(currentTime - lUserInputScan_L) >= lUserInputScan) {
    lUserInputScan_L = currentTime;
    ScanUInputs();
  }

    // Transmit user info on serial port
#ifndef FreeComForDebug
  if (abs(currentTime - lUserSerialInfoScan_L) >= lUserSerialInfoScan) {
    lUserSerialInfoScan_L = currentTime;
    UpdateUserInfoOnSerial();
  }
#endif

}

