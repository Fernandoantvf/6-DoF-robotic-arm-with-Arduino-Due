//Debounces inputs and memorizes last scan for rising or falling edge detection
void DebounceInput(int iInputPin, int &iDebounceCounter, boolean &bInputDebounced, boolean &bInputLastScan) {
  const int iMaxDebounceCount = 2;
  const int iMinDebounceCount = 0;

  //Update debounce counter
  boolean currentPinState = digitalRead(iInputPin);
  if (currentPinState && (iDebounceCounter < iMaxDebounceCount))
    iDebounceCounter++;
  else if (!currentPinState && (iDebounceCounter > iMinDebounceCount))
    iDebounceCounter--;

  //Update last scan boolean value
  bInputLastScan = bInputDebounced;

  //Update debounced input value
  if (iDebounceCounter == iMinDebounceCount)
    bInputDebounced = false;
  if (iDebounceCounter == iMaxDebounceCount)
    bInputDebounced = true;
}

/*
 * Discretizes joystick position as true/false
 */
void ThreshJoyStickUp(float inputValue, boolean &bInputDiscretized, boolean &bInputDiscretizedLS) {
  float fPosThresh = fjsThreshold + fjsHysteresis;
  float fNegThresh = fjsThreshold - fjsHysteresis;
  bInputDiscretizedLS = bInputDiscretized;
  if (inputValue < fNegThresh)
    bInputDiscretized = false;
  if (inputValue > fPosThresh)
    bInputDiscretized = true;
}

/*
 * Discretizes joystick position as true/false
 */
void ThreshJoyStickDown(float inputValue, boolean &bInputDiscretized, boolean &bInputDiscretizedLS) {
  float fPosThresh = - fjsThreshold - fjsHysteresis;
  float fNegThresh = - fjsThreshold + fjsHysteresis;
  bInputDiscretizedLS = bInputDiscretized;
  if (inputValue > fNegThresh)
    bInputDiscretized = false;
  if (inputValue < fPosThresh)
    bInputDiscretized = true;
}



//Scans user inputs and processes them
void ScanUInputs() {
  DebounceInput(jsBut, ijsButDebounce, jsButDB, jsButLS);


  //Read joystick position, center and apply dead range
  ijsPosition[0] = analogRead(jsX) - ijsNeutralPosition;
  ijsPosition[1] = ijsNeutralPosition - analogRead(jsY);
  for (int i=0; i<2; i++){
    if (ijsPosition[i] < 0.0){
      ijsPosition[i] = ijsPosition[i] + ijsDeadRange;
      if (ijsPosition[i] > 0.0)
        ijsPosition[i] = 0.0;
    }
    else  {
      ijsPosition[i] = ijsPosition[i] - ijsDeadRange;
      if (ijsPosition[i] < 0.0)
        ijsPosition[i] = 0.0;
    }
  }
  
  //Generate joystick quadratic signal
  fjsQuadraticSignal[0] = sq((float)ijsPosition[0]) / fjsQuadraticDivider;
  if(ijsPosition[0] < 0.0)
    fjsQuadraticSignal[0] = - fjsQuadraticSignal[0];
  fjsQuadraticSignal[1] = sq((float)ijsPosition[1]) / fjsQuadraticDivider;
  if(ijsPosition[1] < 0.0)
    fjsQuadraticSignal[1] = - fjsQuadraticSignal[1];

  
  //Apply thresholds for digital sccroll functions
  ThreshJoyStickUp(fjsQuadraticSignal[0], jsScrollUp[0], jsScrollUpLS[0]);
  ThreshJoyStickUp(fjsQuadraticSignal[1], jsScrollUp[1], jsScrollUpLS[1]);
  ThreshJoyStickDown(fjsQuadraticSignal[0], jsScrollDown[0], jsScrollDownLS[0]);
  ThreshJoyStickDown(fjsQuadraticSignal[1], jsScrollDown[1], jsScrollDownLS[1]);
  
  
  //Process falling edge of debounced joystick button pressed
  if (!jsButDB && jsButLS)
  {
    //Toggle LED for user to know something has happened
    digitalWrite(obLED, !digitalRead(obLED));
    ljsLastButPushedTime = millis();
    // Change to next input state
    ijsInputMode++;
    //Skip coordinate system select
    if (ijsInputMode == 1)
      ijsInputMode++;
    if (ijsInputMode > 2)
      ijsInputMode = 0;
  }
  
  //Enter coordinate system select if button is pushed for more then 2secs
  if (!jsButDB && (abs(currentTime - ljsLastButPushedTime ) > 2000)){
    ljsLastButPushedTime = currentTime;
    ijsInputMode = 1;
    digitalWrite(obLED, !digitalRead(obLED));
  }
  

 
  if (ijsInputMode == 1){
    if (jsScrollUp[0] && !jsScrollUpLS[0])
    {
      ijsCoordSys ++;
      if (ijsCoordSys > 2)
        ijsCoordSys = 0;
    }
    if (jsScrollDown[0] && !jsScrollDownLS[0])
    {
      ijsCoordSys --;
      if (ijsCoordSys < 0)
        ijsCoordSys = 2;
    }
    if (ijsCoordSys ==0)
      ijsHighestAxInd = qn;
    else
      ijsHighestAxInd = en;
    if (ijsAxisControlled[0] >= ijsHighestAxInd)
      ijsAxisControlled[0] = 1;
    if (ijsAxisControlled[1] >= ijsHighestAxInd)
      ijsAxisControlled[1] = 0;
  }
  else if (ijsInputMode == 2){
      int i=0;
      int iOther = 1-i; //Joystick has 2 axis this is the index of the other one
      if (jsScrollUp[i] && !jsScrollUpLS[i])
      {
        ijsAxisControlled[i]++;
        if (ijsAxisControlled[i] >= ijsHighestAxInd)
          ijsAxisControlled[i] = 0;
        if (ijsAxisControlled[i] == ijsAxisControlled[iOther])        
          ijsAxisControlled[i]++;
        if (ijsAxisControlled[i] >= ijsHighestAxInd)
          ijsAxisControlled[i] = 0;
      }
      if (jsScrollDown[i] && !jsScrollDownLS[i])
      {
        ijsAxisControlled[i]--;
        if (ijsAxisControlled[i] < 0)
          ijsAxisControlled[i] = ijsHighestAxInd - 1;
        if (ijsAxisControlled[i] == ijsAxisControlled[iOther])        
          ijsAxisControlled[i]--;
        if (ijsAxisControlled[i] < 0)
          ijsAxisControlled[i] = ijsHighestAxInd - 1;
      }
      i=1;
      iOther = 1-i; //Joystick has 2 axis this is the index of the other one
      if (jsScrollDown[i] && !jsScrollDownLS[i])
      {
        ijsAxisControlled[i]++;
        if (ijsAxisControlled[i] >= ijsHighestAxInd)
          ijsAxisControlled[i] = 0;
        if (ijsAxisControlled[i] == ijsAxisControlled[iOther])        
          ijsAxisControlled[i]++;
        if (ijsAxisControlled[i] >= ijsHighestAxInd)
          ijsAxisControlled[i] = 0;
      }
      if (jsScrollUp[i] && !jsScrollUpLS[i])
      {
        ijsAxisControlled[i]--;
        if (ijsAxisControlled[i] < 0)
          ijsAxisControlled[i] = ijsHighestAxInd - 1;
        if (ijsAxisControlled[i] == ijsAxisControlled[iOther])        
          ijsAxisControlled[i]--;
        if (ijsAxisControlled[i] < 0)
          ijsAxisControlled[i] = ijsHighestAxInd - 1;
      }
  }
    
}

 
