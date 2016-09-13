
/*
 * Transform vectors from end effector coordinates to global coordinates 
 */
void TransformEecGc(float* vector){
    float vectorAux1[4];
    float vectorAux2[4];
    //Transform position
    for (int i = 0; i < 3; i++)
      vectorAux1[i] = vector[i];
    vectorAux1[3] = 1;
    Matrix1.Multiply((float*)Teegc,(float*)vectorAux1,4,4,1,(float*)vectorAux2);
    for (int i = 0; i < 3; i++)
       vector[i] = vectorAux2[i];
    //Transform Orientation
    for (int i = 0; i < 3; i++)
      vectorAux1[i] = vector[i+3];
    vectorAux1[3] = 1;
    Matrix1.Multiply((float*)Teegc,(float*)vectorAux1,4,4,1,(float*)vectorAux2);
    for (int i = 0; i < 3; i++)
       vector[i + 3] = vectorAux2[i];
}

/*
 * Transforms a Delta from global coordinates to joint coordinates
 */
void TransformGcQc(float* vectorIn, float* vectorOut){
  Matrix1.Multiply((float*)Jinv,(float*)vectorIn,qn,en,1,(float*)vectorOut);
}

/*
 * Transforms a Delta from global coordinates to joint coordinates
 */
void TransformQcGc(float* vectorIn, float* vectorOut){
  Matrix1.Multiply((float*)J,(float*)vectorIn,en,qn,1,(float*)vectorOut);
}


/*
 * Calculates remaining available strokes
 */
void CalcRemainStrokes(){
  for (int i = 0; i < qn; i++){
    if (abs(q[i] - qMins[i]) < abs(q[i] - qMaxs[i]))
      qAvStrokes[i] = qMins[i] - q[i] ;
    else 
      qAvStrokes[i] = qMaxs[i] - q[i] ;
  }
  TransformQcGc(qAvStrokes, eAvStrokes);
}


/*
 * Calculate manual feedrate  for direct joint control based on joystick, linear acceleration ramps
 * and joint stroke limitations
 */
 
void CalcManFeedRateCartesian(){
  //Calculate Direction of movement
  for (int i=0; i<en; i++)
    fjsDelta[i] = 0.0;
  //Leave Required Velocity as zero if not in manual control mode  
  if (ijsInputMode == 0)
   for (int i=0; i<2; i++)
      fjsDelta[ijsAxisControlled[i]] = fjsQuadraticSignal[i]  * fMaxManualLinFeedPerServoCycle * fjsJoyStickDirCart[ijsAxisControlled[i]];
  //Convert to global coordinates if working on end effector coordinate system
  if (ijsCoordSys == 2)
    TransformEecGc(fjsDelta);
  //From here on, we can work allways in global coordinate system  
  
  //Check if there is enough available stroke to stop
  CalcRemainStrokes();
  for (int i=0; i < en; i++){
    float fAvailableStroke = 1000000.0;
    /*
    if ((fjsDelta[i] > 0.0 and eAvStrokes[i] > 0.0) || (fjsDelta[i] < 0.0 and eAvStrokes[i] < 0.0))
      fAvailableStroke = abs(eAvStrokes[i]);
    */
    float fFeedRateVariationPerServoCycle = 0.0;
    if (i < 3)
      fFeedRateVariationPerServoCycle = fManualLinFeedVariationPerServoCycle;
    else
      fFeedRateVariationPerServoCycle = fManualRotFeedVariationPerServoCycle;
    float fRequiredStrokeToStop = sq(fManualCurrentCartFeedRate[i])  / (2 * fFeedRateVariationPerServoCycle);
    //If required stroke to stop is longer than the available stroke, reduce velocity immediately
    if ((fRequiredStrokeToStop > fAvailableStroke)){
      if (fManualCurrentCartFeedRate[i] > 0.0){
        fManualCurrentCartFeedRate[i] = fManualCurrentCartFeedRate[i] - fFeedRateVariationPerServoCycle;
        if (fManualCurrentCartFeedRate[i] < 0.0)
          fManualCurrentCartFeedRate[i] = 0.0;
      }
      if (fManualCurrentCartFeedRate[i] < 0.0){
        fManualCurrentCartFeedRate[i] = fManualCurrentCartFeedRate[i] + fFeedRateVariationPerServoCycle ;
        if (fManualCurrentCartFeedRate[i] > 0.0)
          fManualCurrentCartFeedRate[i] = 0.0;
      } 
    }
    //If we have plenty of space to reduce velocity, accelerate or deccelerate to approx to joystick required speed
    else{
      if (fjsDelta[i] > fManualCurrentCartFeedRate[i]){
        fManualCurrentCartFeedRate[i] = fManualCurrentCartFeedRate[i] + fFeedRateVariationPerServoCycle;
        if (fManualCurrentCartFeedRate[i] > 0.0 && fjsDelta[i]==0)
          fManualCurrentCartFeedRate[i] = 0.0;
      }
      else{
        fManualCurrentCartFeedRate[i] = fManualCurrentCartFeedRate[i] - fFeedRateVariationPerServoCycle;
        if (fManualCurrentCartFeedRate[i] < 0.0 && fjsDelta[i]==0)
          fManualCurrentCartFeedRate[i] = 0.0;
      }
    }    
  }
}




/*
 * Calculate manual feedrate  for direct joint control based on joystick and linear acceleration ramps
 * and joint stroke limitations
 */
void CalcManFeedRateJoint(){
  //Serial.println("");
  //Calculate Direction of movement
  for (int i=0; i< qn; i++)
    fjsDelta[i] = 0.0;
  //Leave Required Velocity as zero if not in manual control mode  
  if (ijsInputMode == 0)
    for (int i=0; i < 2; i++)
      fjsDelta[ijsAxisControlled[i]] = fjsQuadraticSignal[i]  * fMaxManualJointFeedPerServoCycle * fjsJoyStickDirJoint[ijsAxisControlled[i]];
  
  //Check if there is enough available stroke to stop
  CalcRemainStrokes();
  for (int i=0; i < en; i++){
    float fAvailableStroke = 1000000.0;
    if ((fjsDelta[i] > 0.0 and qAvStrokes[i] > 0.0) || (fjsDelta[i] < 0.0 and qAvStrokes[i] < 0.0))
      fAvailableStroke = abs(qAvStrokes[i]);
    float fFeedRateVariationPerServoCycle = fManualJointFeedVariationPerServoCycle;
    float fRequiredStrokeToStop = sq(fManualCurrentJointFeedRate[i])  / (2 * fFeedRateVariationPerServoCycle);
    
    //Serial.println("ReqStrokeToStop:" + FloatToStringFixedChars(fRequiredStrokeToStop) + "  Available Stroke" + FloatToStringFixedChars(fAvailableStroke));
    //Serial.println("fManualJointFeedVariationPerServoCycle:" + FloatToStringFixedChars(fManualJointFeedVariationPerServoCycle));
    
    //If required stroke to stop is longer than the available stroke, reduce velocity immediately
    if ((fRequiredStrokeToStop > fAvailableStroke)){
      if (fManualCurrentJointFeedRate[i] > 0.0){
        fManualCurrentJointFeedRate[i] = fManualCurrentJointFeedRate[i] - fFeedRateVariationPerServoCycle;
        if (fManualCurrentJointFeedRate[i] < 0.0)
          fManualCurrentJointFeedRate[i] = 0.0;
      }
      if (fManualCurrentJointFeedRate[i] < 0.0){
        fManualCurrentJointFeedRate[i] = fManualCurrentJointFeedRate[i] + fFeedRateVariationPerServoCycle ;
        if (fManualCurrentJointFeedRate[i] > 0.0)
          fManualCurrentJointFeedRate[i] = 0.0;
      } 
    }
    //If we have plenty of space to reduce velocity, accelerate or deccelerate to approx to joystick required speed
    else{
      if (fjsDelta[i] > fManualCurrentJointFeedRate[i]){
        fManualCurrentJointFeedRate[i] = fManualCurrentJointFeedRate[i] + fFeedRateVariationPerServoCycle;
        if (fManualCurrentJointFeedRate[i] > 0.0 && fjsDelta[i]==0)
          fManualCurrentJointFeedRate[i] = 0.0;
      }
      else{
        fManualCurrentJointFeedRate[i] = fManualCurrentJointFeedRate[i] - fFeedRateVariationPerServoCycle;
        if (fManualCurrentJointFeedRate[i] < 0.0 && fjsDelta[i]==0)
          fManualCurrentJointFeedRate[i] = 0.0;
      }
    }    
  }
}
  
  
  
  
  
  
  
  
  
 
