/*
 * Converts Float to string with format "+/-000.00
 */
String FloatToStringFixedChars(float fField){
      char cPrintChars[15];
      String sPrintString = String(fField,2);
      int iPointPosition = sPrintString.indexOf('.');
      int iUsefullStringLength = 3 + iPointPosition;
      
      if (fField >= 0.0) sPrintString = sPrintString = sPrintString.substring(0,iUsefullStringLength);
      else  sPrintString = sPrintString.substring(1,iUsefullStringLength);       
      int iUsefullStringlength = sPrintString.length();
      for (int i = iUsefullStringlength; i < 6; i++)
        sPrintString = "0" + sPrintString;
      if (fField >= 0.0) sPrintString = "+" + sPrintString; 
      else  sPrintString = "-" + sPrintString; 
      return sPrintString;
}

 void float2Bytes(float val,byte* bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    byte temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}

//Update user serial info on COM
void UpdateUserInfoOnSerial(){ 

#ifndef TransmitPackageInCompactModeForDebug
  iTransmitVar++;
  if (iTransmitVar >= 19)
    iTransmitVar =0;
  if (iTransmitVar < 7)
    Serial.println("q" +  String(iTransmitVar) + "=" + FloatToStringFixedChars(q[iTransmitVar]) + "0");
  else if (iTransmitVar < 14)
    Serial.println("e" +  String(iTransmitVar - 7) + "=" + FloatToStringFixedChars(e[iTransmitVar - 7] + eUsOff[iTransmitVar - 7]) + "0");
  else if (iTransmitVar == 14)
    Serial.println("Mode=(" + String(ijsInputMode) + ")");
  else if (iTransmitVar == 15)
    Serial.println("CooSys=(" + String(ijsCoordSys) + ")");
  else if (iTransmitVar == 16)
    Serial.println("AxCo0=(" + String(ijsAxisControlled[0]) + ")");
  else if (iTransmitVar == 17)
    Serial.println("AxCo1=(" + String(ijsAxisControlled[1]) + ")");
  else if (iTransmitVar == 18){
    Serial.println("Position:[" + FloatToStringFixedChars(e[0]) + ";" + FloatToStringFixedChars(e[1]) + ";" + FloatToStringFixedChars(e[2]) + ";" + FloatToStringFixedChars(e[3]) + ";" + FloatToStringFixedChars(e[4]) + ";" + FloatToStringFixedChars(e[5]) + ";" + FloatToStringFixedChars(e[6]) + "]");
    Serial.println("Setpoint:[" + FloatToStringFixedChars(er[0]) + ";" + FloatToStringFixedChars(er[1]) + ";" + FloatToStringFixedChars(er[2]) + ";" + FloatToStringFixedChars(er[3]) + ";" + FloatToStringFixedChars(er[4]) + ";" + FloatToStringFixedChars(er[5]) + ";" + FloatToStringFixedChars(er[6]) + "]");
    Serial.println("delta e: [" + FloatToStringFixedChars(d_e[0]) + ";" + FloatToStringFixedChars(d_e[1]) + ";" + FloatToStringFixedChars(d_e[2]) + ";" + FloatToStringFixedChars(d_e[3]) + ";" + FloatToStringFixedChars(d_e[4]) + ";" + FloatToStringFixedChars(d_e[5]) + ";" + FloatToStringFixedChars(d_e[6]) + "]");
    Serial.println("delta q: [" + FloatToStringFixedChars(d_q[0]) + ";" + FloatToStringFixedChars(d_q[1]) + ";" + FloatToStringFixedChars(d_q[2]) + ";" + FloatToStringFixedChars(d_q[3]) + ";" + FloatToStringFixedChars(d_q[4]) + ";" + FloatToStringFixedChars(d_q[5]) + ";" + FloatToStringFixedChars(d_q[6]) + "]");
  }
    
#else

  Serial.println("");
  Serial.println("Mode=(" + String(ijsInputMode) + ")  " + "CooSys=(" + String(ijsCoordSys) + ")  " + "AxCo0=(" + String(ijsAxisControlled[0]) + ")  " + "AxCo1=(" + String(ijsAxisControlled[1]) + ")");
  Serial.println("JoyStick: [" + (String)fjsQuadraticSignal[0] + ";" + (String)fjsQuadraticSignal[1] + "]  " + "Pushutton: " +  String(jsButDB) + "  xScroll:(U" + String(jsScrollUp[0]) + ";D" + String(jsScrollDown[0]) + ")  yScroll:(U" + String(jsScrollUp[1]) + ";D" + String(jsScrollDown[1]) + ")");
  Serial.println("");
  Serial.println("Joints:[" + FloatToStringFixedChars(q[0]) + ";" + FloatToStringFixedChars(q[1]) + ";" + FloatToStringFixedChars(q[2]) + "  ;  " + FloatToStringFixedChars(q[3]) + ";" + FloatToStringFixedChars(q[4]) + ";" + FloatToStringFixedChars(q[5]) + "  ;  " + FloatToStringFixedChars(q[6]) + "]");
  Serial.println("Servos:[" + FloatToStringFixedChars(qServos[0]) + ";" + FloatToStringFixedChars(qServos[1]) + ";" + FloatToStringFixedChars(qServos[2]) + "  ;  " + FloatToStringFixedChars(qServos[3]) + ";" + FloatToStringFixedChars(qServos[4]) + ";" + FloatToStringFixedChars(qServos[5]) + "  ;  " + FloatToStringFixedChars(qServos[6]) + "]");
  Serial.println("");
  Serial.println("Position:[" + FloatToStringFixedChars(e[0]) + ";" + FloatToStringFixedChars(e[1]) + ";" + FloatToStringFixedChars(e[2]) + "  ;  " + FloatToStringFixedChars(e[3]) + ";" + FloatToStringFixedChars(e[4]) + ";" + FloatToStringFixedChars(e[5]) + "  ;  " + FloatToStringFixedChars(e[6]) + "]");
  Serial.println("eReq    :[" + FloatToStringFixedChars(er[0]) + ";" + FloatToStringFixedChars(er[1]) + ";" + FloatToStringFixedChars(er[2]) + " ; " + FloatToStringFixedChars(er[3]) + ";" + FloatToStringFixedChars(er[4]) + ";" + FloatToStringFixedChars(er[5]) + ";" + FloatToStringFixedChars(er[6]) + "]");
  Serial.println("delta e :[" + FloatToStringFixedChars(d_e[0]) + ";" + FloatToStringFixedChars(d_e[1]) + ";" + FloatToStringFixedChars(d_e[2]) + " ; " + FloatToStringFixedChars(d_e[3]) + ";" + FloatToStringFixedChars(d_e[4]) + ";" + FloatToStringFixedChars(d_e[5]) + " ; " + FloatToStringFixedChars(d_e[6]) + "]");
  Serial.println("delta q :[" + FloatToStringFixedChars(d_q[0]) + ";" + FloatToStringFixedChars(d_q[1]) + ";" + FloatToStringFixedChars(d_q[2]) + " ; " + FloatToStringFixedChars(d_q[3]) + ";" + FloatToStringFixedChars(d_q[4]) + ";" + FloatToStringFixedChars(d_q[5]) + " ; " + FloatToStringFixedChars(d_q[6]) + "]");


  Serial.println("");
  Serial.println("VelocRequest:[" + FloatToStringFixedChars(fjsDelta[0]) + ";" + FloatToStringFixedChars(fjsDelta[1]) + ";" + FloatToStringFixedChars(fjsDelta[2]) + ";" + FloatToStringFixedChars(fjsDelta[3]) + ";" + FloatToStringFixedChars(fjsDelta[4]) + ";" 
    + FloatToStringFixedChars(fjsDelta[5]) + ";" + FloatToStringFixedChars(fjsDelta[6]) + "]");
  if (ijsCoordSys == 0)
    Serial.println("CurrentVeloc:[" + FloatToStringFixedChars(fManualCurrentJointFeedRate[0]) + ";" + FloatToStringFixedChars(fManualCurrentJointFeedRate[1]) + ";" + FloatToStringFixedChars(fManualCurrentJointFeedRate[2]) + ";" 
      + FloatToStringFixedChars(fManualCurrentJointFeedRate[3]) + ";" + FloatToStringFixedChars(fManualCurrentJointFeedRate[4]) + ";" + FloatToStringFixedChars(fManualCurrentJointFeedRate[5]) + ";" + FloatToStringFixedChars(fManualCurrentJointFeedRate[6]) + "]");
  else
    Serial.println("CurrentVeloc:[" + FloatToStringFixedChars(fManualCurrentCartFeedRate[0]) + ";" + FloatToStringFixedChars(fManualCurrentCartFeedRate[1]) + ";" + FloatToStringFixedChars(fManualCurrentCartFeedRate[2]) + ";" 
      + FloatToStringFixedChars(fManualCurrentCartFeedRate[3]) + " ; " + FloatToStringFixedChars(fManualCurrentCartFeedRate[4]) + ";" + FloatToStringFixedChars(fManualCurrentCartFeedRate[5]) + " ; " + FloatToStringFixedChars(fManualCurrentCartFeedRate[6]) + "]");
  Serial.println("qAvStrokes:[" + FloatToStringFixedChars(qAvStrokes[0]) + ";" + FloatToStringFixedChars(qAvStrokes[1]) + ";" + FloatToStringFixedChars(qAvStrokes[2]) + ";" 
    + FloatToStringFixedChars(qAvStrokes[3]) + " ; " + FloatToStringFixedChars(qAvStrokes[4]) + ";" + FloatToStringFixedChars(qAvStrokes[5]) + " ; " + FloatToStringFixedChars(qAvStrokes[6]) + "]");
  Serial.println("eAvStrokes:[" + FloatToStringFixedChars(eAvStrokes[0]) + ";" + FloatToStringFixedChars(eAvStrokes[1]) + ";" + FloatToStringFixedChars(eAvStrokes[2]) + ";" 
    + FloatToStringFixedChars(eAvStrokes[3]) + " ; " + FloatToStringFixedChars(eAvStrokes[4]) + ";" + FloatToStringFixedChars(eAvStrokes[5]) + " ; " + FloatToStringFixedChars(eAvStrokes[6]) + "]");

#endif
#ifdef DebugTrigonometry
  Serial.println("Trigonometry:");
  for (int i = 0; i < qn; i++){
    Serial.print(" || cos(q" + String(i) + "):" + FloatToStringFixedChars(cosq[i]) + " | sin(q" + String(i) + "):" + FloatToStringFixedChars(sinq[i]));
  }
  Serial.println("");
  Serial.print(" || cos(q[2]-q[1]):" + FloatToStringFixedChars(cosq1q2) + " | sin(q[2]-q[1]):" + FloatToStringFixedChars(sinq1q2));
  Serial.print(" || cos(q[1]+q[22]):" + FloatToStringFixedChars(cosq1q22) + " | sin(q[1]+q[22]):" + FloatToStringFixedChars(sinq1q22));
  Serial.println(" || q[22]:" + FloatToStringFixedChars(q22));
#endif
#ifdef DebugMatrixCalculations 
    Matrix1.Print((float*)Jo, 3, 3, "J0   ="); 
    Matrix1.Print((float*)JoInv, 3, 3, "JoInv=");
#endif
}

