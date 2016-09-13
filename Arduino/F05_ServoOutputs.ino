
//Check limits, converto to real life servos and write values to outputs
void WriteNewPositionToServo(int iServo)
{
  if (q[iServo] < qMins[iServo])
    q[iServo] = qMins[iServo];
  if (q[iServo] > qMaxs[iServo])
    q[iServo] = qMaxs[iServo];
  qServos[iServo] = (q[iServo] + qZeros[iServo] )* qDirections[iServo];
  if (qServos_L[iServo] != qServos[iServo]){
    qServos_L[iServo] = qServos[iServo];
    servos[iServo].write((int)(qServos[iServo]));
  }
}



  
//Updates the servo positions
void UpdateServoPositions() {
  if (ijsInputMode == 0){
    //Update positions depending on coordinate system and axis controlled
    switch (ijsCoordSys) {
      //Robot joints
      case 0:
        CalcManFeedRateJoint();  
        for (int i = 0; i< qn; i++)
          q[i] = q[i] + fManualCurrentJointFeedRate[i];
         break;
      //Global coordinates &
      //End effector coordinates
      //fManualCurrentCartFeedRate is allways in global coordinates
      //we process info the same way regardless of comands in end effector coordinates
      case 1:
      case 2:
        CalcManFeedRateCartesian(); 
        for (int i = 0; i< en; i++){
          er[i] = er[i] + fManualCurrentCartFeedRate[i];
        }
        for (int i = 0; i< qn; i++)
          q[i] = q[i] + d_q[i];
        //Compensate the q22 issue
        q[2] = q[2] + d_q[1];
        break;
    }     
    for(int i = 0; i < 6; i++)
      WriteNewPositionToServo(i);
    EndEffectorUpToDate = false;
  }
}


//Cyclic task based on interrupt configured on "Timer 6"
void InturruptBasedServo(){
  //If end effector is not up to date, update it!
  if (!EndEffectorUpToDate)
    CalculateEndEffector();
  UpdateServoPositions();
}

