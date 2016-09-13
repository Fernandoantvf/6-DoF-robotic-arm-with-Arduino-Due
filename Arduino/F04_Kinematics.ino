
  /*Definitions
  * 
  *Matrices generated for the following values: 
  * l1 = 116
  * l3 = 130
  * l5 = 50
  * r2 = 33
  * r22 = 38
  * b22 = 116
  * d0 = 45
  * d2 = 17
  * 
  *
  *
  */


  
 /*
  * Calculates end effector position
  */
 void CalculateEndEffector(){
#ifdef CheckKinematicsProcessingWeight   
  long lProcessTimeMicros = micros();
  long lProcessTimeMillis = millis();
#endif
  //The angles will be expressed in degrees (must be carful with the trig functions!)
  for (int i = 0; i < qn; i++){
    cq[i] = cos(q[i]*3.14/180.0);
    sq[i] = sin(q[i]*3.14/180.0);
  }
  q1q2 = q[2]- q[1];
  sq1q2 = sin(q1q2*3.14/180.0);
  cq1q2 = cos(q1q2*3.14/180.0);

  //q22
  sq22 = 0.0131578947368421 * (-57.1576766497729 * sqrt(-19538112.0 * sq(sq1q2) + 1811920.0 * sq1q2 + 25865277.0) * cq1q2 + 252648.0 * sq(sq1q2) + 971685.0 * sq1q2 + 293828.0)/(7656.0 * sq1q2 + 14545.0);

  //q22 is guaranteed to be on the 1st or 4th quadrant
  //so cq22 is allways positive
  cq22 = sqrt(-sq(sq22) + 1);
  
  //q22 is guaranteed to be on the 1st or 4th quadrant
  //it will also be in the same quadrant as q1q2
  q22 = asin(sq22);
  if (q22 > 90.0)
    q22 = q22 - 360.0;
  q22 = q22*180.0 / 3.14;

  q1q22 = q[1] + q22;
  cq1q22 = cos(q1q22*3.14/180.0);
  sq1q22 = sin(q1q22*3.14/180.0);


  
  //End effector position (ep=f(q)):
  e[0] = 116*sq[0]*sq[1] + 50*sq[0]*sq[4]*sq1q22*cq[3] + 17*sq[0]*sq1q22 - 50*sq[0]*cq[4]*cq1q22 - 130*sq[0]*cq1q22 - 45*sq[0] + 50*sq[3]*sq[4]*cq[0];
  e[1] = 50*sq[0]*sq[3]*sq[4] - 116*sq[1]*cq[0] - 50*sq[4]*sq1q22*cq[0]*cq[3] - 17*sq1q22*cq[0] + 50*cq[0]*cq[4]*cq1q22 + 130*cq[0]*cq1q22 + 45*cq[0];
  e[2] = 50*sq[4]*cq[3]*cq1q22 + 50*sq1q22*cq[4] + 130*sq1q22 + 116*cq[1] + 17*cq1q22;
      
  //End effector position (ep=f(q)):
  srx=(sq[4]*cq[3]*cq1q22 + sq1q22*cq[4])/sqrt(-sq(sq[3]*cq[5]*cq1q22 - sq[4]*sq[5]*sq1q22 + sq[5]*cq[3]*cq[4]*cq1q22) + 1);
  sry=sq[3]*cq[5]*cq1q22 - sq[4]*sq[5]*sq1q22 + sq[5]*cq[3]*cq[4]*cq1q22;
  srz=(-sq[0]*sq[3]*sq[5]*cq[4] + sq[0]*cq[3]*cq[5] + sq[3]*sq1q22*cq[0]*cq[5] + sq[4]*sq[5]*cq[0]*cq1q22 + sq[5]*sq1q22*cq[0]*cq[3]*cq[4])/sqrt(-sq(sq[3]*cq[5]*cq1q22 - sq[4]*sq[5]*sq1q22 + sq[5]*cq[3]*cq[4]*cq1q22) + 1);
  
  e[3] = asin(srx) * 180.0 / 3.14;
  e[4] = asin(sry) * 180.0 / 3.14;
  e[5] = asin(srz) * 180.0 / 3.14;
  
  
  //If user is directly controling joints, set required position to current position
  if (ijsCoordSys == 0)
    for (int i = 0; i < en; i++)
      er[i] = e[i];

  EndEffectorUpToDate = true;
  
#ifdef CheckKinematicsProcessingWeight   
  lProcessTimeMicros = micros() - lProcessTimeMicros;
  lProcessTimeMillis = millis() - lProcessTimeMillis;
  Serial.println("End effector position calc time: " + (String)lProcessTimeMicros + " us");
  Serial.println("End effector position calc time: " + (String)lProcessTimeMillis + " ms");
#endif
}


 /*
  * Calculates end effector position
  */
 void CalculateEndEffTransfMatrix(){
#ifdef CheckKinematicsProcessingWeight   
  long lProcessTimeMicros = micros();
  long lProcessTimeMillis = millis();
#endif
  //The angles will be expressed in degrees (must be carful with the trig functions!)

//End effector coordinates transformation matrix:
  Teegc[0][0] = -((sq[0]*sq1q22*cq[3] + sq[3]*cq[0])*cq[4] + sq[0]*sq[4]*cq1q22)*sq[5] - (sq[0]*sq[3]*sq1q22 - cq[0]*cq[3])*cq[5];
  Teegc[0][1] = (sq[0]*sq1q22*cq[3] + sq[3]*cq[0])*sq[4] - sq[0]*cq[4]*cq1q22;
  Teegc[0][2] = ((sq[0]*sq1q22*cq[3] + sq[3]*cq[0])*cq[4] + sq[0]*sq[4]*cq1q22)*cq[5] - (sq[0]*sq[3]*sq1q22 - cq[0]*cq[3])*sq[5];
  Teegc[0][3] = 116*sq[0]*sq[1] + 50*sq[0]*sq[4]*sq1q22*cq[3] + 17*sq[0]*sq1q22 - 50*sq[0]*cq[4]*cq1q22 - 130*sq[0]*cq1q22 - 45*sq[0] + 50*sq[3]*sq[4]*cq[0];
  Teegc[1][0] = ((-sq[0]*sq[3] + sq1q22*cq[0]*cq[3])*cq[4] + sq[4]*cq[0]*cq1q22)*sq[5] + (sq[0]*cq[3] + sq[3]*sq1q22*cq[0])*cq[5];
  Teegc[1][1] = (sq[0]*sq[3] - sq1q22*cq[0]*cq[3])*sq[4] + cq[0]*cq[4]*cq1q22;
  Teegc[1][2] = ((sq[0]*sq[3] - sq1q22*cq[0]*cq[3])*cq[4] - sq[4]*cq[0]*cq1q22)*cq[5] + (sq[0]*cq[3] + sq[3]*sq1q22*cq[0])*sq[5];
  Teegc[1][3] = 50*sq[0]*sq[3]*sq[4] - 116*sq[1]*cq[0] - 50*sq[4]*sq1q22*cq[0]*cq[3] - 17*sq1q22*cq[0] + 50*cq[0]*cq[4]*cq1q22 + 130*cq[0]*cq1q22 + 45*cq[0];
  Teegc[2][0] = (sq[4]*sq1q22 - cq[3]*cq[4]*cq1q22)*sq[5] - sq[3]*cq[5]*cq1q22;
  Teegc[2][1] = sq[4]*cq[3]*cq1q22 + sq1q22*cq[4];
  Teegc[2][2] = -(sq[4]*sq1q22 - cq[3]*cq[4]*cq1q22)*cq[5] - sq[3]*sq[5]*cq1q22;
  Teegc[2][3] = 50*sq[4]*cq[3]*cq1q22 + 50*sq1q22*cq[4] + 130*sq1q22 + 116*cq[1] + 17*cq1q22;
  Teegc[3][0] = 0;
  Teegc[3][1] = 0;
  Teegc[3][2] = 0;
  Teegc[3][3] = 1;
  
#ifdef CheckKinematicsProcessingWeight   
  lProcessTimeMicros = micros() - lProcessTimeMicros;
  lProcessTimeMillis = millis() - lProcessTimeMillis;
  Serial.println("End effector coordinate transf Matrix calc time: " + (String)lProcessTimeMicros + " us");
  Serial.println("End effector coordinate transf Matrix calc time: " + (String)lProcessTimeMillis + " ms");
#endif
}

/*
 * Calculates the jacobian
 */
void CalculateJacobian(){
#ifdef CheckKinematicsProcessingWeight   
  long lProcessTimeMicros = micros();
  long lProcessTimeMillis = millis();
#endif




  //Jacobian (Position):
  Jp[0][0] = -50*sq[0]*sq[3]*sq[4] + 116*sq[1]*cq[0] + 50*sq[4]*sq1q22*cq[0]*cq[3] + 17*sq1q22*cq[0] - 50*cq[0]*cq[4]*cq1q22 - 130*cq[0]*cq1q22 - 45*cq[0];
  Jp[0][1] = (50*sq[4]*cq[3]*cq1q22 + 50*sq1q22*cq[4] + 130*sq1q22 + 116*cq[1] + 17*cq1q22)*sq[0];
  Jp[0][2] = (50*sq[4]*cq[3]*cq1q22 + 50*sq1q22*cq[4] + 130*sq1q22 + 17*cq1q22)*sq[0];
  Jp[1][0] = 116*sq[0]*sq[1] + 50*sq[0]*sq[4]*sq1q22*cq[3] + 17*sq[0]*sq1q22 - 50*sq[0]*cq[4]*cq1q22 - 130*sq[0]*cq1q22 - 45*sq[0] + 50*sq[3]*sq[4]*cq[0];
  Jp[1][1] = -(50*sq[4]*cq[3]*cq1q22 + 50*sq1q22*cq[4] + 130*sq1q22 + 116*cq[1] + 17*cq1q22)*cq[0];
  Jp[1][2] = -(50*sq[4]*cq[3]*cq1q22 + 50*sq1q22*cq[4] + 130*sq1q22 + 17*cq1q22)*cq[0];
  Jp[2][0] = 0;
  Jp[2][1] = -116*sq[1] - 50*sq[4]*sq1q22*cq[3] - 17*sq1q22 + 50*cq[4]*cq1q22 + 130*cq1q22;
  Jp[2][2] = -50*sq[4]*sq1q22*cq[3] - 17*sq1q22 + 50*cq[4]*cq1q22 + 130*cq1q22;

  //Jacobian (Orientation):
  drx_dsrx = 1/sqrt(-sq(srx) + 1);
  dry_dsry = 1/sqrt(-sq(sry) + 1);
  drz_dsrz = 1/sqrt(-sq(srz) + 1);
  
  Jo[0][0] = drx_dsrx*((sq[4]*cq[3]*cq1q22 + sq1q22*cq[4])*(-2*sq[3]*sq[5]*cq[4]*cq1q22 + 2*cq[3]*cq[5]*cq1q22)*(sq[3]*cq[5]*cq1q22 - sq[4]*sq[5]*sq1q22 + sq[5]*cq[3]*cq[4]*cq1q22)/(2*pow(-sq(sq[3]*cq[5]*cq1q22 - sq[4]*sq[5]*sq1q22 + sq[5]*cq[3]*cq[4]*cq1q22) + 1, 3/2)) - sq[3]*sq[4]*cq1q22/sqrt(-sq(sq[3]*cq[5]*cq1q22 - sq[4]*sq[5]*sq1q22 + sq[5]*cq[3]*cq[4]*cq1q22) + 1));
  Jo[0][1] = drx_dsrx*((-sq[4]*sq1q22 + cq[3]*cq[4]*cq1q22)/sqrt(-sq(sq[3]*cq[5]*cq1q22 - sq[4]*sq[5]*sq1q22 + sq[5]*cq[3]*cq[4]*cq1q22) + 1) + (sq[4]*cq[3]*cq1q22 + sq1q22*cq[4])*(-2*sq[4]*sq[5]*cq[3]*cq1q22 - 2*sq[5]*sq1q22*cq[4])*(sq[3]*cq[5]*cq1q22 - sq[4]*sq[5]*sq1q22 + sq[5]*cq[3]*cq[4]*cq1q22)/(2*pow(-sq(sq[3]*cq[5]*cq1q22 - sq[4]*sq[5]*sq1q22 + sq[5]*cq[3]*cq[4]*cq1q22) + 1, 3/2)));
  Jo[0][2] = drx_dsrx*(sq[4]*cq[3]*cq1q22 + sq1q22*cq[4])*(-2*sq[3]*sq[5]*cq1q22 - 2*sq[4]*sq1q22*cq[5] + 2*cq[3]*cq[4]*cq[5]*cq1q22)*(sq[3]*cq[5]*cq1q22 - sq[4]*sq[5]*sq1q22 + sq[5]*cq[3]*cq[4]*cq1q22)/(2*pow(-sq(sq[3]*cq[5]*cq1q22 - sq[4]*sq[5]*sq1q22 + sq[5]*cq[3]*cq[4]*cq1q22) + 1, 3/2));
  Jo[1][0] = dry_dsry*(-sq[3]*sq[5]*cq[4]*cq1q22 + cq[3]*cq[5]*cq1q22);
  Jo[1][1] = dry_dsry*(-sq[4]*sq[5]*cq[3]*cq1q22 - sq[5]*sq1q22*cq[4]);
  Jo[1][2] = dry_dsry*(-sq[3]*sq[5]*cq1q22 - sq[4]*sq1q22*cq[5] + cq[3]*cq[4]*cq[5]*cq1q22);
  Jo[2][0] = drz_dsrz*((-2*sq[3]*sq[5]*cq[4]*cq1q22 + 2*cq[3]*cq[5]*cq1q22)*(sq[3]*cq[5]*cq1q22 - sq[4]*sq[5]*sq1q22 + sq[5]*cq[3]*cq[4]*cq1q22)*(-sq[0]*sq[3]*sq[5]*cq[4] + sq[0]*cq[3]*cq[5] + sq[3]*sq1q22*cq[0]*cq[5] + sq[4]*sq[5]*cq[0]*cq1q22 + sq[5]*sq1q22*cq[0]*cq[3]*cq[4])/(2*pow(-sq(sq[3]*cq[5]*cq1q22 - sq[4]*sq[5]*sq1q22 + sq[5]*cq[3]*cq[4]*cq1q22) + 1, 3/2)) + (-sq[0]*sq[3]*cq[5] - sq[0]*sq[5]*cq[3]*cq[4] - sq[3]*sq[5]*sq1q22*cq[0]*cq[4] + sq1q22*cq[0]*cq[3]*cq[5])/sqrt(-sq(sq[3]*cq[5]*cq1q22 - sq[4]*sq[5]*sq1q22 + sq[5]*cq[3]*cq[4]*cq1q22) + 1));
  Jo[2][1] = drz_dsrz*((-2*sq[4]*sq[5]*cq[3]*cq1q22 - 2*sq[5]*sq1q22*cq[4])*(sq[3]*cq[5]*cq1q22 - sq[4]*sq[5]*sq1q22 + sq[5]*cq[3]*cq[4]*cq1q22)*(-sq[0]*sq[3]*sq[5]*cq[4] + sq[0]*cq[3]*cq[5] + sq[3]*sq1q22*cq[0]*cq[5] + sq[4]*sq[5]*cq[0]*cq1q22 + sq[5]*sq1q22*cq[0]*cq[3]*cq[4])/(2*pow(-sq(sq[3]*cq[5]*cq1q22 - sq[4]*sq[5]*sq1q22 + sq[5]*cq[3]*cq[4]*cq1q22) + 1, 3/2)) + (sq[0]*sq[3]*sq[4]*sq[5] - sq[4]*sq[5]*sq1q22*cq[0]*cq[3] + sq[5]*cq[0]*cq[4]*cq1q22)/sqrt(-sq(sq[3]*cq[5]*cq1q22 - sq[4]*sq[5]*sq1q22 + sq[5]*cq[3]*cq[4]*cq1q22) + 1));
  Jo[2][2] = drz_dsrz*((-sq[0]*sq[3]*cq[4]*cq[5] - sq[0]*sq[5]*cq[3] - sq[3]*sq[5]*sq1q22*cq[0] + sq[4]*cq[0]*cq[5]*cq1q22 + sq1q22*cq[0]*cq[3]*cq[4]*cq[5])/sqrt(-sq(sq[3]*cq[5]*cq1q22 - sq[4]*sq[5]*sq1q22 + sq[5]*cq[3]*cq[4]*cq1q22) + 1) + (-2*sq[3]*sq[5]*cq1q22 - 2*sq[4]*sq1q22*cq[5] + 2*cq[3]*cq[4]*cq[5]*cq1q22)*(sq[3]*cq[5]*cq1q22 - sq[4]*sq[5]*sq1q22 + sq[5]*cq[3]*cq[4]*cq1q22)*(-sq[0]*sq[3]*sq[5]*cq[4] + sq[0]*cq[3]*cq[5] + sq[3]*sq1q22*cq[0]*cq[5] + sq[4]*sq[5]*cq[0]*cq1q22 + sq[5]*sq1q22*cq[0]*cq[3]*cq[4])/(2*(-sq(sq[3]*cq[5]*cq1q22 - sq[4]*sq[5]*sq1q22 + sq[5]*cq[3]*cq[4]*cq1q22) + 1, 3/2)));

  
  /*
   * Up to now we have considered that q22 is directly actuated, since there is a non linear conversion between q22 and (q2 - q1)
   * the the column 2 of J must be multiplied by d(q22)/d(q2-q1)
   */
  //dsq22_dq0q1 = 0.434210526315789*(-19538112.0*sq1q2**2 + 1811920.0*sq1q2 + 25865277.0)**(-0.5)*(222713040.0*(-19538112.0*sq1q2**2 + 1811920.0*sq1q2 + 25865277.0)**0.5*sq1q2*cq1q2 - 58614336.0*(-19538112.0*sq1q2**2 + 1811920.0*sq1q2 + 25865277.0)**0.5*cq1q2**3 + 418723765.0*(-19538112.0*sq1q2**2 + 1811920.0*sq1q2 + 25865277.0)**0.5*cq1q2 - 259086716425.987*sq1q2**4 - 972421211664.152*sq1q2**3 + 68470678389.8912*sq1q2**2 + 1155846560358.7*sq1q2 + 320165040836.395)/(7656.0*sq1q2 + 14545.0)**2;
  dsq22_dq1q2 = 0.434210526315789/sqrt(-19538112.0*sq(sq1q2) + 1811920.0*sq1q2 + 25865277.0)*(222713040.0*sqrt(-19538112.0*sq(sq1q2) + 1811920.0*sq1q2 + 25865277.0)*sq1q2*cq1q2 - 58614336.0*sqrt(-19538112.0*sq(sq1q2) + 1811920.0*sq1q2 + 25865277.0)*pow(cq1q2,3) + 418723765.0*sqrt(-19538112.0*sq(sq1q2) + 1811920.0*sq1q2 + 25865277.0)*cq1q2 - 259086716425.987*pow(sq1q2,4) - 972421211664.152*pow(sq1q2,3) + 68470678389.8912*sq(sq1q2) + 1155846560358.7*sq1q2 + 320165040836.395)/sq(7656.0*sq1q2 + 14545.0);
  //dcq22_dsq22 shouldn't be used
  //dcq22_dsq22 = -sq22/sqrt(-sq(sq22) + 1);
  dq22_dsq22 = 1/sqrt(-sq(sq22) + 1);
  dq22_dq1q2 = dq22_dsq22 * dsq22_dq1q2;
  for (int i= 0; i<3; i++)
  {
    Jp[i][2] = Jp[i][2] * dq22_dq1q2;
  }


#ifdef CheckKinematicsProcessingWeight   
  //Finished (report the time it took)
  lProcessTimeMicros = micros() - lProcessTimeMicros;
  lProcessTimeMillis = millis() - lProcessTimeMillis;
  Serial.println("Jacobian calc time: " + (String)lProcessTimeMicros + " us");
  Serial.println("Jacobian calc time: " + (String)lProcessTimeMillis + " ms");
#endif
}

/*
 * Invert Jacobian
 * Calculate the pseudo-inverse, (this solves the problem caused by non square or unique matrix
 * Jinv = Jtrans*(J*Jtrans)inv
 */
void InvertJacobian(){
#ifdef CheckKinematicsProcessingWeight   
  long lProcessTimeMicros = micros();
  long lProcessTimeMillis = millis();
#endif
  
  //Pseudo inverse: Jinv = Jtrans*(J*Jtrans)inv
  //Matrix1.Transpose((float*)J,3,3,(float*)Jtrans);
  //Matrix1.Multiply((float*)J,(float*)Jtrans,3,3,3,(float*)J_Jtrans_inv);
  //bLastJacobianInversionSuccessful = (bool)Matrix1.Invert((float*)J_Jtrans_inv,3);
  //In some rare cases J * Jtrans can be singular
  //If inversion did not succeed, keep using the old one
  //if(bLastJacobianInversionSuccessful)
  //  Matrix1.Multiply((float*)Jtrans,(float*)J_Jtrans_inv,3,3,3,(float*)Jinv);

  //Invert position Jacobian
  Matrix1.Copy((float*)Jp,3,3,(float*)JpInvTemp);  
  bLastJpInversionSuccessful = (bool)Matrix1.Invert((float*)JpInvTemp,3);
  if (bLastJpInversionSuccessful)
    Matrix1.Copy((float*)JpInvTemp,3,3,(float*)JpInv);
    
  //Invert orientation Jacobian
  //Matrix1.Copy((float*)Jo,3,3,(float*)JoInvTemp);
  //bLastJoInversionSuccessful = (bool)Matrix1.Invert((float*)JoInvTemp,3);
  //if (bLastJoInversionSuccessful)
  //  Matrix1.Copy((float*)JoInvTemp,3,3,(float*)JoInv);

  //Pseudo inverse: Jinv = Jtrans*(J*Jtrans)inv
  Matrix1.Transpose((float*)Jo,3,3,(float*)JoTrans);
  Matrix1.Multiply((float*)Jo,(float*)JoTrans,3,3,3,(float*)JoJoTransInv);
  bLastJoInversionSuccessful = (bool)Matrix1.Invert((float*)JoJoTransInv,3);
  //In some rare cases J * Jtrans can be singular
  //If inversion did not succeed, keep using the old one
  if(bLastJoInversionSuccessful)
    Matrix1.Multiply((float*)JoTrans,(float*)JoJoTransInv,3,3,3,(float*)JoInv);

  
#ifdef CheckKinematicsProcessingWeight   
  //Finished (report the time it took)
  lProcessTimeMicros = micros() - lProcessTimeMicros;
  lProcessTimeMillis = millis() - lProcessTimeMillis;
  Serial.println("Jacobian invertion time: " + (String)lProcessTimeMicros + " us");
  Serial.println("Jacobian inversion time: " + (String)lProcessTimeMillis + " ms");
#endif
}


/*
 * Calculates the direction of movement and converts it to the joint coordinate system
 */
void CalculateDelta(){
#ifdef CheckKinematicsProcessingWeight   
  long lProcessTimeMicros = micros();
  long lProcessTimeMillis = millis();
#endif

  if (bLastJpInversionSuccessful && bLastJoInversionSuccessful){  
    
    d_e_norm = 0.0;
    fDistToTargetPos = 0.0;
    fDistToTargetOri = 0.0;
    float d_eo[3]; //Orientation part of d_e 
    for (int i = 0; i < en; i++){
      d_e[i] = er[i] - e[i];
      d_e_norm = d_e_norm + sq(d_e[i]);
      if (i<3)
       fDistToTargetPos = fDistToTargetPos + sq(d_e[i]); 
      else
      {
       fDistToTargetOri = fDistToTargetOri + sq(d_e[i]);
       d_eo[i-3] = d_e[i];
      }
    }
    
    //d_e_norm = sqrt(d_e_norm);
    //fDistToTargetPos = sqrt(fDistToTargetPos);
    //fDistToTargetOri = sqrt(fDistToTargetOri);
  
    //Delta should not go too far... otherwise we move away from the current kinematic configuration before we can recalc...
    //If the norm of Delta is higher then the maximum feed per scan, scale delta...
  
    //if(fDistToTargetPos > 20.0)
    //  for (int i = 0; i < 3; i++)
    //    d_e[i] = d_e[i] * 20.0 / fDistToTargetPos;

          
    float d_q_aux[3];
    
    //Convert to joint coordinate system
    Matrix1.Multiply((float*)JpInv,(float*)d_e,3,3,1,(float*)d_q);
    Matrix1.Multiply((float*)JoInv,(float*)d_eo,3,3,1,(float*)d_q_aux);

    for (int i = 0; i < 3; i++)
      d_q[i+3] = d_q_aux[i];
    
  }
  //If matrix is singular, we should slightly move the robot
  //If it is allready in motion, keep moving in same direction, otherwise just sligtly offset one of the axis
  else
  {
      d_q[3] = 0.01;
      d_q[4] = 0.01;
      d_q[5] = 0.01;    
  }
 
  
#ifdef CheckKinematicsProcessingWeight   
  //Finished (report the time it took)
  lProcessTimeMicros = micros() - lProcessTimeMicros;
  lProcessTimeMillis = millis() - lProcessTimeMillis;
  Serial.println("Direction calc time: " + (String)lProcessTimeMicros + " us");
  Serial.println("Direction calc time: " + (String)lProcessTimeMillis + " ms");
#endif
}


