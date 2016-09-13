# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import sympy as sp

##/Matrices generated for the following values:
#l1 = 116
#l3 = 130
#l5 = 50
#r2 = 33
#r22 = 38
#b22 = 116
#d0 = 45
#d2 = 17
pi = 3.14

# set up our joint angle symbols (6th angle doesn't affect any kinematics)
q = [sp.Symbol('q0'), sp.Symbol('q1'), sp.Symbol('q2'), sp.Symbol('q3'), 
     sp.Symbol('q4'), sp.Symbol('q5')]
    
# set up our arm segment length symbols
l1 = sp.Symbol('l1')
l3 = sp.Symbol('l3')
l5 = sp.Symbol('l5')
r2 = sp.Symbol('r2')
r22 = sp.Symbol('r22')
b22 = sp.Symbol('b22')
d0 = sp.Symbol('d0')
d2 = sp.Symbol('d2')
    
sinq22 = sp.Symbol('sinq22')
cosq22 = sp.Symbol('cosq22')
q22 = sp.Symbol('q22')




def calc_pos_matrix():
    T0gc = sp.Matrix([[sp.cos(q[0]), -sp.sin(q[0]), 0, -d0*sp.sin(q[0])],
                     [sp.sin(q[0]), sp.cos(q[0]), 0, d0*sp.cos(q[0])],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])
 
    T10 = sp.Matrix([[1, 0, 0, 0],
                    [0, sp.cos(q[1]), -sp.sin(q[1]), -l1*sp.sin(q[1])],
                    [0, sp.sin(q[1]), sp.cos(q[1]), l1*sp.cos(q[1])],
                    [0, 0, 0, 1]])
    

    T21 = sp.Matrix([[1, 0, 0, 0],
                    [0, sp.cos(q[2]), -sp.sin(q[2]), -d2*sp.sin(q[2])],
                    [0, sp.sin(q[2]), sp.cos(q[2]), d2*sp.cos(q[2])],
                    [0, 0, 0, 1]])
 
    T32 = sp.Matrix([[sp.cos(q[3]), 0, sp.sin(q[3]), 0],
                    [0, 1, 0, l3],
                    [-sp.sin(q[3]), 0, sp.cos(q[3]), 0],
                    [0, 0, 0, 1]])
 
    T43 = sp.Matrix([[1, 0, 0, 0],
                    [0, sp.cos(q[4]), -sp.sin(q[4]), 0],
                    [0, sp.sin(q[4]), sp.cos(q[4]), 0],
                    [0, 0, 0, 1]])
 
    Tee4 = sp.Matrix([[sp.cos(q[5]), 0, sp.sin(q[5]), 0],
                    [0, 1, 0, l5],
                    [-sp.sin(q[5]), 0, sp.cos(q[5]), 0],
                    [0, 0, 0, 1]])
 
    Teegc = T0gc * T10 * T21 * T32 * T43 * Tee4
 
    # position of the end-effector relative to joint axes 6 (right at the origin)
    eeep = sp.Matrix([0,0,0,1])
 
    ep = Teegc * eeep
    
    

    print("//End effector position (ep=f(q)):")
    for i in range(0, 3):
        ep[i] = sp.simplify(ep[i])
        print(mass_replace("e[" + str(i) + "] = " + str(ep[i]) + ";"))
        
    print("") 
    print("//Jacobian (Position):")
    Jp = sp.zeros(3,3)
             
    for i in range(0, 3):
        for j in range(0, 3):
            Jp[i,j] = sp.simplify(ep[i].diff(q[j]))
            print(mass_replace("Jp[" + str(i) + "][" + str(j) + "] = " + str(Jp[i,j]) + ";"))
    
    print("") 



    print("//End effector coordinates transformation matrix:")         
    for i in range(0, 4):
        for j in range(0, 4):
            Teegc[i,j] = sp.simplify(Teegc[i,j])
            print(mass_replace("Teegc[" + str(i) + "][" + str(j) + "] = " + str(Teegc[i,j]) + ";"))
    
    print("")        
        

 #.evalf()

def calc_orient_matrix(): 



    #rx = sp.Symbol('rx')
    #ry = sp.Symbol('ry')
    #rz = sp.Symbol('rz')
        
    sinrx = sp.Symbol('sinrx')
    sinry = sp.Symbol('sinry')    
    sinrz = sp.Symbol('sinrz')    

    #cosrx = sp.Symbol('cosrx')
    #cosry = sp.Symbol('cosry')
    #cosrz = sp.Symbol('cosrz')

    cosrx = sp.sqrt(1-sinrx**2)
    cosry = sp.sqrt(1-sinry**2)
    cosrz = sp.sqrt(1-sinrz**2)
    
    
    R0gc = sp.Matrix([[sp.cos(q[0]), -sp.sin(q[0]), 0],
                      [sp.sin(q[0]),  sp.cos(q[0]), 0],
                      [    0,      0, 1]])
 
    R10 = sp.Matrix([[1,     0,      0],
                     [0, sp.cos(q[1]), -sp.sin(q[1])],
                     [0, sp.sin(q[1]),  sp.cos(q[1])]])

    R21 = sp.Matrix([[1,     0,      0],
                     [0, sp.cos(q[2]), -sp.sin(q[2])],
                     [0, sp.sin(q[2]),  sp.cos(q[2])]])
 
    R32 = sp.Matrix([[ sp.cos(q[3]), 0, sp.sin(q[3])],
                     [     0, 1,     0],
                     [-sp.sin(q[3]), 0, sp.cos(q[3])]])
 
    R43 = sp.Matrix([[1,     0,      0],
                     [0, sp.cos(q[4]), -sp.sin(q[4])],
                     [0, sp.sin(q[4]),  sp.cos(q[4])]])
 
    Ree4 = sp.Matrix([[ sp.cos(q[5]), 0, sp.sin(q[5])],
                      [     0, 1,     0],
                      [-sp.sin(q[5]), 0, sp.cos(q[5])]])
                           

    Rx = sp.Matrix([[1,      0,      0],
                     [0, cosrx, -sinrx],
                     [0, sinrx,  cosrx]])
                     
                     
    Ry = sp.Matrix([[ cosry, 0, sinry],
                    [     0, 1,     0],
                    [-sinry, 0, cosry]])   
                    
                    
    Rz = sp.Matrix([[cosrz, -sinrz, 0],
                    [sinrz,  cosrz, 0],
                    [    0,      0, 1]])                
     
     
    # Rotation transformation 1
    Reegc1 = R0gc * R10 * R21 * R32 * R43 * Ree4
    
    #Rotation transformation 2
    Reegc2 = Rz * Ry * Rx    
    
    for i in range(0, 3):
        for j in range(0, 3):
            print(mass_replace_for_doc("Reegc(f(rxryrz)[" + str(i) + "][" + str(j) + "] = " + str(Reegc2[i,j]) + ";"))
    print("")
    
    return
    
    #If we transform any vector with any of the 2 matrices,
    #We should get the same result, so:
    testvec = sp.Matrix([1,0,0])    
    testvec1 = Reegc1 * testvec        
    testvec2 = Reegc2 * testvec
    #print("Transforming [1,0,0] we get:")
    print("") 
    zeroVec = testvec2-testvec1
    solution1 = sp.solve(zeroVec[2], sinry)
    sinry1 = sp.simplify(solution1[0])
    expression_sinry = mass_replace("sinry=" + str(sinry1) + ";")
     
    
    #print("And:") 
    solution2 =sp.solve(zeroVec[1].subs(sinry, sinry1), sinrz)
    sinrz1 = sp.simplify(solution2[0])
    expression_sinrz = mass_replace("sinrz=" + str(sinrz1) + ";")
    
        
    #print("Transforming [0,1,0] we get:")
    testvec = sp.Matrix([0,1,0])         
    testvec1 = Reegc1 * testvec        
    testvec2 = Reegc2 * testvec
    
    zeroVec = testvec2-testvec1  
    solution3 =sp.solve(zeroVec[2].subs(sinry, sinry1), sinrx)
    sinrx1 = sp.simplify(solution3[0])
    expression_sinrx = mass_replace("sinrx=" + str(sinrx1) + ";")

        
    rx = sp.asin(sinrx)
    ry = sp.asin(sinry)
    rz = sp.asin(sinrz)
    
    drx_dsinrx = sp.simplify(rx.diff(sinrx))    
    dry_dsinry = sp.simplify(ry.diff(sinry))
    drz_dsinrz = sp.simplify(rz.diff(sinrz))

    print("//End effector position (ep=f(q)):")
    print(expression_sinrx) 
    print(expression_sinry)
    print(expression_sinrz) 
    print("")
    
    eo = sp.zeros(3)
    eo[0] = rx
    eo[1] = ry
    eo[2] = rz


    for i in range(0, 3):
        print(mass_replace("e[" + str(i+3) + "] = " + str(eo[i]) + ";"))
    print("")         
                       
    
    print("//Jacobian (Orientation):")
    esino = sp.zeros(3)
    esino[0] = sinrx1
    esino[1] = sinry1
    esino[2] = sinrz1
    expression_drx_dsinrx = "drx_dsinrx = " + str(drx_dsinrx) + ";"
    expression_dry_dsinry = "dry_dsinry = " + str(dry_dsinry) + ";"
    expression_drz_dsinrz = "drz_dsinrz = " + str(drz_dsinrz) + ";"        
    print(expression_drx_dsinrx)        
    print(expression_dry_dsinry)
    print(expression_drz_dsinrz)
    print("")
    Jsino = sp.zeros(3,3)
 
    for i in range(0, 3):
        for j in range(0, 3):
            Jsino[i,j] = esino[i].diff(q[j+3])
            #print(mass_replace("Jsino[" + str(i) + "][" + str(j) + "] = " + str(Jsino[i,j]) + ";"))
    #print("")
    
    
    
    de_dsin = sp.zeros(3,3)    
    de_dsin[0,0] = sp.Symbol('drx_dsinrx')
    de_dsin[1,1] = sp.Symbol('dry_dsinry')
    de_dsin[2,2] = sp.Symbol('drz_dsinrz')    
        
    Jo = de_dsin * Jsino
    
    
    for i in range(0, 3):
        for j in range(0, 3):
            print(mass_replace("Jo[" + str(i) + "][" + str(j) + "] = " + str(Jo[i,j]) + ";"))
    print("")


def calc_orient_matrix_v2(): 



    #rx = sp.Symbol('rx')
    #ry = sp.Symbol('ry')
    #rz = sp.Symbol('rz')
        
    sinrx = sp.Symbol('sinrx')
    sinry = sp.Symbol('sinry')    
    sinrz = sp.Symbol('sinrz')    

    #cosrx = sp.Symbol('cosrx')
    #cosry = sp.Symbol('cosry')
    #cosrz = sp.Symbol('cosrz')

    cosrx = sp.sqrt(1-sinrx**2)
    cosry = sp.sqrt(1-sinry**2)
    cosrz = sp.sqrt(1-sinrz**2)
    
    
    R0gc = sp.Matrix([[sp.cos(q[0]), -sp.sin(q[0]), 0],
                      [sp.sin(q[0]),  sp.cos(q[0]), 0],
                      [    0,      0, 1]])
 
    R10 = sp.Matrix([[1,     0,      0],
                     [0, sp.cos(q[1]), -sp.sin(q[1])],
                     [0, sp.sin(q[1]),  sp.cos(q[1])]])

    R21 = sp.Matrix([[1,     0,      0],
                     [0, sp.cos(q[2]), -sp.sin(q[2])],
                     [0, sp.sin(q[2]),  sp.cos(q[2])]])
 
    R32 = sp.Matrix([[ sp.cos(q[3]), 0, sp.sin(q[3])],
                     [     0, 1,     0],
                     [-sp.sin(q[3]), 0, sp.cos(q[3])]])
 
    R43 = sp.Matrix([[1,     0,      0],
                     [0, sp.cos(q[4]), -sp.sin(q[4])],
                     [0, sp.sin(q[4]),  sp.cos(q[4])]])
 
    Ree4 = sp.Matrix([[ sp.cos(q[5]), 0, sp.sin(q[5])],
                      [     0, 1,     0],
                      [-sp.sin(q[5]), 0, sp.cos(q[5])]])
                           

    Rx = sp.Matrix([[1,      0,      0],
                     [0, cosrx, -sinrx],
                     [0, sinrx,  cosrx]])
                     
                     
    Ry = sp.Matrix([[ cosry, 0, sinry],
                    [     0, 1,     0],
                    [-sinry, 0, cosry]])   
                    
                    
    Rz = sp.Matrix([[cosrz, -sinrz, 0],
                    [sinrz,  cosrz, 0],
                    [    0,      0, 1]])                
     
     
    # Rotation transformation 1
    Reegc_f_q = R0gc * R10 * R21 * R32 * R43 * Ree4
    
    #Rotation transformation 2
    Reegc_f_r = Rz * Ry * Rx    
    
    #for i in range(0, 3):
    #    for j in range(0, 3):
    #        print(mass_replace_for_doc("Reegc(f(rxryrz)[" + str(i) + "][" + str(j) + "] = " + str(Reegc2[i,j]) + ";"))
    #print("")
    
    
    #print("Transforming [1,0,0] we get:")
    print("") 
    solution1 = sp.solve(Reegc_f_q[2,0] - Reegc_f_r[2,0], sinry)
    sinry1 = sp.simplify(solution1[0])
    expression_sinry = mass_replace("sinry=" + str(sinry1) + ";")
     
    
    #print("And:")
    solution2 = Reegc_f_q[1,0] - Reegc_f_r[1,0]
    solution2 =sp.solve(solution2.subs(sinry, sinry1), sinrz)
    sinrz1 = sp.simplify(solution2[0])
    expression_sinrz = mass_replace("sinrz=" + str(sinrz1) + ";")
    
        
    solution3 = Reegc_f_q[2,1] - Reegc_f_r[2,1]
    solution3 =sp.solve(solution3.subs(sinry, sinry1), sinrx)
    sinrx1 = sp.simplify(solution3[0])
    expression_sinrx = mass_replace("sinrx=" + str(sinrx1) + ";")

        
    rx = sp.asin(sinrx)
    ry = sp.asin(sinry)
    rz = sp.asin(sinrz)
    
    drx_dsinrx = sp.simplify(rx.diff(sinrx))    
    dry_dsinry = sp.simplify(ry.diff(sinry))
    drz_dsinrz = sp.simplify(rz.diff(sinrz))

    print("//End effector orientation (eo=f(q)):")
    print(expression_sinrx) 
    print(expression_sinry)
    print(expression_sinrz) 
    print("")
    
    eo = sp.zeros(3)
    eo[0] = rx
    eo[1] = ry
    eo[2] = rz


    for i in range(0, 3):
        print(mass_replace("e[" + str(i+3) + "] = " + str(eo[i]) + ";"))
    print("")         
                       
    
    print("//Jacobian (Orientation):")
    esino = sp.zeros(3)
    esino[0] = sinrx1
    esino[1] = sinry1
    esino[2] = sinrz1
    expression_drx_dsinrx = "drx_dsinrx = " + str(drx_dsinrx) + ";"
    expression_dry_dsinry = "dry_dsinry = " + str(dry_dsinry) + ";"
    expression_drz_dsinrz = "drz_dsinrz = " + str(drz_dsinrz) + ";"        
    print(expression_drx_dsinrx)        
    print(expression_dry_dsinry)
    print(expression_drz_dsinrz)
    print("")
    Jsino = sp.zeros(3,3)
 
    for i in range(0, 3):
        for j in range(0, 3):
            Jsino[i,j] = esino[i].diff(q[j+3])
            #print(mass_replace("Jsino[" + str(i) + "][" + str(j) + "] = " + str(Jsino[i,j]) + ";"))
    #print("")
    
    
    
    de_dsin = sp.zeros(3,3)    
    de_dsin[0,0] = sp.Symbol('drx_dsinrx')
    de_dsin[1,1] = sp.Symbol('dry_dsinry')
    de_dsin[2,2] = sp.Symbol('drz_dsinrz')    
        
    Jo = de_dsin * Jsino
    
    
    for i in range(0, 3):
        for j in range(0, 3):
            print(mass_replace("Jo[" + str(i) + "][" + str(j) + "] = " + str(Jo[i,j]) + ";"))
    print("")




def calc_q22():
    #Calculate sin(q22) and cos(q22) <= f(q[1],q[2])
    q1q2 = sp.Symbol('q2-q1')   
    sq22 = sp.Symbol('sq22')
    cq22 = sp.sqrt(1-sq22**2)
    dcq22_dsq22 = sp.simplify(cq22.diff(sq22))
    q22   = sp.asin(sq22)
    dq22_dsq22  = sp.simplify(q22.diff(sq22))
    p2 = sp.Matrix([0,-r2 *sp.cos(q1q2),-r2*sp.sin(q1q2)])
    p22 = sp.Matrix([0,-r22 *cq22,l1-r22*sq22])
    p2p22 = p22-p2
    distance_sq = p2p22[0]**2 + p2p22[1]**2 + p2p22[2]**2
    sq22 = sp.simplify(sp.solve(b22**2-distance_sq, sq22))
    print("Solutions for sq22")        
    print(mass_replace_for_doc(str(sq22)))
        
    sq22 = sq22[0];
    #Print as friendly as possible for C++
    expression_sinq22 = "sq22 = " + mass_replace(str(sinq22)) + ";" #.evalf()
    expression_cosq22 = "cq22 = " + mass_replace(str(cosq22)) + ";"
    expression_q22 = "q22 = " + mass_replace(str(q22)) + ";"

    
    print("//sq22, cq22 and q22:")        
    print(expression_sinq22)
    print("")
    print(expression_cosq22)
    print("")
    print(expression_q22)
    print("")

    print("//Partial derivates:")
    dsq22 = sp.simplify(sq22.diff(q1q2)) 
    expression_dsq22 = "dsq22_dq0q1 = " + mass_replace(str(dsq22)) + ";" #.evalf()       
    print(expression_dsq22)
    print("")
    expression_dcq22 = "dcq22_dsq22 = " + mass_replace(str(dcq22_dsq22)) + ";"
    print(expression_dcq22)
    print("")
    expression_dq22 = "dq22_dsq22 = " + mass_replace(str(dq22_dsq22)) + ";"
    print(expression_dq22)
    print("")





def mass_replace( expression ):
    for i in range (0,7):
        old = "cos(q" + str(i) + ")"    
        new = "cosq[" + str(i) + "]"
        expression = expression.replace(old,new)
        old = "sin(q" + str(i) + ")"    
        new = "sinq[" + str(i) + "]"
        expression = expression.replace(old,new)
        old = "q" + str(i)    
        new = "q[" + str(i) + "]"
        expression = expression.replace(old,new)
    
        
    old = "q[2]"    
    new = "q22"
    expression = expression.replace(old,new)    
    old = "sin(q[1] + q22)"    
    new = "sinq1q22"
    expression = expression.replace(old,new)
    old = "cos(q[1] + q22)"    
    new = "cosq1q22"
    expression = expression.replace(old,new)        
    old = "sin(q2-q1)"    
    new = "sq1q2"
    expression = expression.replace(old,new)  
    old = "cos(q2-q1)"    
    new = "cq1q2"
    expression = expression.replace(old,new)
    return expression;        
    
def mass_replace_for_doc( expression ):
    for i in range (0,7):
        old = "cos(q" + str(i) + ")"    
        new = "cq" + str(i)
        expression = expression.replace(old,new)
        old = "sin(q" + str(i) + ")"    
        new = "sq" + str(i)
        expression = expression.replace(old,new)    
    old = "**"    
    new = "^"
    expression = expression.replace(old,new)
    old = "*"    
    new = " cdot "
    expression = expression.replace(old,new)      
    old = "sin(q[1] + q22)"    
    new = "sq1q22"
    expression = expression.replace(old,new)
    old = "cos(q[1] + q22)"    
    new = "cq1q22"
    expression = expression.replace(old,new)  
    old = "sin(q2-q1)"    
    new = "sq1q2"
    expression = expression.replace(old,new)  
    old = "cos(q2-q1)"    
    new = "cq1q2"
    expression = expression.replace(old,new)
    return expression; 
    
    
def misc_solve():
    x = sp.Symbol('x')   
    y = sp.Symbol('y')
    r = sp.Symbol('R')
    l = sp.Symbol('L')
    
    
    ysq = - x**2 + r**2
    equation = (x-r)**2 + ysq - l**2
    solution = sp.simplify(sp.solve(equation, x))
    #solution = solution[0];
    print("Solution")           
    for i in range (0,1):
        print(solution[i])
        print("")