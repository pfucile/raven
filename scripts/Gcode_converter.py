#! /usr/bin/env python3
#! /usr/bin/env python

import math

#this code shifts the position of the printing to left side of the table while converting the gcode
points = []
#reading the path from the file
file = open('../xarm_ws/src/raven/scripts/to_convert.txt', 'r')
[speed,X_val,Y_val,Z_val,E_val] = [0,0,0,0,0]
Converted_file = open("../xarm_ws/src/raven/scripts/Converted_file.txt", "w")
Converted_file.write("#G(0/1) F(m/min)    X(m)      Y(m)      Z(m)       E(mm)       Rx        Ry       Rz    seg#\n")


Rx = (math.pi)
Ry = 0.0
Rz = 0.0
s = 0

def check_relative_cooordinate ():
    file = open('../xarm_ws/src/raven/scripts/to_convert.txt', 'r')
    relative1  = "M83"
    relative2  = "M91"
    lines = file.readlines()
    output = 0
    for line in lines:
        if line.find(relative1)!= -1 or line.find(relative2) != -1 :
            output = 1
            #print (output, line.find( relative1))
    return output

if check_relative_cooordinate () !=1:
    print("absolute coordinates")
    for line in file:
        flag = False
        if line: #avoid blank lines
            if (line.startswith(';LAYER:')) or (line.startswith(';SEGMENT:')) :
                s = s + 1
            elif (line.startswith('G0')) or (line.startswith('G1')) or (line.startswith('G01')) or (line.startswith('G00')) :
                #finding the size of the line
                len_line = len(line)
                #finding the number of times the charactors F,X,Y,Z,E are present in the line
                F_num = line.count('F')
                X_num = line.count('X')
                Y_num = line.count('Y')
                Z_num = line.count('Z')
                E_num = line.count('E')
                #using the indexes find the feed rate
                if line.startswith('G1') or (line.startswith('G01')) :
                    g = 1
                else :
                    g = 0
                if F_num ==1 :
                    F_inx = line.index('F')
                    if X_num ==1:
                        X_inx = line.index('X')
                        speed = line[F_inx+1:X_inx].strip(' ').strip('\n ')
                    elif Y_num ==1:
                        Y_inx = line.index('Y')
                        speed = line[F_inx+1:Y_inx].strip(' ').strip('\n ')
                    elif Z_num ==1:
                        Z_inx = line.index('Z')
                        speed = line[F_inx+1:Z_inx].strip(' ').strip('\n ')
                    elif E_num ==1:
                        E_inx = line.index('E')
                        speed = line[F_inx+1:E_inx].strip(' ').strip('\n ')
                    else:
                        speed = line[F_inx+1:len_line].strip(' ').strip('\n ')
                #using the indexes find the X value
                if X_num ==1:
                    X_inx = line.index('X')
                    if Y_num ==1:
                        Y_inx = line.index('Y')
                        X_val = line[X_inx+1:Y_inx].strip(' ').strip('\n ')
                    elif Z_num ==1:
                        Z_inx = line.index('Z')
                        X_val = line[X_inx+1:Z_inx].strip(' ').strip('\n ')
                    elif E_num ==1:
                        E_inx = line.index('E')
                        X_val = line[X_inx+1:E_inx].strip(' ').strip('\n ')
                    else:
                        X_val = line[X_inx+1:len_line].strip(' ').strip('\n ')
                #using the indexes find the Y value
                if Y_num ==1:
                    Y_inx = line.index('Y')
                    if Z_num ==1:
                        Z_inx = line.index('Z')
                        Y_val = line[Y_inx+1:Z_inx].strip(' ').strip('\n ')
                    elif E_num ==1:
                        E_inx = line.index('E')
                        Y_val = line[Y_inx+1:E_inx].strip(' ').strip('\n ')
                    else:
                        Y_val = line[Y_inx+1:len_line].strip(' ').strip('\n ')
                #using the indexes find the Z value
                if Z_num ==1:
                    Z_inx = line.index('Z')
                    if E_num ==1:
                        E_inx = line.index('E')
                        Z_val = line[Z_inx+1:E_inx].strip(' ').strip('\n ')
                    else:
                        Z_val = line[Z_inx+1:len_line].strip(' ').strip('\n ')
                #using the indexes find the E value
                if E_num ==1:
                    E_inx = line.index('E')
                    if E_val == line[E_inx+1:len_line].strip(' ').strip('\n '):
                        g = 0
                        E_val = line[E_inx+1:len_line].strip(' ').strip('\n ')
                    else:
                        E_val = line[E_inx+1:len_line].strip(' ').strip('\n ')

                # Define the number of decimal places you want for each value
                decimal_places = 8
                # Format each value as a string with the desired number of decimal places
                formatted_line = (
                	str(g)+" "+
                	f"{float(speed)/1000:.{decimal_places}f} "
                	f"{float(X_val)/1000:.{decimal_places}f} "
                	f"{float(Y_val)/1000:.{decimal_places}f} "
                	f"{float(Z_val)/1000:.{decimal_places}f} "
                	f"{float(E_val):.{decimal_places}f} "
                	f"{Rx:.{decimal_places}f} "
                	f"{Ry:.{decimal_places}f} "
                	f"{Rz:.{decimal_places}f} "
                	+str(s)+"\n"
                )
                #converting the values to metres and storing insid the set named points
                if (X_num > 0 or Y_num > 0 or Z_num>0):
                     Converted_file.write(formatted_line)
                #Converted_file.write(str(g)+" "+str(round(float(speed)/1000,8))+" "+str(round((float(X_val)/1000),8))+" "+str(round((float(Y_val)/1000),8))+" "+str(round((float(Z_val)/1000),8))+" "+str(round(float(E_val)*0.8,8))+" "+str(Rx) +" "+str(Ry) +" "+str(Rz)+ " "+str(s)+"\n")
                #print (speed,X_val,Y_val,Z_val,E_val)
            else:
                pass
    #print (points)
else :
    E_float = 0
    print ("relative coordinates")
    for line in file:
        flag = False
        if line: #avoid blank lines
            if (line.startswith(';LAYER:')) or (line.startswith(';SEGMENT:')) : #to find the segment number or in conventional printing layer number
                s = s + 1
            elif (line.startswith('G0')) or (line.startswith('G1')) or (line.startswith('G01')) or (line.startswith('G00')) :
                #finding the size of the line
                len_line = len(line)
                #finding the number of times the charactors F,X,Y,Z,E are present in the line
                F_num = line.count('F')
                X_num = line.count('X')
                Y_num = line.count('Y')
                Z_num = line.count('Z')
                E_num = line.count('E')
                #using the indexes find the feed rate
                if line.startswith('G1') or (line.startswith('G01')) :
                    g = 1
                else :
                    g = 0
                if F_num ==1 :
                    F_inx = line.index('F')
                    if X_num ==1:
                        X_inx = line.index('X')
                        speed = line[F_inx+1:X_inx].strip(' ').strip('\n ')
                    elif Y_num ==1:
                        Y_inx = line.index('Y')
                        speed = line[F_inx+1:Y_inx].strip(' ').strip('\n ')
                    elif Z_num ==1:
                        Z_inx = line.index('Z')
                        speed = line[F_inx+1:Z_inx].strip(' ').strip('\n ')
                    elif E_num ==1:
                        E_inx = line.index('E')
                        speed = line[F_inx+1:E_inx].strip(' ').strip('\n ')
                    else:
                        speed = line[F_inx+1:len_line].strip(' ').strip('\n ')
                #using the indexes find the X value
                if X_num ==1:
                    X_inx = line.index('X')
                    if Y_num ==1:
                        Y_inx = line.index('Y')
                        X_val = line[X_inx+1:Y_inx].strip(' ').strip('\n ')
                    elif Z_num ==1:
                        Z_inx = line.index('Z')
                        X_val = line[X_inx+1:Z_inx].strip(' ').strip('\n ')
                    elif E_num ==1:
                        E_inx = line.index('E')
                        X_val = line[X_inx+1:E_inx].strip(' ').strip('\n ')
                    else:
                        X_val = line[X_inx+1:len_line].strip(' ').strip('\n ')
                #using the indexes find the Y value
                if Y_num ==1:
                    Y_inx = line.index('Y')
                    if Z_num ==1:
                        Z_inx = line.index('Z')
                        Y_val = line[Y_inx+1:Z_inx].strip(' ').strip('\n ')
                    elif E_num ==1:
                        E_inx = line.index('E')
                        Y_val = line[Y_inx+1:E_inx].strip(' ').strip('\n ')
                    else:
                        Y_val = line[Y_inx+1:len_line].strip(' ').strip('\n ')
                #using the indexes find the Z value
                if Z_num ==1:
                    Z_inx = line.index('Z')
                    if E_num ==1:
                        E_inx = line.index('E')
                        Z_val = line[Z_inx+1:E_inx].strip(' ').strip('\n ')
                    else:
                        Z_val = line[Z_inx+1:len_line].strip(' ').strip('\n ')
                #using the indexes find the E value
                if E_num ==1:
                    E_inx = line.index('E')
                    E_val = line[E_inx+1:len_line].strip(' ').strip('\n ')
                    E_float = E_float + float(E_val)
                    #print (E_float)

                # Define the number of decimal places you want for each value
                decimal_places = 8
                # Format each value as a string with the desired number of decimal places
                formatted_line = (
                	str(g)+" "+
                	f"{float(speed)/1000:.{decimal_places}f} "
                	f"{float(X_val)/1000:.{decimal_places}f} "
                	f"{float(Y_val)/1000:.{decimal_places}f} "
                	f"{float(Z_val)/1000:.{decimal_places}f} "
                	f"{float(E_val):.{decimal_places}f} "
                	f"{Rx:.{decimal_places}f} "
                	f"{Ry:.{decimal_places}f} "
                	f"{Rz:.{decimal_places}f} "
                	+str(s)+"\n"
                )
                #converting the values to metres and storing insid the set named points
                Converted_file.write(formatted_line)
                #Converted_file.write(str(g)+" "+str(round(300/1000,8))+" "+str(round((float(X_val)/1000),8))+" "+str(round((float(Y_val)/1000),8))+" "+str(round((float(Z_val)/1000),8))+" "+str(round(E_float,8))+" "+str(Rx) +" "+str(Ry) +" "+str(Rz)+ " "+str(s)+"\n")
                #print (speed,X_val,Y_val,Z_val,E_val)
            else:
                pass

#print (points)
