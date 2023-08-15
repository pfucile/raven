#! /usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from scipy.optimize import curve_fit
from matplotlib import rcParams
from matplotlib.ticker import StrMethodFormatter
import tkinter as tk
from tkinter import filedialog
from multiprocessing import Pool
import threading
import math
import copy
from scipy import signal

lock = threading.Lock()

def select_file(directory):
    root = tk.Tk()
    root.withdraw()
    file_path = filedialog.askopenfilename(initialdir=directory, title="Select a File")
    root.destroy()
    return file_path

selected_file = select_file("../Documents/")

def actual_path_processing(args):
    line, first_line_found, time_zero, TBP_total = args
    if ('points' not in line.lower()) and ('x' not in line.lower()) and first_line_found:
        split_line = [float(col) for col in line.split()]
        if (split_line[3] - time_zero) <= TBP_total:
            split_line[3] = split_line[3] - time_zero
            return split_line

file = open(selected_file, 'r')
goal_array = []
recorded_array = []
TBP_total = 0
first_line_found = False
print("Reading the file:", selected_file)
line_num = 0
section = None

lines = file.readlines()  # Read all lines before processing

for line in lines:
    line_num += 1
    if 'goal' in line.lower():
        section = 1
    elif 'actual' in line.lower():
        section = 0
    elif section == 1 and ('points' not in line.lower()) and ('x' not in line.lower()):
        split_line = [float(col) for col in line.split()]
        TBP_total += split_line[3]
        split_line[3] = TBP_total
        goal_array.append(split_line)
    elif section == 0 and ('points' not in line.lower()) and ('x' not in line.lower()) and not first_line_found:
        first_line_found = True
        split_line = [float(col) for col in line.split()]
        time_zero = split_line[3]
        split_line[3] = 0.000
        recorded_array.append(split_line)
        print ( "tbp total = ", TBP_total)
        print ( "time zero  = ", time_zero)
        print ( "first line found = ", first_line_found)
        break
actual_path_section = lines[line_num:len(lines)]
file.close()

#using pool process to do the loading of the file to the array in a multi-threaded way, making the code faster
with Pool() as pool:
    temp_array = pool.map(actual_path_processing, [(line, first_line_found, time_zero, TBP_total) for line in actual_path_section])
    temp_array = [x for x in temp_array if x is not None]
    recorded_array += temp_array

recorded_array = np.array(recorded_array)
recorded_array_shifted = copy.deepcopy(recorded_array) # for storing the shifted array
goal_array = np.array(goal_array)


#doing interpolation on the goal array to introduce more points
x_new = np.interp(recorded_array[:,3], goal_array[:,3], goal_array[:,0])
y_new = np.interp(recorded_array[:,3], goal_array[:,3], goal_array[:,1])
z_new = np.interp(recorded_array[:,3], goal_array[:,3], goal_array[:,2])
#to shift the recorded array in time axis such that the ideal graph and the actual graph is more aligned
region_of_intrest_start =350
region_of_intrest_end = 1500
sweep = 300
region_of_intrest = recorded_array[region_of_intrest_start:region_of_intrest_end]

shift_error_array = []
for shift in range(region_of_intrest_start-sweep,region_of_intrest_start+sweep,1):
    current_alligement_x= x_new[shift:shift + len(region_of_intrest)]
    current_alligement_y= y_new[shift:shift + len(region_of_intrest)]
    current_alligement_z= z_new[shift:shift + len(region_of_intrest)]
    #to calculate the RMS error for this particular value of shift for each axis
    error_x = 0
    error_y = 0
    error_z =0
    for i in range(region_of_intrest_end -region_of_intrest_start):
        error_x = error_x + (current_alligement_x[i]-region_of_intrest[i,0])*(current_alligement_x[i]-region_of_intrest[i,0])
        error_y = error_y + (current_alligement_y[i]-region_of_intrest[i,1])*(current_alligement_y[i]-region_of_intrest[i,1])
        error_z = error_z + (current_alligement_z[i]-region_of_intrest[i,2])*(current_alligement_z[i]-region_of_intrest[i,2])
    ##the product of rms errors of each axis
    error = math.sqrt(error_x/(len(region_of_intrest))) * math.sqrt(error_y/(len(region_of_intrest))) *math.sqrt(error_z/(len(region_of_intrest)))
    shift_error_array.append([shift,error])
shift_error_array = np.array(shift_error_array)
shift_num = shift_error_array[np.argmin(shift_error_array[:,1]),0]
time_shift = recorded_array[int(shift_num),3] - recorded_array[ region_of_intrest_start,3]
recorded_array_shifted [:, 3] += time_shift


#to calculate the velocity of endeffector and storing that data in an array
vel_act = []
for i in range (2,len(recorded_array[:,0])-2,1) :
    velocity = math.sqrt((recorded_array[i,0]-recorded_array[i-1,0])**2  +  (recorded_array[i,1]-recorded_array[i-1,1])**2 + (recorded_array[i,2]-recorded_array[i-1,2])**2)/(recorded_array[i,3]-recorded_array[i-1,3])
    vel_act.append([float(velocity),(recorded_array[i,3]+recorded_array[i-1,3])/2])
vel_act = [x for x in vel_act if x[0] != 0]
vel_act = np.array(vel_act)
#to do a curve fitting
trend = np.polyfit(recorded_array[:,3], recorded_array[:,2], 1)
fit = np.poly1d(trend)

vel_act_shifted = copy.deepcopy(vel_act) # for storing the shifted velocity array
vel_act_shifted [:, 1] += time_shift


## now lets calculate the error in each axis assuming the alignment process is successful
print ("Calculating the error ")

#doing interpolation on the goal array to introduce more points
x_new= np.interp(recorded_array_shifted[:,3], goal_array[:,3], goal_array[:,0])
y_new = np.interp(recorded_array_shifted[:,3], goal_array[:,3], goal_array[:,1])
z_new = np.interp(recorded_array_shifted[:,3], goal_array[:,3], goal_array[:,2])

#for storing the error for x,y and z axis along with the time
error_arr = np.empty(((0, (4))),dtype=float)

for i in range(0,np.size(recorded_array_shifted,0),1):
    error_x = x_new[i]-recorded_array_shifted[i,0]
    error_y = y_new[i]-recorded_array_shifted[i,1]
    error_z = z_new[i]-recorded_array_shifted[i,2]
    error_arr = np.append(error_arr,[[error_x,error_y,error_z,recorded_array_shifted[i,3]]],axis=0)



fig = plt.figure()
ax = Axes3D(fig)
#this section helps find the centre of the dat and keeps the plot centred and make sure all the axis are in the same scale
plot_len  = max(np.ptp(recorded_array[:,0],axis = 0), np.ptp(recorded_array[:,1],axis = 0), np.ptp(recorded_array[:,2],axis = 0))
ax.set_xlim3d(np.min(recorded_array[:,0])-((plot_len-np.ptp(recorded_array[:,0],axis = 0))/2)-0.001,np.max(recorded_array[:,0])+((plot_len-np.ptp(recorded_array[:,0],axis = 0))/2)+0.001)
ax.set_ylim3d(np.min(recorded_array[:,1])-((plot_len-np.ptp(recorded_array[:,1],axis = 0))/2)-0.001,np.max(recorded_array[:,1])+((plot_len-np.ptp(recorded_array[:,1],axis = 0))/2)+0.001)
ax.set_zlim3d(np.min(recorded_array[:,2])-((plot_len-np.ptp(recorded_array[:,2],axis = 0))/2)-0.001,np.max(recorded_array[:,2])+((plot_len-np.ptp(recorded_array[:,2],axis = 0))/2)+0.001)
ax.xaxis.labelpad=30
ax.set_xlabel('$X (m)$', fontsize=30)
ax.xaxis._axinfo['label']
ax.yaxis.labelpad=30
ax.set_ylabel('$Y (m)$',fontsize=30)
ax.yaxis._axinfo['label']
ax.zaxis.labelpad=35
ax.set_zlabel('$Z (m)$',fontsize=30)
ax.zaxis._axinfo['label']
for t in ax.xaxis.get_major_ticks(): t.label.set_fontsize(20)
ax.xaxis.set_major_formatter(StrMethodFormatter('{x:,.3f}')) # 2 decimal places
for t in ax.yaxis.get_major_ticks(): t.label.set_fontsize(20)
ax.yaxis.set_major_formatter(StrMethodFormatter('{x:,.3f}')) # 2 decimal places
for t in ax.zaxis.get_major_ticks(): t.label.set_fontsize(20)
for t in ax.zaxis.get_major_ticks(): t.label.set_horizontalalignment( 'left')
ax.zaxis.set_major_formatter(StrMethodFormatter('{x:,.3f}')) # 2 decimal places
ax.plot3D(recorded_array[:,0], recorded_array[:,1], recorded_array[:,2], color='red', label='Actual path')
ax.plot3D(goal_array[:,0], goal_array[:,1], goal_array[:,2], color='grey', label='Ideal path')
ax.plot3D(goal_array[:,0], goal_array[:,1], goal_array[:,2], 'o', color='grey')
#plt.legend()
plt.title('Plot of the motion of the robot')
plt.show()

plt.plot(recorded_array[:,2], recorded_array[:,0], 'b-', label='data')
plt.plot(goal_array[:,2], goal_array[:,0], 'r-', label='ideal')
plt.xlabel('z')
plt.ylabel('x')
plt.legend()
plt.show()


x_fit = np.linspace(recorded_array[0,3], recorded_array[len(recorded_array[:,3])-1,3])
#plt.plot(x_fit, fit(x_fit), "r--", label='fitting')

plt.plot(recorded_array[:,3], recorded_array[:,2], 'b-', label='data')
#plt.plot(recorded_array[:,3], recorded_array[:,2], marker=".", markersize=20, label='data')
plt.plot(goal_array[:,3],goal_array[:,2], 'r--', label='ideal')
plt.plot(goal_array[:,3],goal_array[:,2], marker=".", markersize=20, label='ideal')
#plt.plot(recorded_array[:,3], z_new, '-', label='data')
plt.plot(recorded_array_shifted[:,3], recorded_array_shifted[:,2], 'g-', label='shifted')
plt.xlabel('Time (s)',fontsize=30)
plt.ylabel('Z (m)',fontsize=30)
#plt.gca().yaxis.set_major_formatter(StrMethodFormatter('{x:,.3f}')) # 2 decimal places
plt.yticks([0.01,0.02,0.03,0.04,0.05,0.06,0.07,0.08,0.09], ['0.010', '0.020', '0.030','0.040', '0.050', '0.060', '0.070', '0.080', '0.090',],rotation=0)  # Set text labels and properties.
plt.xticks(fontsize=20)
plt.yticks(fontsize=20)
plt.legend()
plt.show()

plt.plot(recorded_array[:,3], recorded_array[:,0], 'b-', label='data')
plt.plot(goal_array[:,3],goal_array[:,0], 'r--', label='ideal')
plt.plot(goal_array[:,3],goal_array[:,0], marker=".", markersize=20, label='ideal')
plt.plot(recorded_array_shifted[:,3], recorded_array_shifted[:,0], 'g-', label='Shifted')
plt.xlabel('Time (s)',fontsize=30)
plt.ylabel('X (m)',fontsize=30)
plt.xticks(fontsize=20)
plt.yticks(fontsize=20)
plt.legend()
plt.show()

plt.plot(recorded_array[:,3], recorded_array[:,1], 'b-', label='data')
plt.plot(goal_array[:,3],goal_array[:,1], 'r--', label='ideal')
plt.plot(goal_array[:,3],goal_array[:,1], marker=".", markersize=20, label='ideal')
plt.plot(recorded_array_shifted[:,3], recorded_array_shifted[:,1], 'g-', label='Shifted')
plt.xlabel('Time (s)',fontsize=30)
plt.ylabel('Y (m)',fontsize=30)
plt.xticks(fontsize=20)
plt.yticks(fontsize=20)
plt.legend()
plt.show()

#for plotting the velocity
plt.plot(vel_act[:,1],vel_act[:,0], 'b-', label='velocity of end-effector')
plt.xlabel('Time (s)',fontsize=30)
plt.ylabel('Velocity (m/s)',fontsize=30)
plt.xticks(fontsize=20)
plt.yticks(fontsize=20)
plt.legend()
plt.show()

#for plotting the error
#plt.ylim(-15,15)
plt.plot(error_arr[:,3], error_arr[:,0] *1000, 'r-', label='X axis error')
plt.plot(error_arr[:,3], error_arr[:,1] *1000, 'g-', label='Y axis error')
plt.plot(error_arr[:,3], error_arr[:,2] *1000, 'b-', label='Z axis error')
#plt.plot(vel_act_shifted[:,1],vel_act_shifted[:,0]*100-20, '-', label='velocity of end-effector(m/s)')
#plt.plot(goal_array[:,3],goal_array[:,0]*0, 'o', label='trajectory points')
plt.xlabel('Time (s)',fontsize=30)
plt.ylabel('error (mm)',fontsize=30)
plt.xticks(fontsize=20)
plt.yticks(fontsize=20)
plt.legend()
plt.axhline(0, color='black')
plt.legend()
plt.show()



fig, axs = plt.subplots(2, 2)
axs[0, 0].plot(recorded_array[:,3], recorded_array[:,2], 'b-', label='data')
axs[0, 0].plot(goal_array[:,3],goal_array[:,2], linestyle = 'dotted',color = "red", label='ideal')
axs[0, 0].set_title('Z Axis')
axs[0, 0].set(xlabel='Time(s)', ylabel='Z(m)')


axs[0, 1].plot(recorded_array[:,3], recorded_array[:,0], 'b-', label='data')
axs[0, 1].plot(goal_array[:,3],goal_array[:,0], 'r--', label='ideal')
axs[0, 1].set_title('X Axis')
axs[0, 1].set(xlabel='Time(s)', ylabel='X(m)')



axs[1, 0].plot(recorded_array[:,3], recorded_array[:,1], 'b-', label='data')
axs[1, 0].plot(goal_array[:,3],goal_array[:,1], 'r--', label='ideal')
axs[1, 0].set_title('Y Axis')
axs[1, 0].set(xlabel='Time(s)', ylabel='Y(m)')




axs[1, 1].plot(recorded_array[:,3], recorded_array[:,2], 'b-', label='data')
axs[1, 1].plot(goal_array[:,3],goal_array[:,2], 'r-', label='ideal')
axs[1, 1].set_title('Z Axiszoomed')
axs[1, 1].set(xlabel='Time(s)', ylabel='Y(m)')
plt.legend()
plt.show()



