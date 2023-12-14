#! /usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from scipy.optimize import curve_fit
from scipy.ndimage import gaussian_filter1d
from matplotlib import rcParams
from matplotlib.ticker import StrMethodFormatter
import tkinter as tk
from tkinter import filedialog
from multiprocessing import Pool
import threading
import math
import copy
from cmath import inf
import numba
import os


lock = threading.Lock()

## function definitions
def select_file(directory,title):
    root = tk.Tk()
    root.withdraw()
    file_path = filedialog.askopenfilename(initialdir=directory, title=title)
    root.destroy()
    return file_path

def actual_path_processing(args):
    line, first_line_found, time_zero, TBP_total = args
    if ('points' not in line.lower()) and ('x' not in line.lower()) and first_line_found:
        split_line = [float(col) for col in line.split()]
        if (split_line[3] - time_zero) <= TBP_total:
            split_line[3] = split_line[3] - time_zero
            return split_line

@numba.njit
def get_indices_of_closest_questioned_points(interogators: np.ndarray,questioned: np.ndarray,) -> np.ndarray:
    """For each element in `interogators` get the index of the closest element in set `questioned`.
    """
    res = np.empty(shape=interogators.shape, dtype=np.uint32)
    N = len(interogators)
    M = len(questioned)
    n = m = 0
    closest_left_to_x = -inf
    while n < N and m < M:
        x = interogators[n]
        y = questioned[m]
        if y < x:
            closest_left_to_x = y
            m += 1
        else:
            res[n] = m - (x - closest_left_to_x < y - x)
            n += 1
    while n < N:
        res[n] = M - 1
        n += 1
    return res
def convert_to_np_array(selected_file):
    file = open(selected_file, 'r')
    lines = file.readlines()  # Read all lines before processing
    goal_array = []
    recorded_array = []
    TBP_total = 0
    first_line_found = False
    print("Reading the file:", selected_file)
    line_num = 0
    section = None
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
    with Pool() as pool:
        temp_array = pool.map(actual_path_processing, [(line, first_line_found, time_zero, TBP_total) for line in actual_path_section],chunksize=10)
        temp_array = [x for x in temp_array if x is not None]
        recorded_array += temp_array
    recorded_array = np.array(recorded_array)
    goal_array = np.array(goal_array)
    return goal_array, recorded_array, TBP_total

def allign_in_time(goal_array,recorded_array):
    ## the mechanism to align the two graphs in time axis

    recorded_array_shifted = copy.deepcopy(recorded_array) # for storing the shifted array

    #doing interpolation on the goal array to introduce more points
    x_new = np.interp(recorded_array[:,3], goal_array[:,3], goal_array[:,0])
    y_new = np.interp(recorded_array[:,3], goal_array[:,3], goal_array[:,1])
    z_new = np.interp(recorded_array[:,3], goal_array[:,3], goal_array[:,2])
    #to shift the recorded array in time axis such that the ideal graph and the actual graph is more aligned
    region_of_intrest_start =350
    region_of_intrest_end = 1500
    sweep = 300
    region_of_intrest = recorded_array[region_of_intrest_start:region_of_intrest_end]

    #correlation_array = [] # convolution doesn't seem to work so trying to minimize the error.
    shift_error_array = []
    for shift in range(region_of_intrest_start-sweep,region_of_intrest_start+sweep,1):
        current_alligement_x= x_new[shift:shift + len(region_of_intrest)]
        current_alligement_y= y_new[shift:shift + len(region_of_intrest)]
        current_alligement_z= z_new[shift:shift + len(region_of_intrest)]
        #to calculate the RMS error for this particular value of shift for each axis
        error_x = 0
        error_y = 0
        error_z =0
        for i in range(len(region_of_intrest)):
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
    return recorded_array_shifted
def compensate_fixed_shifts(recorded_array):
    recorded_array [:, 0] += fixed_shifts[0]
    recorded_array [:, 1] += fixed_shifts[1]
    recorded_array [:, 2] += fixed_shifts[2] 
    return recorded_array
def calculate_error(goal_array,recorded_array_shifted):
    ## now let's calculate the error in each axis assuming the alignment process is successful
    print ("Calculating the error ")

    #doing interpolation on the goal array to introduce more points
    x_new = np.interp(recorded_array_shifted[:,3], goal_array[:,3], goal_array[:,0])
    y_new = np.interp(recorded_array_shifted[:,3], goal_array[:,3], goal_array[:,1])
    z_new = np.interp(recorded_array_shifted[:,3], goal_array[:,3], goal_array[:,2])

    #for storing the error for x,y and z axis along with the time
    error_arr = np.empty(((0, (4))),dtype=float)

    for i in range(0,np.size(recorded_array_shifted,0),1):
        error_x = x_new[i]-recorded_array_shifted[i,0]
        error_y = y_new[i]-recorded_array_shifted[i,1]
        error_z = z_new[i]-recorded_array_shifted[i,2]
        error_arr = np.append(error_arr,[[error_x,error_y,error_z,recorded_array_shifted[i,3]]],axis=0)


    # since the get_indices_of_closest_questioned_points function used numba  we need to install numba using the command "pip install numba"
    #finding the index corresponding to each points in goal_array
    start_time = 10.0 #the move to start time used in the raven system
    start_point = get_indices_of_closest_questioned_points(np.array([start_time]), goal_array[:,3])
    start_point = start_point[0]
    index_array = get_indices_of_closest_questioned_points(goal_array[:,3], recorded_array_shifted[:,3])
    return error_arr, index_array, start_point

def calculate_correction(goal_array, index_array,start_point,error_arr,n):
    #calculating the correction for each axis one point at a time
    #to find the correction value for x,y and z axis of each points in the goal array.
    # for this we are using a process similar to convolution using a Gaussian kernal which multiplies over all the values on
    # the recorded_shifted array to obtain a single value of each axis per point
    # the size of the cnvloution kernal can be adjusted using "n"
    correction_array = np.empty(((0, (4))),dtype=float)
    #for creating a kernal with gaussian distribution
    mean = 0.0
    std_dev = 1.0
    x = np.linspace(-3*std_dev, 3*std_dev, n*2+1)  # Range of x values
    gaussian_curve = np.exp(-(x - mean)**2 / (2 * std_dev**2))  # Gaussian PDF
    # Calculate the sum of the elements in the array
    array_sum = np.sum(gaussian_curve)
    # Divide the array by its sum to make the sum of elements 1
    gaussian_curve /= array_sum
    kernal = gaussian_curve
    for i in range (0,np.size(goal_array,0), 1):
        if i > start_point and i < np.size(goal_array,0)-3 :
            correction_x = 0
            correction_y = 0
            correction_z = 0
            for j in range (index_array[i]-n,index_array[i]+n,1):
                if j > index_array[start_point] and j < index_array[np.size(goal_array,0)-2]:
                    correction_x = correction_x + error_arr[j,0]*kernal[j-index_array[i]-n]
                    correction_y = correction_y + error_arr[j,1]*kernal[j-index_array[i]-n]
                    correction_z = correction_z + error_arr[j,2]*kernal[j-index_array[i]-n]
                #to process the points padded with 0
                if j <= index_array[start_point]:
                    correction_x = correction_x
                    correction_y = correction_y
                    correction_z = correction_z
                    correction_point_beginning = i
                if j >= index_array[np.size(goal_array,0)-2]:
                    correction_x = correction_x
                    correction_y = correction_y
                    correction_z = correction_z
            correction_array = np.append(correction_array,[[correction_x,correction_y,correction_z,goal_array[i,3]]],axis=0)
        else:
            correction_array = np.append(correction_array,[[0.0,0.0,0.0,goal_array[i,3]]],axis=0) #to process the points padded with 0
    # processing the correction array to make sure there is uniform correction for the starting and the ending of the Gcode
    adjustemnt_length = correction_point_beginning - start_point
    for k in range (0,adjustemnt_length+5,1):
        correction_array[start_point+k] = correction_array [start_point + adjustemnt_length +5+1]
        correction_array[np.size(goal_array,0)-k-1] = correction_array [np.size(goal_array,0) - adjustemnt_length -1- 5]
    return correction_array

def plot_graphs(goal_array,recorded_array,correction_array,recorded_array_shifted):

    #for plotting the error
    fig = plt.figure()
    plt.plot(goal_array[:,3],goal_array[:,2], 'r-', label='trajectory points')
    plt.plot(goal_array[:,3],goal_array[:,2] + correction_array[:,2], 'g-', label='corrected trajectory points')
    plt.plot(goal_array[:,3],correction_array[:,2], 'b', label='correction')
    plt.plot(recorded_array_shifted[:,3], recorded_array_shifted[:,2], 'y-', label='Shifted')
    plt.xlabel('Time (s)',fontsize=30)
    plt.ylabel('z axis (m)',fontsize=30)
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    plt.legend()
    plt.show()
    return 0
def plot_graphs3D(goal_array,recorded_array,correction_array,recorded_array_shifted):
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
    for t in ax.xaxis.get_major_ticks(): t.label1.set_fontsize(20)
    ax.xaxis.set_major_formatter(StrMethodFormatter('{x:,.3f}')) # 2 decimal places
    for t in ax.yaxis.get_major_ticks(): t.label1.set_fontsize(20)
    ax.yaxis.set_major_formatter(StrMethodFormatter('{x:,.3f}')) # 2 decimal places
    for t in ax.zaxis.get_major_ticks(): t.label1.set_fontsize(20)
    for t in ax.zaxis.get_major_ticks(): t.label1.set_horizontalalignment( 'left')
    ax.zaxis.set_major_formatter(StrMethodFormatter('{x:,.3f}')) # 2 decimal places
    ax.plot3D(recorded_array[:,0], recorded_array[:,1], recorded_array[:,2], color='red', label='Actual path')
    ax.plot3D(goal_array[:,0], goal_array[:,1], goal_array[:,2], color='grey', label='Ideal path')
    ax.plot3D(goal_array[:,0]+correction_array[:,0], goal_array[:,1]+correction_array[:,1], goal_array[:,2]+correction_array[:,2], color='green', label='Corrected path')
    ax.legend()
    plt.title('Plot of the motion of the robot')
    plt.show()
    return 0


def plot_progress_graph(goal_array,set_of_recorded_array,set_of_correction_array,set_of_error_array):
    fig = plt.figure()
    ax = Axes3D(fig)
    recorded_array= set_of_recorded_array[0]
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
    for t in ax.xaxis.get_major_ticks(): t.label1.set_fontsize(20)
    ax.xaxis.set_major_formatter(StrMethodFormatter('{x:,.3f}')) # 2 decimal places
    for t in ax.yaxis.get_major_ticks(): t.label1.set_fontsize(20)
    ax.yaxis.set_major_formatter(StrMethodFormatter('{x:,.3f}')) # 2 decimal places
    for t in ax.zaxis.get_major_ticks(): t.label1.set_fontsize(20)
    for t in ax.zaxis.get_major_ticks(): t.label1.set_horizontalalignment( 'left')
    ax.zaxis.set_major_formatter(StrMethodFormatter('{x:,.3f}')) # 2 decimal places
    ax.plot3D(recorded_array[:,0], recorded_array[:,1], recorded_array[:,2], color='red', label='Actual path_1')
    ax.plot3D(goal_array[:,0], goal_array[:,1], goal_array[:,2], color='grey', label='Ideal path')
    for i in range (1,len(set_of_recorded_array),1):
        recorded_array = set_of_recorded_array[i]
        color = plt.cm.viridis(i/len(set_of_recorded_array))
        ax.plot3D(recorded_array[:,0], recorded_array[:,1], recorded_array[:,2], color=color, label=f'Actual path {i+1}')
    plt.legend()
    plt.title('Plot of the motion of the robot')
    plt.show()

    #for plotting the error
    fig = plt.figure()
    #plt.plot(goal_array[:,3],goal_array[:,2], 'r-', label='trajectory points')
    for i in range (0,len(set_of_correction_array),1):
        correction_array = set_of_correction_array[i]
        color = plt.cm.viridis(i/len(set_of_correction_array))
        plt.plot(correction_array[:,3],correction_array[:,2], color=color, label=f'correction {i+1}')
    plt.xlabel('Time (s)',fontsize=30)
    plt.ylabel('z axis (m)',fontsize=30)
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    plt.legend()
    plt.show()


    #for plotting the X error
    fig = plt.figure()
    #plt.plot(goal_array[:,3],goal_array[:,2], 'r-', label='trajectory points')
    for i in range (0,len(set_of_error_array),1):
        error_array = set_of_error_array[i]
        color = plt.cm.viridis(i/len(set_of_error_array))
        plt.plot(error_array[:,3],error_array[:,0], color=color, label=f'Error {i+1}')
    plt.xlabel('Time (s)',fontsize=30)
    plt.ylabel('x axis (m)',fontsize=30)
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    plt.legend()
    plt.show()

    #for plotting the Y error
    fig = plt.figure()
    #plt.plot(goal_array[:,3],goal_array[:,2], 'r-', label='trajectory points')
    for i in range (0,len(set_of_error_array),1):
        error_array = set_of_error_array[i]
        color = plt.cm.viridis(i/len(set_of_error_array))
        plt.plot(error_array[:,3],error_array[:,1], color=color, label=f'Error {i+1}')
    plt.xlabel('Time (s)',fontsize=30)
    plt.ylabel('x axis (m)',fontsize=30)
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    plt.legend()
    plt.show()

    #for plotting the Z error
    fig = plt.figure()
    #plt.plot(goal_array[:,3],goal_array[:,2], 'r-', label='trajectory points')
    for i in range (0,len(set_of_error_array),1):
        error_array = set_of_error_array[i]
        color = plt.cm.viridis(i/len(set_of_error_array))
        plt.plot(error_array[:,3],error_array[:,2], color=color, label=f'Error {i+1}')
    plt.xlabel('Time (s)',fontsize=30)
    plt.ylabel('z axis (m)',fontsize=30)
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    plt.legend()
    plt.show()



    return 0



def plot_resultant_error(goal_array,set_of_recorded_array,set_of_correction_array,set_of_error_array):
    for i in range (0,len(set_of_error_array),1):
        fig = plt.figure()
        error_array = set_of_error_array[i]
        for j in range (0,len(error_array[:,3]),1):
            error_array[j,0] = math.sqrt(error_array[j,0]**2+error_array[j,1]**2+error_array[j,2]**2)
        color = plt.cm.viridis(i/len(set_of_error_array))
        plt.plot(error_array[:,3],error_array[:,0], color=color, label=f'Error {i+1}')
    plt.xlabel('Time (s)',fontsize=30)
    plt.ylabel('Error (m)',fontsize=30)
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    plt.legend()
    plt.show()
    return 0




def generate_corrected_file(original_traj_file,selected_file,iter_num,goal_array,start_point,correction_array):
    file = open(original_traj_file, 'r')
    print (original_traj_file)
    lines = file.readlines()  # Read all lines before processing
    log_file_name =  os.path.basename(selected_file).strip('.txt')
    dirname =  "../Documents/"
    corrected_file = dirname+original_traj_file_name+"_processed_with_"+ log_file_name +"_iteration_num_"+str(iter_num)+".txt"
    corrected_traj_file = open(corrected_file, 'w')
    line_num = -1

    for line in lines:
        if line_num == -1 and '#' not in line.lower():
            line_num = 0
            print(" starting_time ", goal_array[start_point,3],"index: ", start_point)
            split_line = [float(col) for col in line.split()]
            x_ori_adj = split_line[2] - goal_array[start_point+line_num,0]
            y_ori_adj = split_line[3] - goal_array[start_point+line_num,1]
            z_ori_adj = split_line[4] - goal_array[start_point+line_num,2]
            corrected_traj_file.write("#G(0/1) F(m/min)      X(m)        Y(m)        Z(m)       E(mm)     Rx        Ry          Rz       seg#\n")
        if '#' not in line.lower():
            split_line = [float(col) for col in line.split()]
            #to recalculate the velocity
            if line_num>0:
                #old distance to the point
                X1 = math.sqrt((split_line[2]-previous_original[0])**2  +  (split_line[3]-previous_original[1])**2 + (split_line[4]-previous_original[2])**2)
            previous_original = [split_line[2],split_line[3],split_line[4]]
            split_line[2] =  split_line[2] + correction_array[start_point+line_num,0]
            split_line[3] =  split_line[3] + correction_array[start_point+line_num,1]
            split_line[4] =  split_line[4] + correction_array[start_point+line_num,2]
            #to recalculate the velocity
            if line_num>0 and X1 == 0:
                print ("same",split_line[2])
            if line_num>0 and X1 != 0:
                #new distance to the point
                X2 = math.sqrt((split_line[2]-previous_corrected[0])**2  +  (split_line[3]-previous_corrected[1])**2 + (split_line[4]-previous_corrected[2])**2)
                split_line[1] = split_line[1]*(X2/X1) #recalculating the velocity so that time between points stays the same
            previous_corrected = [split_line[2],split_line[3],split_line[4]]
            data = ""
            for sp in split_line:
                data = data + str(sp)+ " "
            data = data + "\n"
            corrected_traj_file.write(data)
            line_num = line_num+1
    corrected_traj_file.close()
    file.close()
    return corrected_file

def interation_one(original_traj_file,kernal_size):
    selected_file = select_file("../Documents/","select the log file for this trajectory- Iteration number "+str(0))
    goal_array, recorded_array, TBP_total = convert_to_np_array(selected_file)
    #to compensate for the inherent shift in the system
    recorded_array = compensate_fixed_shifts(recorded_array)
    ## the mechanism to align the two graphs in time axis
    recorded_array_shifted = allign_in_time(goal_array,recorded_array)
    ## now let's calculate the error in each axis assuming the alignment process is successful
    error_arr, index_array, start_point = calculate_error(goal_array,recorded_array_shifted)
    #calculating the correction for each axis one point at a time
    correction_array = calculate_correction(goal_array, index_array, start_point,error_arr,kernal_size)
    correction_array =correction_array_smoothing_function(correction_array,3)
    #plot the graphs
    new_file = generate_corrected_file(original_traj_file,selected_file,0,goal_array,start_point,correction_array)
    return goal_array,recorded_array,correction_array,recorded_array_shifted, new_file,error_arr

def interation(iter_num,traj_file,goal_array_0,kernal_size):
    selected_file = select_file("../Documents/","select the log file for this trajectory- Iteration number "+str(iter_num))
    goal_array, recorded_array, TBP_total = convert_to_np_array(selected_file)
    #to compensate for the inherent shift in the system
    recorded_array = compensate_fixed_shifts(recorded_array)
    ## the mechanism to align the two graphs in time axis
    recorded_array_shifted = allign_in_time(goal_array_0,recorded_array)
    ## now let's calculate the error in each axis assuming the alignment process is successful
    error_arr, index_array, start_point = calculate_error(goal_array_0,recorded_array_shifted)
    #calculating the correction for each axis one point at a time
    correction_array = calculate_correction(goal_array_0, index_array, start_point,error_arr,kernal_size)
    correction_array =correction_array_smoothing_function(correction_array,3)
    #plot the graphs
    new_file = generate_corrected_file(traj_file,selected_file,iter_num,goal_array_0,start_point,correction_array)
    return goal_array,recorded_array,correction_array,recorded_array_shifted, new_file,error_arr

def correction_array_smoothing_function(goal_array, correction_array,sigma):
    #first we need to identify the continuous line segments
    #to identify continuous lines segments we are going to measure the change in angle for three consicutive points
    #adding this angle and we consider it as a new segments when angle>max_angle
    max_angle = 15
    angle = np.zeros(1)
    segment_start = 0
    segments = []
    for i in range (0,np.size(goal_array,0)-3,1):
        a = np.array([goal_array[i,0], goal_array[i,1],goal_array[i,2]])
        b = np.array([goal_array[i+1,0], goal_array[i+1,1],goal_array[i,2]])
        c = np.array([goal_array[i+2,0], goal_array[i+2,1],goal_array[i,2]])
        f = b-a
        e = b-c
        angle_radian = np.dot(f, e) / (np.linalg.norm(f) * np.linalg.norm(e))
        if not np.isnan(angle_radian):
            angle = angle+ 180 -np.arccos(angle_radian)*180.0/ np.pi
        if angle > max_angle:
            segments.append(np.array([segment_start, i]))
            segment_start = i
            angle = 0
    #now that we have found the segments we can do the smoothing for each segment individually.
    for set in segments:
        start,end = set
        correction_array[start:end, 0] = gaussian_filter1d(correction_array[start:end, 0], sigma)
        correction_array[start:end, 1] = gaussian_filter1d(correction_array[start:end, 1], sigma)
        correction_array[start:end, 2] = gaussian_filter1d(correction_array[start:end, 2], sigma)
    return correction_array





## actual code
set_of_recorded_array = []
set_of_error_array = []
set_of_correction_array = []
#to compensate for the inherent shift in the system
fixed_shifts = [0.0,0.0,0.0] #Adjust these numbers if there are constant shift/error in the axis which is always present [x,y,z]
original_traj_file = select_file("../Documents/","select the original trajectory file")
original_traj_file_name = os.path.basename(original_traj_file).strip('.txt')
number_of_iterations = 3
n=20
goal_array_0, recorded_array_0,correction_array_0, recorded_array_shifted_0, traj_file,error_arr = interation_one(original_traj_file,n)
aa = plot_graphs(goal_array_0,recorded_array_0,correction_array_0,recorded_array_shifted_0)
aa= plot_graphs3D(goal_array_0,recorded_array_0,correction_array_0,recorded_array_shifted_0)
set_of_recorded_array.append(recorded_array_0)
set_of_error_array.append(error_arr)
set_of_correction_array.append(correction_array_0)
for i in range ( number_of_iterations-1):
    n = n-1
    print("n = ", n)
    goal_array,recorded_array,correction_array,recorded_array_shifted,traj_file,error_arr = interation(i+1,traj_file,goal_array_0,n)
    aa= plot_graphs(goal_array_0,recorded_array,correction_array,recorded_array_shifted)
    aa =plot_graphs3D(goal_array_0,recorded_array,correction_array,recorded_array_shifted)
    #plot_graphs(goal_array,recorded_array,correction_array,recorded_array_shifted)
    set_of_recorded_array.append(recorded_array)
    set_of_error_array.append(error_arr)
    set_of_correction_array.append(correction_array)


selected_file = select_file("../Documents/","select the log file for this trajectory- Iteration number "+str(number_of_iterations+1))
goal_array, recorded_array, TBP_total = convert_to_np_array(selected_file)
#to compensate for the inherent shift in the system
recorded_array = compensate_fixed_shifts(recorded_array)
## the mechanism to align the two graphs in time axis
recorded_array_shifted = allign_in_time(goal_array_0,recorded_array)
## now let's calculate the error in each axis assuming the alignment process is successful
error_arr, index_array, start_point = calculate_error(goal_array_0,recorded_array_shifted)
#calculating the correction for each axis one point at a time
correction_array = calculate_correction(goal_array_0, index_array, start_point,error_arr,n)
correction_array =correction_array_smoothing_function(correction_array,3)
set_of_recorded_array.append(recorded_array)
set_of_error_array.append(error_arr)
#plotting all the trajectories together
plot_progress_graph(goal_array_0,set_of_recorded_array,set_of_correction_array,set_of_error_array)
plot_resultant_error(goal_array_0,set_of_recorded_array,set_of_correction_array,set_of_error_array)
