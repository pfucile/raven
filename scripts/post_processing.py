#! /usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.pyplot as plt
#import matplotlib
#matplotlib.use('Agg')
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
import time
from datetime import datetime
from optimizer_config import *
from raven_optimizer_functions import *

lock = threading.Lock()


    
#to slect the Gcode file to be optimized 
original_traj_file = select_file(traject_file_location,"select the original trajectory file")
original_traj_file_name = os.path.basename(original_traj_file).strip('.txt')
original_traj_file_for_later = original_traj_file_name
#loading the Gcode into an array
Gcode = write_gcode_to_array(original_traj_file)
segmentation_array = find_segments(Gcode)
Optimized_Gcode = np.empty(((0, (10))),dtype=float)
Segment_number = 0
print_stat_sub = rospy.Subscriber('/print_stat', String, print_stat_callback)
datalogger_stat_sub = rospy.Subscriber('/datalogger_stat', String, datalogger_stat_callback)
# Create a separate thread for publishing optimization_status
publish_thread = threading.Thread(target=publish_message)
publish_thread.start()
for segment in segmentation_array:
    print("Starting the optimization of the segment number : ", Segment_number + 1 )
    Segment_class = SegmentOptimizer(Gcode, segment[0], segment[1])
    number_of_iterations = 5
    initial_kernal_number = 20
    Optmized_Segment = Segment_class. Optimize(initial_kernal_number,number_of_iterations)
    Optmized_Segment[:, 9] = Segment_number
    Optimized_Gcode = np.vstack((Optimized_Gcode, Optmized_Segment))
    Segment_number = Segment_number + 1
    

# to set optimization_status to "completed"
optimization_status = "completed"
time.sleep(5)

print("optmization of Gcode completed")
print("writing optimized gcode")
write_gcode(Optimized_Gcode,optimized_Gcode)
print("Process completed")
