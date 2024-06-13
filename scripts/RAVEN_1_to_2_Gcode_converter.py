#! /usr/bin/env python3

import numpy as np
from pathlib import Path
import math

# declare constants
DECIMAL_PRECISION = 6  # amount of digits when floats are written to digits


nozzle_diameter = 0.4
layer_height = 0.399
filament_diameter = 1.75
extrusion_multiplier = 1.0

reclaculate_E = True

def load_gcode(file_name):
    file = open(file_name, "r")
    lines = []
    for line in file:
        if line.startswith("#"):
            pass
        elif line == "\n":  # avoid blank lines
            pass
        elif line:  # avoid blank lines
            lines.append(np.array(line.strip("\n").split(" ")).astype(float))
    return lines


# reading the path from the file
raven_pkg_root = Path(__file__).resolve().parents[1]
#raven1_gcode_path = input("\ninput the absolute path to your file:\n")
raven1_gcode_path = '../xarm_ws/src/raven/scripts/Converted_file.txt'
#filename = raven1_gcode_path.split("/")[-1]
filename = "RAVEN_2_gcode.txt"
temperature = float(input("\nat what temperature should the nozzle be set?\n"))
raven2_gcode_filepath = raven_pkg_root/"scripts"/filename


with raven2_gcode_filepath.open("w") as file:
    file.write("; Set initial conditions\n")
    file.write("G28; home\n")
    file.write(
        f"M109 S{temperature}; set and wait for hotend temperature to {temperature}°C\n"
    )  # todo: ask user
    file.write("G90; Use absolute positioning\n")
    Gcode = load_gcode(raven1_gcode_path)
    current_pose = [Gcode[0][2],Gcode[0][3],Gcode[0][4]]
    Origin = [Gcode[0][2],Gcode[0][3],Gcode[0][4]]
    E_total = 0.0
    for line in load_gcode(raven1_gcode_path):
        is_print_cmd = bool(line[0])
        formatted_line = (
            f"G{int(is_print_cmd)} "
            f"F{line[1]*1000:.{DECIMAL_PRECISION}f} "
            f"X{(line[2]-Origin[0])*1000:.{DECIMAL_PRECISION}f} "
            f"Y{(line[3]-Origin[1])*1000:.{DECIMAL_PRECISION}f} "
            f"Z{(line[4]-Origin[2])*1000:.{DECIMAL_PRECISION}f} "
            f"A{line[6]-math.pi:.{DECIMAL_PRECISION}f} "
            f"B{line[7]:.{DECIMAL_PRECISION}f} "
            f"C{line[8]:.{DECIMAL_PRECISION}f} "
        )

        if is_print_cmd :
            if reclaculate_E:
                #i situations were E has to be recalculated 
                Length = (math.sqrt(math.pow(line[2] - current_pose[0], 2) + math.pow(line[3] - current_pose[1], 2) + math.pow(line[4] - current_pose[2], 2)) * 1000)
                Extrusion_for_GcodeArray =  extrusion_multiplier *(Length)*nozzle_diameter*layer_height*4/(math.pi*(pow(filament_diameter,2)))
                E_total = E_total + Extrusion_for_GcodeArray
                formatted_line += f"E{E_total:.{DECIMAL_PRECISION}f}"
            else:
                formatted_line += f"E{line[5]:.{DECIMAL_PRECISION}f}"
                
        current_pose = [line[2],line[3],line[4]]
        formatted_line += "\n"

        file.write(formatted_line)

print(f"succesfully transformed file! you can find it in:\n-> {raven2_gcode_filepath}")
