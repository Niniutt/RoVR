import random
import numpy as np
import matplotlib.pyplot as plt
import csv
import ast

# Depth
# Pressure
# Battery (battery percentage, time before ascent, time left of battery)
# Orientation

def batteryVals(numSamples, adjustVal):
    startVal = 100
    
    batteryArray = np.zeros(numSamples)

    batteryArray[0] = startVal

    for i in range(1, numSamples):
        if batteryArray[i-1] != 0:
            batteryArray[i] = batteryArray[i-1] - adjustVal*random.random()
            # batteryArray[i] = batteryArray[i-1] - 0.01

        if batteryArray[i] < 0:
            batteryArray[i] = 0

    return batteryArray

def depthVals(numSamples, startVal, frequency, amplitude):

    depthArray = np.zeros(numSamples)

    x_axis = np.linspace(0, numSamples, numSamples) 

    for i in range(numSamples):
        depthArray[i] = amplitude*np.sin(2*np.pi*frequency*x_axis[i]) + startVal + np.random.normal(0, 1)

    pAtm = 101325
    density = 1020
    g = 9.81

    pressureArray = (density*g*depthArray + pAtm)/pAtm

    return depthArray, pressureArray

def calcBatteryTime(numSamples, battery, time, depth):
    est_battery_lifetime = [600] # Initializing as an unreasonable value
    est_ascent_time = [600]

    for i in range(1, numSamples):
        estimate = battery[i]/(-(battery[i] - battery[i-1])/(float(time[i]) - float(time[i-1])))

        if estimate < est_ascent_time[0]:
            est_battery_lifetime.append(estimate)
        else:
            est_battery_lifetime.append(est_battery_lifetime[i-1])

    margin = 60 # Margin to estimated battery lifetime (s)
    vmax = 1.06 # maximum velocity upwards (m/s)

    for i in range(1, numSamples):
        estimate = (est_battery_lifetime[i] - margin) - ((float(depth[i]))/vmax)
        
        if estimate < est_ascent_time[0]:
            est_ascent_time.append(estimate)
        else:
            est_ascent_time.append(est_ascent_time[i-1])

    return est_battery_lifetime, est_ascent_time

def readData():
    input_file = open('ProcessedData/DataSet01Resampled/data.csv', 'r')
    reader = csv.DictReader(input_file)
    data = []
    for dictionary in reader:
        data.append(dictionary)
    
    return data

def orientationVals(numSamples, var):
    data = readData()
    
    orientation = []

    for i in range(numSamples):
        orientation.append(ast.literal_eval(data[i]["orientation"]))
        # orientation.append(np.multiply(1.25, ast.literal_eval(data[i]["orientation"])))

        # val = [orien for orien in np.multiply(1/1.25, ast.literal_eval(data[i]["orientation"]))]
        # orientation.append(val)

    return orientation

def writeData(data):
    keys = data[0].keys()

    with open('ProcessedData/FictionalData/data_fictional.csv', 'w', newline='') as output_file:
        dict_writer = csv.DictWriter(output_file, keys)
        dict_writer.writeheader()
        dict_writer.writerows(data)

numSamples = 10000

batteryVal = batteryVals(numSamples, 0.01)

depthVal, pressureVal = depthVals(numSamples, 150, 10, 25)

timeAxis = np.linspace(0, 600, numSamples)

est_battery_lifetime, est_ascent_time = calcBatteryTime(numSamples, batteryVal, timeAxis, depthVal)

orientationVal = orientationVals(numSamples, 0)

data = []

for i in range(numSamples):
    dict = {"time" : timeAxis[i],
            "pressure" : pressureVal[i],
            "depth" : depthVal[i],
            "battery" : batteryVal[i],
            "estimated_battery_time" : est_battery_lifetime[i],
            "estimated_time_before_ascent" : est_ascent_time[i],
            "orientation" : orientationVal[i]
            }

    data.append(dict)

writeData(data)