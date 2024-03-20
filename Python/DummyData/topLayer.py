import csv
import numpy as np
import ast
from statistics import mean
from depthAnalysis import analyzeDepth
from imuAnalysis import analyzeIMU
from dvlAnalysis import analyzeDVL
from resample import upsample

def writeDepth(dataset):
    depths = analyzeDepth(dataset + '/pressureData.yaml')

    keys = depths[0].keys()

    with open('ProcessedData/' + dataset + '/depth.csv', 'w', newline='') as output_file:
        dict_writer = csv.DictWriter(output_file, keys)
        dict_writer.writeheader()
        dict_writer.writerows(depths)

def readDepth(dataset):
    input_file = open('ProcessedData/' + dataset + '/depth.csv', 'r')
    reader = csv.DictReader(input_file)
    depths = []
    for dictionary in reader:
        depths.append(dictionary)

    return depths

def writeImu(dataset):
    imus = analyzeIMU(dataset + '/xsensImuData.yaml')

    keys = imus[0].keys()

    with open('ProcessedData/' + dataset + '/imu.csv', 'w', newline='') as output_file:
        dict_writer = csv.DictWriter(output_file, keys)
        dict_writer.writeheader()
        dict_writer.writerows(imus)

def readImu(dataset):
    input_file = open('ProcessedData/' + dataset + '/imu.csv', 'r')
    reader = csv.DictReader(input_file)
    imus = []
    for dictionary in reader:
        imus.append(dictionary)

    return imus

def writeDvl(dataset):
    dvls = analyzeDVL(dataset + '/waterlinkedA50Data.yaml')

    keys = dvls[0].keys()

    with open('ProcessedData/' + dataset + '/dvl.csv', 'w', newline='') as output_file:
        dict_writer = csv.DictWriter(output_file, keys)
        dict_writer.writeheader()
        dict_writer.writerows(dvls)

def readDvl(dataset):
    input_file = open('ProcessedData/' + dataset + '/dvl.csv', 'r')
    reader = csv.DictReader(input_file)
    dvls = []
    for dictionary in reader:
        dvls.append(dictionary)
    
    return dvls

def batteryCal(data):
    steps = len(data)
    # battery = np.linspace(100, 0, steps)
    battery = np.linspace(100, 0, steps)

    est_battery_lifetime = [9999] # Initializing as an unreasonable value
    est_ascent_time = [9999]

    for i in range(1, steps):
        est_battery_lifetime.append(battery[i]/(-(battery[i] - battery[i-1])/(float(data[i]["time"]) - float(data[i-1]["time"]))))

    margin = 60 # Margin to estimated battery lifetime (s)
    vmax = 1.06 # maximum velocity upwards (m/s)

    for i in range(1, steps):
        est_ascent_time.append((est_battery_lifetime[i] - margin) - ((float(data[i]["depth"]))/vmax))

    # Everything into one
    battery_all = []

    for i in range(steps):
        battery_all.append([battery[i], est_battery_lifetime[i], est_ascent_time[i]])

    return battery_all

def readResampledData(dataset):
    input_file_depth = open('ProcessedData/' + dataset + 'Resampled/depth.csv', 'r')
    input_file_dvl = open('ProcessedData/' + dataset + 'Resampled/dvl.csv', 'r')
    input_file_imu = open('ProcessedData/' + dataset + 'Resampled/imu.csv', 'r')

    data_depth = []
    data_dvl = []
    data_imu = []

    # Read
    readerDepth = csv.DictReader(input_file_depth)
    for dictionary in readerDepth:
        data_depth.append(dictionary)

    battery = batteryCal(data_depth)

    readerDvl = csv.DictReader(input_file_dvl)
    for dictionary in readerDvl:
        data_dvl.append(dictionary)

    readerImu = csv.DictReader(input_file_imu)
    for dictionary in readerImu:
        data_imu.append(dictionary)

    # Read everything into one array
    data = []
    for i in range(len(data_depth)):
        dict = {"time" : data_depth[i]["time"],
                "pressure" : data_depth[i]["pressure"],
                "depth" : data_depth[i]["depth"],
                "battery" : battery[i][0],
                "estimated_battery_time" : battery[i][1],
                "estimated_time_before_ascent" : battery[i][2],
                "orientation" : ast.literal_eval(data_imu[i]["orientation"]),
                "angular_v" : ast.literal_eval(data_imu[i]["angular_v"]),
                "linear_a" : ast.literal_eval(data_imu[i]["linear_a"]),
                "doppler_v" : [float(data_dvl[i]["vx"]), float(data_dvl[i]["vy"]), float(data_dvl[i]["vz"])]
                }

        data.append(dict)

    return data

def writeData(dataset, data):
    keys = data[0].keys()

    with open('ProcessedData/' + dataset + 'Resampled/data.csv', 'w', newline='') as output_file:
        dict_writer = csv.DictWriter(output_file, keys)
        dict_writer.writeheader()
        dict_writer.writerows(data)

def readData(dataset):
    input_file = open('ProcessedData/' + dataset + 'Resampled/data.csv', 'r')
    reader = csv.DictReader(input_file)
    data = []
    for dictionary in reader:
        data.append(dictionary)
    
    return data

def main():
    dataset = 'DataSet01'

    # Things will need to be uncommented for new data sets if needed

    # # Parsing data and writing to file
    # writeDepth(dataset)
    # writeImu(dataset)
    # writeDvl(dataset)

    # # Reading parsed data from csv-file
    # depths = readDepth(dataset)
    # imus = readImu(dataset)
    # dvls = readDvl(dataset)

    # # Resampling the data to the same rate
    # upsample(depths, imus, dvls, dataset)

    # # Reading the resampled data from the csv-file
    # data = readResampledData(dataset)

    # # Writing the resampled data in one entire set
    # writeData(dataset, data)

    # # Reading the entire set of data
    # data = readData(dataset)

main()