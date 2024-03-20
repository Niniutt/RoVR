import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
import ast
import csv

def linear_interpolation_upsampler(input_array, upsampling_factor):
    input_array = np.asarray(input_array, dtype=float)
    # Determine the size of the output array based on the upsampling factor
    n = len(input_array)
    upsampled_size = n * upsampling_factor
    
    # Create an output array with the calculated size
    upsampled_array = np.zeros(upsampled_size)
    
    # Assign original data points to their corresponding positions in the output array
    upsampled_array[::upsampling_factor] = input_array
    
    # Perform linear interpolation for the missing data points
    for i in range(upsampling_factor - 1):
        weight = float((i + 1) / upsampling_factor)
        interp_value = (1 - weight) * input_array + weight * input_array
        upsampled_array[i+1::upsampling_factor] = interp_value
    
    return upsampled_array

def resampleDepth(depths, intFactorDepths, newTimeAxis):
    allPressures = []
    allDepths = []

    for depth in depths:
        allPressures.append(float(depth["pressure"]))
        allDepths.append(float(depth["depth"]))

    allPressuresResampled = linear_interpolation_upsampler(allPressures, intFactorDepths)
    allDepthsResampled = linear_interpolation_upsampler(allDepths, intFactorDepths)

    depthsResampled = []

    for i in range(len(newTimeAxis)):
        depthsResampled.append({"time" : newTimeAxis[i], 
                                "pressure" : allPressuresResampled[i],
                                "depth" : allDepthsResampled[i]})

    return depthsResampled

def resampleImu(imus, intFactorImus, newTimeAxis):
    orientation = []
    angular_v = []
    linear_a = []
    for i in range(len(imus)):
        orientation.append([ast.literal_eval(imus[i]["orientation"])[0][1],
                            ast.literal_eval(imus[i]["orientation"])[1][1],
                            ast.literal_eval(imus[i]["orientation"])[2][1],
                            ast.literal_eval(imus[i]["orientation"])[3][1]])
        angular_v.append([ast.literal_eval(imus[0]["angular_v"])[0][1],
                          ast.literal_eval(imus[0]["angular_v"])[1][1],
                          ast.literal_eval(imus[0]["angular_v"])[2][1]])
        linear_a.append([ast.literal_eval(imus[0]["linear_a"])[0][1],
                         ast.literal_eval(imus[0]["linear_a"])[1][1],
                         ast.literal_eval(imus[0]["linear_a"])[2][1]])

    # np.transpose(orientation)[0][:] # All in x-direction    

    orientation_x_resampled = linear_interpolation_upsampler(np.transpose(orientation)[0][:], intFactorImus)
    orientation_y_resampled = linear_interpolation_upsampler(np.transpose(orientation)[1][:], intFactorImus)
    orientation_z_resampled = linear_interpolation_upsampler(np.transpose(orientation)[2][:], intFactorImus)
    orientation_w_resampled = linear_interpolation_upsampler(np.transpose(orientation)[3][:], intFactorImus)

    angular_v_x_resampled = linear_interpolation_upsampler(np.transpose(angular_v)[0][:], intFactorImus)
    angular_v_y_resampled = linear_interpolation_upsampler(np.transpose(angular_v)[1][:], intFactorImus)
    angular_v_z_resampled = linear_interpolation_upsampler(np.transpose(angular_v)[2][:], intFactorImus)

    linear_a_x_resampled = linear_interpolation_upsampler(np.transpose(linear_a)[0][:], intFactorImus)
    linear_a_y_resampled = linear_interpolation_upsampler(np.transpose(linear_a)[1][:], intFactorImus)
    linear_a_z_resampled = linear_interpolation_upsampler(np.transpose(linear_a)[2][:], intFactorImus)

    imusResampled = []
    for i in range(len(newTimeAxis)):
        dict = {'time' : newTimeAxis[i],
                'orientation' : [orientation_x_resampled[i], orientation_y_resampled[i], 
                                 orientation_z_resampled[i], orientation_w_resampled[i]],
                'angular_v' : [angular_v_x_resampled[i], angular_v_y_resampled[i], angular_v_z_resampled[i]],
                'linear_a' : [linear_a_x_resampled[i], linear_a_y_resampled[i], linear_a_z_resampled[i]]}
        imusResampled.append(dict)

    return imusResampled

def resampleDvl(dvls, intFactorDvl, newTimeAxis):
    vx = []
    vy = []
    vz = []
    for i in range(len(dvls)):
        vx.append(ast.literal_eval(dvls[0]["vx"])[1])
        vy.append(ast.literal_eval(dvls[0]["vy"])[1])
        vz.append(ast.literal_eval(dvls[0]["vz"])[1])
        
    vx_resampled = linear_interpolation_upsampler(vx, intFactorDvl)
    vy_resampled = linear_interpolation_upsampler(vy, intFactorDvl)
    vz_resampled = linear_interpolation_upsampler(vz, intFactorDvl)

    dvlsResampled = []
    for i in range(len(newTimeAxis)):
        dict = {'time' : newTimeAxis[i],
                'vx' : vx_resampled[i],
                'vy' : vy_resampled[i],
                'vz' : vz_resampled[i]}
        dvlsResampled.append(dict)

    return dvlsResampled

def writeResampled(depths, imus, dvls, dataset):
    depthKeys = depths[0].keys()
    imuKeys = imus[0].keys()
    dvlKeys = dvls[0].keys()

    with open('ProcessedData/' + dataset + 'Resampled' + '/depth.csv', 'w', newline='') as output_file:
        dict_writer = csv.DictWriter(output_file, depthKeys)
        dict_writer.writeheader()
        dict_writer.writerows(depths)

    with open('ProcessedData/' + dataset + 'Resampled' + '/imu.csv', 'w', newline='') as output_file:
        dict_writer = csv.DictWriter(output_file, imuKeys)
        dict_writer.writeheader()
        dict_writer.writerows(imus)

    with open('ProcessedData/' + dataset + 'Resampled' + '/dvl.csv', 'w', newline='') as output_file:
        dict_writer = csv.DictWriter(output_file, dvlKeys)
        dict_writer.writeheader()
        dict_writer.writerows(dvls)

def upsample(depths, imus, dvls, dataset):
    endTime = max([depths[-1]["time"], imus[-1]["time"], dvls[-1]["time"]])

    # Valid for at least dataset 01
    depths = depths[:40000]
    imus = imus[:120000]
    dvls = dvls[:3000]
    length = len(max([depths, imus, dvls], key=len))

    intFactorDepths = int(length/len(depths))
    intFactorImus = int(length/len(imus))
    intFactorDvls = int(length/len(dvls))

    newTimeAxis = np.linspace(0, float(endTime), length, endpoint=True)

    # Interpolate the depths
    depthsResampled = resampleDepth(depths, intFactorDepths, newTimeAxis)
    imusResampled = resampleImu(imus, intFactorImus, newTimeAxis)
    dvlsResampled = resampleDvl(dvls, intFactorDvls, newTimeAxis)

    writeResampled(depthsResampled, imusResampled, dvlsResampled, dataset)
    