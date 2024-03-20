import socket
import csv
import ast
import time

dataset = 'DataSet01'
input_file = open('ProcessedData/' + dataset + 'Resampled/data.csv', 'r')

data_dict = []

reader = csv.DictReader(input_file)
for dictionary in reader:
        data_dict.append(dictionary)

host, port = "127.0.0.1", 25001

# SOCK_STREAM means TCP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    # Connect to the server and send the data
    sock.connect((host, port))

    for i in range(len(data_dict)):

        time_val = float(data_dict[i]["time"])
        pressure = float(data_dict[i]["pressure"])
        depth = float(data_dict[i]["depth"])
        battery = float(data_dict[i]["battery"])
        est_battery_life = float(data_dict[i]["estimated_battery_time"])
        est_ascent = float(data_dict[i]["estimated_time_before_ascent"])
        orientation = ast.literal_eval(data_dict[i]["orientation"])
        angular_v = ast.literal_eval(data_dict[i]["angular_v"])
        linear_a = ast.literal_eval(data_dict[i]["linear_a"])
        doppler_v = ast.literal_eval(data_dict[i]["doppler_v"])

        data = str("{"
                   "[time : " + str(time_val) +
                   "],[pressure : " + str(pressure) +
                   "],[depth : [" + str(depth) +
                   "],[battery : [" + str(battery) +
                   "],[battery lifetime : [" + str(est_battery_life) +
                   "],[time before ascent : [" + str(est_ascent) +
                   "],[orientation : " + str(orientation) +
                   "],[angular velocity : " + str(angular_v) +
                   "],[linear acceleration : " + str(linear_a) +
                   "],[doppler velocity : " + str(doppler_v) +
                   "]}")

        sock.sendall(data.encode("utf-8"))
        response = sock.recv(1024).decode("utf-8")
        print (response)

        if i < len(data_dict)-1:
            time.sleep(float(data_dict[i+1]["time"])-float(data_dict[i]["time"]))

finally:
    sock.close()