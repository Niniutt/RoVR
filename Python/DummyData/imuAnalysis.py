import sys
from ruamel.yaml import YAML
from ruamel.yaml.constructor import SafeConstructor


## Setup YAML parser
def construct_yaml_map(self, node):
    # test if there are duplicate node keys
    data = []
    yield data
    for key_node, value_node in node.value:
        key = self.construct_object(key_node, deep=True)
        val = self.construct_object(value_node, deep=True)
        data.append((key, val))

def analyzeIMU(filename):
    SafeConstructor.add_constructor(u'tag:yaml.org,2002:map', construct_yaml_map)
    yaml = YAML(typ='safe')

    ## Read data into string
    print("Formatting the IMU data")
    with open(filename, 'r') as file:
        imu_data = yaml.load(file)

    measurements = []
    for i in range(0, len(imu_data), 7):
        dict = {}
        dict["secs"] = imu_data[i][1][0][1][0][1]
        dict["nano_secs"] = imu_data[i][1][0][1][1][1]
        dict["orientation"] = [imu_data[i+1][1][0], imu_data[i+1][1][1], imu_data[i+1][1][2], imu_data[i+1][1][3]]
        dict["angular_v"] = [imu_data[i+3][1][0], imu_data[i+3][1][1], imu_data[i+3][1][2]]
        dict["linear_a"] = [imu_data[i+5][1][0], imu_data[i+5][1][1], imu_data[i+5][1][2]]

        if i == 0:
            dict["time"] = 0
        else:
            prev = measurements[-1]
            time_diff = (dict["secs"]-prev["secs"]) + (dict["nano_secs"]*1e-9 - prev["nano_secs"]*1e-9)
            dict["time"] = round(prev["time"] + time_diff, 3)

        measurements.append(dict)

    print("Finished formatting the IMU data")

    return measurements
