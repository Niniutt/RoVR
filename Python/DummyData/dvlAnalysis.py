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

def analyzeDVL(filename):
    SafeConstructor.add_constructor(u'tag:yaml.org,2002:map', construct_yaml_map)
    yaml = YAML(typ='safe')

    ## Read data into string
    print("Formatting the DVL data")
    with open(filename, 'r') as file:
        dvl_data = yaml.load(file)

    measurements = []
    for i in range(0, len(dvl_data), 2):
        dict = {}
        dict["nanosecs"] = dvl_data[i][1]
        dict["vx"] = dvl_data[i+1][1][0]
        dict["vy"] = dvl_data[i+1][1][1]
        dict["vz"] = dvl_data[i+1][1][2]
        dict["timestamp"] = dvl_data[i+1][1][-1][1]

        if i == 0:
            dict["time"] = 0
        else:
            prev = measurements[-1]
            time_diff = (dict["timestamp"]-prev["timestamp"])*1e-6
            dict["time"] = round(prev["time"] + time_diff, 3)

        measurements.append(dict)

    print("Finished formatting the DVL data")

    return measurements