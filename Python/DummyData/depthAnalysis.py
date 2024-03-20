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

def analyzeDepth(filename):
    SafeConstructor.add_constructor(u'tag:yaml.org,2002:map', construct_yaml_map)
    yaml = YAML(typ='safe')

    ## Read data into string
    print("Formatting the depth data")
    with open(filename, 'r') as file:
        depth_data = yaml.load(file)

    # Organize into an easier format
    measurements = []
    for i in range(0, len(depth_data), 3):
        dict = {}
        dict["secs"] = depth_data[i][1][0][1][0][1]
        dict["nano_secs"] = depth_data[i][1][0][1][1][1]
        dict["pressure"] = depth_data[i+1][1]
        dict["depth"] = round((dict["pressure"]-101325)/(1025*9.81))

        # Adding relative timestamp
        if i == 0:
            dict["time"] = 0
        else:
            prev = measurements[-1]
            time_diff = (dict["secs"]-prev["secs"]) + (dict["nano_secs"]*1e-9 - prev["nano_secs"]*1e-9)
            dict["time"] = round(prev["time"] + time_diff, 3)

        measurements.append(dict)
    print("Finished formatting depth data\n")

    return measurements

    # for i in range(len(measurements)):
    #     curr = measurements[i]
    #     print("Time: " + str(format(curr["time"], '.3f')) + " seconds -",  str(round((curr["pressure"]-101325)/(1025*9.81))) + " meters")

# measurements = analyzeDepth('DataSet01/pressureData_test.yaml')