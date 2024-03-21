import json


def loadJson():

    with open('src/control_systems/waypoints/waypoint_locations.json') as json_file:
        data = json.load(json_file)
    
    return data

