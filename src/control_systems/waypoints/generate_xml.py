import json

file = open('/home/saturn/Desktop/Dev/catamaran/src/control_systems/waypoints/waypoint_locations.json')
loadedWaypoints = json.load(file)


xmlFile = open("/home/saturn/Desktop/Dev/catamaran/src/control_systems/waypoints/model/model.sdf","w")
tab = '  '

xmlFile.write("<?xml version='1.0'?>\n")
xmlFile.write("<sdf version='1.7'>\n")
xmlFile.write(tab+'<model name="Waypoints">\n')
xmlFile.write(tab+tab+'<static>1</static>\n')

for i in list(loadedWaypoints.keys()):

    fileContents = [tab+tab+'<link name="'+i+'">',
        tab+tab+'<visual name="b">',
            tab+tab+tab+'<cast_shadows>0</cast_shadows>',
            tab+tab+tab+'<pose>'+str(loadedWaypoints[i]['x'])+'  '+ str(loadedWaypoints[i]['y'])+' '+str(loadedWaypoints[i]['z'])+' 0 0 0</pose>',
            tab+tab+tab+'<geometry>',
            tab+tab+tab+'<box>',
                tab+tab+tab+tab+'<size>0.1 0.1 10</size>',
            tab+tab+tab+'</box>',
            tab+tab+tab+'</geometry>',
            tab+tab+tab+'<material>',
            tab+tab+tab+'<ambient>0.5 0 0.5 1</ambient>',
            tab+tab+tab+'<diffuse>0.5 0 0.5 1</diffuse>',
            tab+tab+tab+'<emissive>0.5 0 0.5 1</emissive>',
            tab+tab+tab+'<specular>0.6 0.6 0.6 1</specular>',
            tab+tab+tab+'</material>',
        tab+tab+'</visual>',
        tab+tab+'</link>']

    for j in range(len(fileContents)):
        xmlFile.write(fileContents[j])
        xmlFile.write("\n")

xmlFile.write(tab+"</model>\n")
xmlFile.write("</sdf>")
xmlFile.close()
