#include "rapidjson/document.h" 
#include <fstream> 
#include <iostream> 
  
using namespace std; 
using namespace rapidjson; 

int main()
{
    ifstream file("/home/saturn/Desktop/Dev/catamaran/src/control_systems/waypoints/waypoint_locations.json");

    string json((istreambuf_iterator<char>(file)), 
                istreambuf_iterator<char>()); 

    Document doc;

    doc.Parse(json.c_str()); 

    return 0;
}