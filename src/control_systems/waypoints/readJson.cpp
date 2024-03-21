#include "rapidjson/document.h" 
#include <fstream> 
#include <iostream> 
  
using namespace std; 
using namespace rapidjson; 

struct JsonStruct {
    float x, y, z;
};

string readJson()
{
    ifstream file("/home/saturn/Desktop/Dev/catamaran/src/control_systems/waypoints/waypoint_locations.json");

    string json((istreambuf_iterator<char>(file)), 
                istreambuf_iterator<char>()); 

    Document doc;

    doc.Parse(json.c_str()); 

    return json.c_str();
}

int main() 
{
    string jsonvar = readJson();

    std::cout << "Reading Json File\n";

    return 0;
}