
#include "MyGPS.hpp"
#include <iostream>
#include <vector>

using namespace std;

void print(std::vector <float> const &a) {

   for(size_t i=0; i < a.size(); i++)
   std::cout << a.at(i) << ' ';
   return;
}

int main()
{
    cout << "Team's position response:\n";
    vector<float> pos = GetCurrentPosition();
    vector<float> way = GetNextWaypoint();
    
    cout << "\nPosition: ";
    print(pos);
    cout << "\nWaypoint: ";
    print(way);
}