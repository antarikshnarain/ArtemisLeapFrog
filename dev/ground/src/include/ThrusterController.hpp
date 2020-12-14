/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Cold gas thruster commands for Ground Station
----------------------------------------------------------------- */

#include <map>

using namespace std;

class ThrusterController
{
  private:
  int count;
    map<int,int> thruster_index_map;
    public:
    void FireThruster(int index, float duration);

}