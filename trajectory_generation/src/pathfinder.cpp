#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>
#include "a_star.h"

using namespace std;
using namespace Eigen;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pathfinder");
    ros::NodeHandle nh;
    double _resolution = 0.2;
    double _x_size = 50.0;
    double _y_size = 50.0 ;
    double _z_size = 5.0 ;
    double _x_local_size = 50.0;
    double _y_local_size = 50.0;
    double _z_local_size = 5.0;

    Vector3d _map_origin;
    _map_origin << -_x_size/2.0, -_y_size/2.0, 0.0;

    double _inv_resolution = 1.0 / _resolution;
    int _max_x_id = (int)(_x_size * _inv_resolution);
    int _max_y_id = (int)(_y_size * _inv_resolution);
    int _max_z_id = (int)(_z_size * _inv_resolution);
    int _max_local_x_id = (int)(_x_local_size * _inv_resolution);
    int _max_local_y_id = (int)(_y_local_size * _inv_resolution);
    int _max_local_z_id = (int)(_z_local_size * _inv_resolution);

    Vector3i GLSIZE(_max_x_id, _max_y_id, _max_z_id);
    Vector3i LOSIZE(_max_local_x_id, _max_local_y_id, _max_local_z_id);
    
    gridPathFinder Astar( GLSIZE, LOSIZE); 
    
    Astar.initGridNodeMap(_resolution, _map_origin);
    Eigen::Vector3d _start_pt = {0,0,0};
    Eigen::Vector3d _end_pt = {1,-2,1};
    Astar.AstarSearch(_start_pt, _end_pt);
    vector<Eigen::Vector3d> gridPath = Astar.getPath();
    int xxxx = gridPath.size();
    printf("CCCCCCCCCCCCCCCgrid%d\n",xxxx);
    for (size_t i = 0; i < gridPath.size(); i++)
    {
    printf("%f\t%f\t%f\n",gridPath[i](0),gridPath[i](1),gridPath[i](2));
    }
    return 0;
}