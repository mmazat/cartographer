#include <fstream>
#include <iostream>
#include <vector>
#include <iomanip>

#include "cartographer/common/time.h"
#include "trajectory_extractor.h"

#include "../Utils/Geodesy.h"
#include "../Utils/rotation"

#include <Eigen/Geometry>
using namespace cartographer::mapping::proto;


struct SBET_DATA
{
       double time;						// seconds, GPS time
       double latitude;					// radians
       double longitude;				// radians
       double altitude;					// meters
       double x_wander_angle_velocity=0;	// meters/second
       double y_wander_angle_velocity=0;	// meters/second
       double z_wander_angle_velocity=0;	// meters/second
       double roll;						// radians
       double pitch;					// radians
       double platform_heading;			// radians
       double wander_angle=0;				// radians
       double x_body_acceleration=0;		// m/s^2
       double y_body_acceleration=0;		// m/s^2
       double z_body_acceleration=0;		// m/s^2
       double x_body_angular_rate=0;		// radians/second
       double y_body_angular_rate=0;		// radians/second
       double z_body_angular_rate=0;		// radians/second
};

static_assert(sizeof(SBET_DATA)==17*8,"invalid size for SBET_DATA");

namespace cartographer {
namespace io {

  //converts the time stamp of protobuf to the ros time (which is gps seconds)
  //the time stamp of protobuf is 100ns ticks since beging of UTC time
  //see time_conversion.cc
  double ToGpsSeconds(::google::protobuf::int64  time) {


    double hundred_ns_since_unix_epoch =
        (time -
         ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
             10000000ll);

    return  hundred_ns_since_unix_epoch/10000000.0;
  }

  TrajectoryExtractor::TrajectoryExtractor(mapping::proto::SparsePoseGraph& pose_graph_proto)
  {
    int trajectorysize= pose_graph_proto.trajectory_size();
    std::cout<<trajectorysize<<" trajectories found in proto trajectorysize"<<std::endl;

     all_trajectories.insert(all_trajectories.begin(),
         pose_graph_proto.trajectory().begin(),
         pose_graph_proto.trajectory().end());

  }
  void TrajectoryExtractor::ExtractToTextFile(std::string file_name)
  {
    std::ofstream ofs(file_name);
    ofs.precision(16);

    for(const Trajectory& trajectory: all_trajectories)
    {
      for (const Trajectory_Node& node : trajectory.node())
      {
        ofs<<"t_utc= "<<node.timestamp()<<std::endl;
         double gpsSeconds=ToGpsSeconds(node.timestamp());

         ofs<<"t_gps= "<<gpsSeconds<<std::endl;
         auto &pose_rot=node.pose().rotation();
         Eigen::Quaterniond q(pose_rot.w(),pose_rot.x(),pose_rot.y(),pose_rot.z());
         Eigen::Matrix3d R=q.toRotationMatrix();
         ofs<<"R= "<< R<<std::endl;
         auto &pose_t=node.pose().translation();
         Eigen::Vector3d t(pose_t.x(),pose_t.y(),pose_t.z());
         ofs<< "t= "<<t.transpose() <<std::endl;
         double rpy[3];
         ceres::DecomposeRotationToNovatelRPY(ceres::ColumnMajorAdapter3x3(R.data()),rpy);
         ofs<< "rpy= "<<rpy[0]<<" "<<rpy[1]<<" "<<rpy[2]<< std::endl;
      }
    }

  }
  void TrajectoryExtractor::ExtractToSBETFile(std::string file_name, Eigen::Vector3d Origin_ECEF)
  {
    SBET_DATA sbet_data;
    Geodesy geodesy;    
    std::ofstream ofs;
    ofs.open(file_name,std::ios::binary);
    for(const Trajectory& trajectory: all_trajectories)
    {
      for (const Trajectory_Node& node : trajectory.node())
      {
         double gpsSeconds=ToGpsSeconds(node.timestamp());
         sbet_data.time=gpsSeconds;
         auto &pose_rot=node.pose().rotation();
         Eigen::Quaterniond q(pose_rot.w(),pose_rot.x(),pose_rot.y(),pose_rot.z());
         Eigen::Matrix3d R=q.toRotationMatrix();
         double rpy[3];
         ceres::DecomposeRotationToNovatelRPY(ceres::ColumnMajorAdapter3x3(R.data()),rpy);

         //local translation could be also ecef, in such case
         auto &t=node.pose().translation();
         Eigen::Vector3d local_pos(t.x(),t.y(),t.z());
         Eigen::Vector3d ecef_pos=local_pos+Origin_ECEF;
         double lat, lon, h;
         geodesy.EcefToGeodetic(lat,lon,h,ecef_pos[0],ecef_pos[1],ecef_pos[2]);        
         sbet_data.latitude=deg2rad(lat);
         sbet_data.longitude=deg2rad(lon);
         sbet_data.altitude=h;
         sbet_data.roll=deg2rad(rpy[0]);
         sbet_data.pitch=deg2rad(rpy[1]);
         sbet_data.platform_heading=deg2rad(rpy[2]);

         ofs.write((char *) &sbet_data, sizeof(sbet_data));
      }
    }
    ofs.close();
  }

}
}
