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

  class NedRotation
  {
  public:
    static double getRoll(const Eigen::Matrix3d &r) { return atan(r(2, 1) / r(2, 2)); }

      static double getPitch(const Eigen::Matrix3d &r) {
        return atan(-r(2, 0) / sqrt(r(0, 0)*r(0,0) + r(1, 0)*r(1,0)));
      }

      static double getHeading(const Eigen::Matrix3d &r) {
        double h = atan2(r(1, 0), r(0, 0));
        if (h < 0) {
          h += 3.1415926535;
        }
        return h;
      }
  };
  void RotationToSBETRPH(const cartographer::transform::proto::Quaterniond &pose_rotation, double rph_rad[3])
  {
    //convert to Eigen quaternion
    Eigen::Quaterniond q(pose_rotation.w(),pose_rotation.x(),pose_rotation.y(),pose_rotation.z());
    q.normalize();
    Eigen::Matrix3d Rb2l=q.toRotationMatrix();
#if 0
    Eigen::Vector3d ea = Rb2l.eulerAngles(0,1,2);
    for(int i=0;i<3;++i)
    rpy_rad[i]=ea[i];
    return;
#endif

    Eigen::Vector3d rpy1,rpy2, novRPY;
    Eigen::Matrix3d Rb2v;
    double vehicle2body[3]= {0.0,0.0,180.0};
    ceres::CreateRotationMzMyMx(vehicle2body,ceres::ColumnMajorAdapter3x3(Rb2v.data()));

    Eigen::Matrix3d Rv2l=Rb2l*Rb2v.transpose();
    ceres::DecomposeRotationToNovatelRPY(ceres::ColumnMajorAdapter3x3(Rv2l.data()),rpy1.data());

    //double enu2ned[3]= {0.0,180.0,90.0};
    //Eigen::Matrix3d Renu2ned;
    //ceres::CreateRotationMzMyMx(enu2ned,ceres::ColumnMajorAdapter3x3(Renu2ned.data()));
    //Eigen::Matrix3d Rv2l_Ned=Renu2ned*Rv2l;

    //the applanix Ned coordinte system Rb2l=Mz(-h)*My(-p)*Mx(-r) therefor when decomposed to RzRyRx the output will be r p h  and now sign change is required
    //ceres::DecomposeRotationToRzRyRx(ceres::ColumnMajorAdapter3x3(Rv2l.data()),rpy1.data(),rpy2.data());


    //std::cout<<"novatel rpy "<<novRPY.transpose()<< " app1 "<<rpy1.transpose()<< std::endl;



    for(int i=0;i<3;++i)
    rph_rad[i]=deg2rad(rpy1[i]);

    //negate the yaw to get heading
    rph_rad[2]=-rph_rad[2];

    if(rph_rad[2]<0)
      rph_rad[2]+=3.1415926535*2.0;

  }

  TrajectoryExtractor::TrajectoryExtractor(mapping::proto::SparsePoseGraph& pose_graph_proto)
  {
    int trajectorysize= pose_graph_proto.trajectory_size();
    std::cout<<trajectorysize<<" trajectories found in proto trajectorysize"<<std::endl;

     all_trajectories.insert(all_trajectories.begin(),
         pose_graph_proto.trajectory().begin(),
         pose_graph_proto.trajectory().end());

  }

  void TrajectoryExtractor::ExtractToSBETFile(std::string file_name, Eigen::Vector3d Origin_ECEF)
  {
    SBET_DATA sbet_data;
    Geodesy geodesy;    
    std::ofstream sbet_ofs, text_ofs;
    sbet_ofs.open(file_name,std::ios::binary);
    text_ofs.open(file_name+".csv");
    text_ofs.precision(16);
    double rph_rad[3];
    for(const Trajectory& trajectory: all_trajectories)
    {
      for (const Trajectory_Node& node : trajectory.node())
      {
         double gpsSeconds=ToGpsSeconds(node.timestamp());
         sbet_data.time=gpsSeconds;         
         RotationToSBETRPH(node.pose().rotation(),rph_rad);
         //local translation could be also ecef, in such case
         auto &t=node.pose().translation();
         Eigen::Vector3d local_pos(t.x(),t.y(),t.z());
         Eigen::Vector3d ecef_pos=local_pos+Origin_ECEF;
         double lat, lon, h;
         geodesy.EcefToGeodetic(lat,lon,h,ecef_pos[0],ecef_pos[1],ecef_pos[2]);        
         sbet_data.latitude=deg2rad(lat);
         sbet_data.longitude=deg2rad(lon);
         sbet_data.altitude=h;
         sbet_data.roll=rph_rad[0];
         sbet_data.pitch=rph_rad[1];
         sbet_data.platform_heading=rph_rad[2];

         sbet_ofs.write((char *) &sbet_data, sizeof(sbet_data));
         text_ofs<<gpsSeconds<<","<<lat<<","<<lon<<","<<sbet_data.altitude<<","<<
                   sbet_data.roll<<","<<sbet_data.pitch<<","<<sbet_data.platform_heading<<std::endl;


      }
    }
    sbet_ofs.close();
    text_ofs.close();
  }

}
}
