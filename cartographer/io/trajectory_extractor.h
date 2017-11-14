#ifndef TRAJECTORY_EXTRACTOR_H
#define TRAJECTORY_EXTRACTOR_H
#include <string>
#include <vector>
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/sparse_pose_graph.h"

#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/mapping/proto/sparse_pose_graph.pb.h"
#include "cartographer/common/port.h"

namespace cartographer {
namespace io {

class TrajectoryExtractor
{
public:
  TrajectoryExtractor(mapping::proto::SparsePoseGraph& pose_graph_proto);
   void ExtractToTextFile(std::string file_name);
   /**
    * @brief ExtractToSBETFile
    * @param file_name
    * @param Origin_ECEF shift from local to ecef system
    */
   void ExtractToSBETFile(std::string file_name, Eigen::Vector3d Origin_ECEF);

private:  
  std::vector<mapping::proto::Trajectory> all_trajectories;

};
}
}

#endif // TRAJECTORY_EXTRACTOR_H
