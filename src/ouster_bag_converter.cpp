#include <iostream>
#include <vector>
#include <string>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "point_type.h"

int main(int argc, char** argv)
{
  if (argc != 4)
  {
    std::cerr << "Usage: " << argv[0] << " input_rosbag topic_name output_rosbag" << std::endl;
    return 1;
  }

  std::string input_rosbag(argv[1]);
  std::string topic_name(argv[2]);
  std::string output_rosbag(argv[3]);

  rosbag::Bag input_bag;
  input_bag.open(input_rosbag, rosbag::bagmode::Read);

  rosbag::View view;
  view.addQuery(input_bag);
  
  std::vector<const rosbag::ConnectionInfo*> connections = view.getConnections();

  // Check topic existence
  bool topic_exist = false;
  for (const rosbag::ConnectionInfo* connection : connections)
  {
    if (connection->topic == topic_name)
    {
      topic_exist = true;
      break;
    }
  }

  if (!topic_exist)
  {
    std::cerr << "Topic not found: " << topic_name << std::endl;
    return 1;
  }

  // Open output rosbag
  rosbag::Bag output_bag;
  output_bag.open(output_rosbag, rosbag::bagmode::Write);

  int total = view.size();
  int cnt = 0;
  int last_progress = 0;

  // Convert
  for (const rosbag::MessageInstance& m : view)
  {
    cnt++;
    int current_progress = cnt * 100 / total;
    if (current_progress > last_progress)
    {
      std::cout << "\rProgress: " << current_progress << "%" << std::flush;
      last_progress = current_progress;
    }

    if (m.getTopic() == topic_name)
    {
      sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
      pcl::PointCloud<PointOUSTER>::Ptr cloud(new pcl::PointCloud<PointOUSTER>);
      pcl::fromROSMsg(*msg, *cloud);

      // Filter out invalid points
      pcl::PointCloud<PointOUSTER>::Ptr filtered_cloud(new pcl::PointCloud<PointOUSTER>);
      for (const auto& point : cloud->points)
      {
        if ((std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) &&
            (point.x != 0.0 || point.y != 0.0 || point.z != 0.0))
        {
          filtered_cloud->points.push_back(point);
        }
      }

      sensor_msgs::PointCloud2 output_msg;
      pcl::toROSMsg(*filtered_cloud, output_msg);
      output_msg.header = msg->header;
      output_bag.write(topic_name, m.getTime(), output_msg);
    }
    else
    {
      output_bag.write(m.getTopic(), m.getTime(), m);
    }
  }

  std::cout << "\nAll messages converted." << std::endl;
  std::cout << "Writing output rosbag..." << std::endl;

  input_bag.close();
  output_bag.close();

  return 0;
}
