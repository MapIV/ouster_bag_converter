#ifndef OUSTER_BAG_CONVERTER_POINT_TYPE_H
#define OUSTER_BAG_CONVERTER_POINT_TYPE_H

#include <pcl/point_types.h>

struct PointOUSTER
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY
  std::uint32_t t;
  std::uint16_t reflectivity;
  std::uint16_t ring;
  std::uint16_t ambient;
  std::uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointOUSTER,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (std::uint32_t, t, t)
                                   (std::uint16_t, reflectivity, reflectivity)
                                   (std::uint16_t, ring, ring)
                                   (std::uint16_t, ambient, ambient)
                                   (std::uint32_t, range, range)
)

#endif
