#ifndef OW_EPHEMERIS_H_
#define OW_EPHEMERIS_H_

#include <unistd.h>
#include <stdint.h>

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

namespace ow
{
  typedef double float64_ow;
  typedef float float32_ow;

  const int32_t SUN_BODY_ID = 10;
  const int32_t EARTH_BODY_ID = 399;
  const int32_t EUROPA_BODY_ID = 502;
  const int32_t ENCELADUS_BODY_ID = 602;
  const int32_t TITAN_BODY_ID = 606;
  const int32_t VENUS_BODY_ID = 299;

  class Ephemeris
  {
  public:
    Ephemeris();
    
    Ephemeris(const std::string& leapSecondKernelPath,
	      const std::string& constantsKernelPath,
	      const std::string& positionKernelPath);

    Ephemeris(const std::string& leapSecondKernelPath,
	      const std::string& constantsKernelPath,
	      const std::vector<std::string>& positionKernelPaths);

    ~Ephemeris();
    
    void PlanetoGraphicToCentric(const std::string& planetaryBody,
				 const float64_ow in_lat,
				 const float64_ow in_lon,
				 float64_ow& out_lat,
				 float64_ow& out_lon);

    void BodyToBodyTransform(const std::string& referenceBody,
			     const std::string& targetBody,
			     const std::string& time,
			     float64_ow transform[16]);

    void SurfaceToTargetBodyTransform(const std::string& referenceBody,
				      const float64_ow lat,
				      const float64_ow lon,
				      const std::string& targetBody,
				      const std::string& time,
				      float64_ow transform[16]);

    void VectorToTarget(const std::string& referenceBody,
			const float64_ow lat,
			const float64_ow lon,
			const std::string& targetBody,
			const std::string& time,
			float64_ow out_vec[3]);

  private:
    void Load();

    void UnLoad();
    
    int32_t m_ephemerisHandle = 0;
    std::string m_leapSecondKernelPath;
    std::string m_constantsKernelPath; // planetary shape/size/orientation constants
    std::vector<std::string> m_ephemerisPaths; // ephemerides with planet and spacecraft positions and velocities
  };
}

#endif	// OW_EPHEMERIS_H_
