// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef OW_EPHEMERIS_H_
#define OW_EPHEMERIS_H_

#include <unistd.h>
#include <stdint.h>
#include <math.h>

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <fstream>

namespace ow
{
  typedef double float64_ow;
  typedef float float32_ow;

  const int32_t SUN_BODY_ID = 10;
  const int32_t MERCURY_BODY_ID = 199;
  const int32_t VENUS_BODY_ID = 299;  
  const int32_t EARTH_BODY_ID = 399;
  const int32_t MARS_BODY_ID = 499;
  const int32_t JUPITER_BODY_ID = 599;
  const int32_t EUROPA_BODY_ID = 502;
  const int32_t SATURN_BODY_ID = 699;
  const int32_t ENCELADUS_BODY_ID = 602;
  const int32_t TITAN_BODY_ID = 606;
  const int32_t URANUS_BODY_ID = 799;
  const int32_t NEPTUNE_BODY_ID = 899;
  const int32_t PLUTO_BODY_ID = 999;

  class Ephemeris
  {
  public:
    Ephemeris();
    
    Ephemeris(const std::string& leapSecondKernelPath,
	      const std::string& constantsKernelPath,
	      const std::string& positionKernelPath,
	      bool z_down_surface_frame = true);

    Ephemeris(const std::string& leapSecondKernelPath,
	      const std::string& constantsKernelPath,
	      const std::vector<std::string>& positionKernelPaths,
	      bool z_down_surface_frame = true);

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
				      const float64_ow elev,
				      const std::string& targetBody,
				      const std::string& time,
				      float64_ow transform[16]);

    void VectorToTarget(const std::string& referenceBody,
			const float64_ow lat,
			const float64_ow lon,
			const float64_ow elev,
			const std::string& targetBody,
			const std::string& time,
			float64_ow out_vec[3]);

    // Return fraction of referenceBody visible from behind occulterBody.
    // Result is in the range {0.0, 1.0}
    double FractionVisible(const std::string& referenceBody,
                           const std::string& occulterBody);
  private:
    void Load();

    void UnLoad();
    
    bool m_z_down_surface_frame = true;
    int32_t m_ephemerisHandle = 0;
    std::string m_leapSecondKernelPath;
    std::string m_constantsKernelPath; // planetary shape/size/orientation constants
    std::vector<std::string> m_ephemerisPaths; // ephemerides with planet and spacecraft positions and velocities

    // Storage for vectors to target bodies
    struct Vec3{
      double xyz[3];
      double length() { return sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]); }
      double normalize() { const double l = length(); xyz[0] /= l; xyz[1] /= l; xyz[2] /= l; return l; }
      double& operator[] (int x) { return xyz[x]; }
    };
    std::map<std::string, Vec3> m_bodyVecMap;
  };
}

#endif	// OW_EPHEMERIS_H_
