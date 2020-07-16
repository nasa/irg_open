// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "ephemeris.h"

#include <cmath>

// SPICE includes
#include <cspice/SpiceUsr.h>
#include <cspice/SpiceZfc.h>

#include <assert.h>

using namespace ow;
using namespace std;

Ephemeris::Ephemeris()
{
  m_ephemerisHandle = 0;
  m_leapSecondKernelPath = "./latest_leapseconds.tls";
  m_constantsKernelPath =  "./pck00010.tpc";
  m_ephemerisPaths.push_back("./de430.bsp");
  m_z_down_surface_frame = true;
  
  // Set the SPICELIB error response action to "RETURN":
  erract_c("SET", 0, (char *) "RETURN");

  Load();			// Load the SPICE kernels
}

Ephemeris::Ephemeris(const string& leapSecondKernelPath,
		     const string& constantsKernelPath,
		     const string& ephemerisPath,
		     bool z_down_surface_frame)
{ 
  m_ephemerisHandle = 0;
  m_leapSecondKernelPath = leapSecondKernelPath;
  m_constantsKernelPath = constantsKernelPath;
  m_ephemerisPaths.push_back(ephemerisPath);
  m_z_down_surface_frame = z_down_surface_frame;

  // Set the SPICELIB error response action to "RETURN":
  erract_c("SET", 0, (char *) "RETURN");

  Load();			// Load the SPICE kernels
}

Ephemeris::Ephemeris(const string& leapSecondKernelPath,
		     const string& constantsKernelPath,
		     const vector<string>& ephemerisPaths,
		     bool z_down_surface_frame)
{ 
  m_ephemerisHandle = 0;
  m_leapSecondKernelPath = leapSecondKernelPath;
  m_constantsKernelPath = constantsKernelPath;
  for (int i = 0; i < ephemerisPaths.size(); ++i)
    m_ephemerisPaths.push_back(ephemerisPaths[i]);
  m_z_down_surface_frame = z_down_surface_frame;

  // Set the SPICELIB error response action to "RETURN":
  erract_c("SET", 0, (char *) "RETURN");

  Load();			// Load the SPICE kernels
}

Ephemeris::~Ephemeris()
{ 
  UnLoad();
}

void Ephemeris::Load()
{
    clpool_c();
    ldpool_c(m_leapSecondKernelPath.c_str());
    ldpool_c(m_constantsKernelPath.c_str());
    // Note: we never use the ephemeris handle, so we could just use the
    // high level function furnsh_c() instead of spklef_c().
    for (int i = 0; i < m_ephemerisPaths.size(); ++i)
      spklef_c(m_ephemerisPaths[i].c_str(), &m_ephemerisHandle);
}

void Ephemeris::UnLoad()
{
    clpool_c();
    spkuef_c(m_ephemerisHandle);
}

void Ephemeris::BodyToBodyTransform(const string& referenceBody,
				    const string& targetBody,
				    const string& time,
				    float64_ow transform[16])
{
  SpiceDouble state[6];
  SpiceDouble target_in_ref_frame[3];
  SpiceDouble translation[3];
  SpiceDouble rotation[3][3];
  SpiceDouble lightTime;
  SpiceDouble ephemTime;
  SpiceInt dim;
  logical found;		// SpiceInt, logical & integer are SPICE types
  integer ref_body_id;

  // Convert the time string to ephemeris time
  str2et_c(time.c_str(), &ephemTime);

  // Compute target state in reference body frame with a single call to SPKEZR
  string frameName = "IAU_" + referenceBody;
  spkezr_c(targetBody.c_str(), ephemTime, frameName.c_str(), "LT+S",
	   referenceBody.c_str(),
	   state, &lightTime);
	
  // Extract 3-component target position from the 6-component state.
  vpack_c(state[0], state[1], state[2], target_in_ref_frame);

  // Calculate translation from target to surface in body reference
  // frame.
  vpack_c(-target_in_ref_frame[0], -target_in_ref_frame[1],
	  -target_in_ref_frame[2], translation);

  // Get rotation from reference body to target body
  string targetFrameName = "IAU_" + targetBody;
  pxform_c(frameName.c_str(), targetFrameName.c_str(), ephemTime,
	   rotation);

  // The following assumes column vector notation, and row-major
  // storage. It transforms points given in surface frame to target
  // body frame.
  transform[0] = rotation[0][0];
  transform[1] = rotation[0][1];
  transform[2] = rotation[0][2];
  transform[4] = rotation[1][0];
  transform[5] = rotation[1][1];
  transform[6] = rotation[1][2];
  transform[8] = rotation[2][0];
  transform[9] = rotation[2][1];
  transform[10] = rotation[2][2];
  transform[3] = translation[0];
  transform[7] = translation[1];
  transform[11] = translation[2];
  transform[12] = 0.0;
  transform[13] = 0.0;
  transform[14] = 0.0;
  transform[15] = 1.0;
}

void Ephemeris::SurfaceToTargetBodyTransform(const string& referenceBody,
					     const float64_ow lat,
					     const float64_ow lon,
					     const float64_ow elev,
					     const string& targetBody,
					     const string& time,
					     float64_ow transform[16])
{
  const SpiceInt X_axis_ID = 1, Y_axis_ID = 2, Z_axis_ID = 3;
  const SpiceDouble z_axis[3] = {0.0, 0.0, 1.0};
  SpiceDouble ref_radii[3];
  SpiceInt max_radii_dim = sizeof(ref_radii)/sizeof(ref_radii[0]);
  SpiceDouble state[6];
  SpiceDouble ref_surf_point[3];
  SpiceDouble ref_surf_normal[3];
  SpiceDouble ref_to_surf_rotation[3][3] = {{0.0, 0.0, 0.0},
					    {0.0,0.0,0.0},
					    {0.0,0.0,0.0}};
  SpiceDouble surf_to_target_rotation[3][3] = {{0.0, 0.0, 0.0},
					       {0.0,0.0,0.0},
					       {0.0,0.0,0.0}};
  SpiceDouble target_in_ref_frame[3];
  SpiceDouble translation[3] = {0.0, 0.0, 0.0};
  SpiceDouble rotation[3][3] = {{0.0, 0.0, 0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
  SpiceDouble lightTime;
  SpiceDouble ephemTime;
  SpiceInt dim;
  logical found;		// SpiceInt, logical & integer are SPICE types
  integer ref_body_id;

  // Convert the time string to ephemeris time
  str2et_c(time.c_str(), &ephemTime);

  // Convert reference body ID string to integer code
  bodn2c_c(referenceBody.c_str(), &ref_body_id, &found);

  // Get radius of reference body - Deprecated
  // bodvar_c(ref_body_id, "RADII", &dim, ref_radii);

  // Get radius of reference body
  bodvrd_c(referenceBody.c_str(), "RADII", max_radii_dim, &dim, ref_radii);

  // Compute cartesian coordinates of the specified lat/lon on the
  // reference body surface (assumes planetocentric lat/lon)
  srfrec_c(ref_body_id, lon * rpd_c(), lat * rpd_c(), ref_surf_point);

  // Compute reference body surface normal
  surfnm_c(ref_radii[0], ref_radii[1], ref_radii[2], ref_surf_point,
	   ref_surf_normal);

  // Compute surface transform matrix
  if (m_z_down_surface_frame)
  {
    // For a X-North, Y-East, and Z-down (NED) local-level frame we
    // negate the normal to the surface, otherwise the twovec_c() call
    // would produce a Z-up frame.
    ref_surf_normal[0] = -ref_surf_normal[0];
    ref_surf_normal[1] = -ref_surf_normal[1];
    ref_surf_normal[2] = -ref_surf_normal[2];
    twovec_c(ref_surf_normal, Z_axis_ID, z_axis, X_axis_ID, ref_to_surf_rotation);
  }
  else
  {
    // For a Z-up frame, we use the surface normal, but modify the
    // twovec_c() call slightly to produce a X-East, Y-North, and Z-up
    // (ENU) which is common in GIS work.
    twovec_c(ref_surf_normal, Z_axis_ID, z_axis, Y_axis_ID, ref_to_surf_rotation);
  }

  // Compute target state in reference body frame with a single call to SPKEZR
  string frameName = "IAU_" + referenceBody;
  spkezr_c(targetBody.c_str(), ephemTime, frameName.c_str(), "LT+S",
	   referenceBody.c_str(), state, &lightTime);
	
  // Extract 3-component target position from the 6-component state.
  vpack_c(state[0], state[1], state[2], target_in_ref_frame);

  // Calculate vector from surface to target in body reference frame.
  // I.e., target_in_ref_frame - ref_surf_point
  vsub_c(target_in_ref_frame, ref_surf_point, translation);

  // Rotate the target vector from body reference frame to surface
  // frame. This will be the translation component of the transform.
  mxv_c(ref_to_surf_rotation, translation, translation);

  // Get rotation from reference body to target body. We want the
  // matrix that will transform vectors in target body frame to
  // reference body frame.
  string targetFrameName = "IAU_" + targetBody;
  pxform_c(targetFrameName.c_str(), frameName.c_str(), ephemTime,
	   rotation);
  
  // Concatenate the the reference body to surface rotation with the
  // reference body to target body rotation.
  mxm_c(ref_to_surf_rotation, rotation, surf_to_target_rotation);

  // The following assumes column vector notation, and row-major
  // storage. It transforms points given in surface frame to target
  // body frame.
  transform[0] = surf_to_target_rotation[0][0];
  transform[1] = surf_to_target_rotation[0][1];
  transform[2] = surf_to_target_rotation[0][2];
  transform[4] = surf_to_target_rotation[1][0];
  transform[5] = surf_to_target_rotation[1][1];
  transform[6] = surf_to_target_rotation[1][2];
  transform[8] = surf_to_target_rotation[2][0];
  transform[9] = surf_to_target_rotation[2][1];
  transform[10] = surf_to_target_rotation[2][2];
  transform[3] = translation[0];
  transform[7] = translation[1];
  transform[11] = translation[2];
  transform[12] = 0.0;
  transform[13] = 0.0;
  transform[14] = 0.0;
  transform[15] = 1.0;

  // Store vector to body for computation of sun occultation
  m_bodyVecMap[targetBody][0] = transform[3];
  m_bodyVecMap[targetBody][1] = transform[7];
  m_bodyVecMap[targetBody][2] = transform[11];
}

void Ephemeris::VectorToTarget(const string& referenceBody,
			       const float64_ow lat,
			       const float64_ow lon,
			       const float64_ow elev,
			       const string& targetBody,
			       const string& time,
			       float64_ow out_vec[3])
{
  SpiceDouble z_axis[3] = {0, 0, 1};
  SpiceDouble ref_center[3] = {0, 0, 0};
  SpiceDouble vec_from_ref_center[3];
  SpiceDouble ref_radii[3];
  SpiceInt max_radii_dim = sizeof(ref_radii)/sizeof(ref_radii[0]);
  SpiceDouble state[6];
  SpiceDouble ref_surf_point[3];
  SpiceDouble ref_surf_normal[3];
  SpiceDouble ref_to_surf_rotation[3][3];
  SpiceDouble target_in_ref_frame[3];
  SpiceDouble target_vec[3];
  SpiceDouble lightTime;
  SpiceDouble ephemTime;
  SpiceInt dim;
  logical found;		// SpiceInt, logical & integer are SPICE types
  integer ref_body_id;

  // Convert the time string to ephemeris time
  str2et_c(time.c_str(), &ephemTime);

  // Get radius of reference body - Deprecated
  // zzbodn2c_((char*) referenceBody.c_str(), &ref_body_id, &found, referenceBody.length());
  // bodvar_c(ref_body_id, "RADII", &dim, ref_radii);

  // Get radius of reference body
  bodvrd_c(referenceBody.c_str(), "RADII", max_radii_dim, &dim, ref_radii);

  // Compute vector from reference body center to specified lat, lon
  // (assumes planetocentric lat/lon)
  latrec_c(1.0, lon * rpd_c(), lat * rpd_c(), vec_from_ref_center);

  // Compute surface point on reference body in rectangular
  // coordinates and with ellipsoid
  surfpt_c(ref_center, vec_from_ref_center, ref_radii[0], ref_radii[1],
	   ref_radii[2], ref_surf_point, (int*) &found);

  // Compute surface normal
  surfnm_c(ref_radii[0], ref_radii[1], ref_radii[2], ref_surf_point,
	   ref_surf_normal);

  // Compute surface frame rotation matrix - as called, twovec_c
  // defines a surface frame with X-north, Y-west, and Z-up
  if (m_z_down_surface_frame)
  {
    // If Z down surface frame (typical for missions), we rotate 180
    // degrees about X-axis here.
    SpiceDouble ref_to_surf_rotation_z_up[3][3];
    twovec_c(ref_surf_normal, 3, z_axis, 1, ref_to_surf_rotation_z_up);
    const SpiceInt X_axis = 1;
    rotmat_c(ref_to_surf_rotation_z_up, 180.0*rpd_c(), X_axis,
	     ref_to_surf_rotation);
  }
  else
  {
    twovec_c(ref_surf_normal, 3, z_axis, 1, ref_to_surf_rotation);
  }
  
  // Compute target state in reference frame with a single call to SPKEZR
  string frameName = "IAU_" + referenceBody;
  spkezr_c(targetBody.c_str(), ephemTime, frameName.c_str(), "LT+S",
	   referenceBody.c_str(), state, &lightTime);

  if (failed_c())  // If we have an error, return early
  {
    // Reset the SPICE error mechanism.
    reset_c();

    out_vec[0] = 0;
    out_vec[1] = 0;
    out_vec[2] = 0;
    return;
  }

  // Extract target position from this 6-component state.
  vpack_c(state[0], state[1], state[2], target_in_ref_frame);

  // Compute target's pointing vector in reference frame.
  vsub_c(target_in_ref_frame, ref_surf_point, target_vec);

  // Rotate target vector by reference frame to surface frame rotation
  mxv_c(ref_to_surf_rotation, target_vec, target_vec);

  // NOTE: we ignore the translation to the surface. Should be a
  // negligible effect in interplanetary cases. Probably
  // non-negligible for orbiters.

  // Normalize vector to target
  vhat_c(target_vec, target_vec);
  
  // Copy the target vector to the output, only done in case the
  // out_vec type differs from SpiceDouble
  out_vec[0] = target_vec[0];
  out_vec[1] = target_vec[1];
  out_vec[2] = target_vec[2];

  // Compute target's AZ and EL in surface frame. */
  // *out_azim = dpr_c() * fmod(-atan2(target_vec[1], target_vec[0]) + twopi_c(), twopi_c());
  // *out_elev = asin(target_vec[2] ) * dpr_c();
}

void Ephemeris::PlanetoGraphicToCentric(const string& planetaryBody,
					const float64_ow in_lat, const float64_ow in_lon,
					float64_ow& out_lat, float64_ow& out_lon)
{
  SpiceDouble radii[3];
  SpiceInt max_radii_dim = sizeof(radii)/sizeof(radii[0]);
  SpiceInt dim(0);

  // Get radius of body -- Deprecated
  // zzbodn2c_((char*) planetaryBody.c_str(), &body_id, &found, planetaryBody.length());
  // bodvar_c(body_id, "RADII", &dim, radii);

  // Get radius of body
  bodvrd_c(planetaryBody.c_str(), "RADII", max_radii_dim, &dim, radii);

  // Convert planetographic latitude to planetocentric
  float64_ow f = 1.0 - (radii[2] / radii[0]); // f is the "flattening"
  float64_ow e_sqr = 2.0 * f - f * f;	      // e_sqr is the eccentricity squared
  // rpd_c() returns radians per degree
  out_lat = (atan((1.0 - e_sqr) * tan(in_lat * rpd_c()))) / rpd_c();

  // Convert planetographic longitude if necessary
  SpiceInt body_id(0);
  SpiceBoolean found(0);
  bodn2c_c(planetaryBody.c_str(), &body_id, &found);
  // Unknown body errors should be signalled above, so we don't warn
  // here.
  if (found)
  {
    // plnsns() apparently only exists as a Fortran function. There is
    // no CSPICE version yet.
    SpiceInt lon_conversion = plnsns_(&body_id);
    if (lon_conversion != 0)
      out_lon = lon_conversion * in_lon;
    else
      cerr << "WARNING [PlanetoGraphicToCentric()]: "
	   << "longitude sense not available for"
	   << planetaryBody << ". Longitude conversion skipped."
	   << endl;
  }
}

double Ephemeris::FractionVisible(const string& referenceBody,
                                  const string& occulterBody)
{
  // Get radius of reference body
  const SpiceInt max_radii_dim = 3;
  SpiceInt dim;
  SpiceDouble radii[max_radii_dim];
  bodvrd_c(referenceBody.c_str(), "RADII", max_radii_dim, &dim, radii);
  double ref_radius = 0.0;
  for (int i=0; i<dim; i++) {
    ref_radius += radii[i];
  }
  ref_radius /= double(dim);

  // Get radius of occulter
  bodvrd_c(occulterBody.c_str(), "RADII", max_radii_dim, &dim, radii);
  double occ_radius = 0.0;
  for (int i=0; i<dim; i++) {
    occ_radius += radii[i];
  }
  occ_radius /= double(dim);

  // These vectors should have been computed already
  assert(m_bodyVecMap.find(referenceBody) != m_bodyVecMap.end());
  assert(m_bodyVecMap.find(occulterBody) != m_bodyVecMap.end());

  // Make radii relative from the point of view of the observer, and
  // normalize vectors to the bodies
  Vec3& ref_vec = m_bodyVecMap[referenceBody];
  ref_radius /= ref_vec.normalize();
  Vec3& occ_vec = m_bodyVecMap[occulterBody];
  occ_radius /= occ_vec.normalize();

  // Separation between circles can be defined as the angle between vectors
  // that point to them
  const double d = acos(ref_vec[0] * occ_vec[0] + ref_vec[1] * occ_vec[1] + ref_vec[2] * occ_vec[2]);

  // No occultation
  if (d >= ref_radius + occ_radius)
    return 1.0;

  // Full occultation
  if (occ_radius >= ref_radius && d <= occ_radius - ref_radius )
    return 0.0;

  // Maximum occultation possible when occulter is apparently smaller than ref body
  const double ref_area = M_PI * ref_radius * ref_radius;
  if (ref_radius >= occ_radius && d <= ref_radius - occ_radius)
  {
    const double occ_area = M_PI * occ_radius * occ_radius;
    return (ref_area - occ_area) / ref_area;
  }

  // Solution for intersection area of two circles taken from:
  // https://stackoverflow.com/questions/4247889/area-of-intersection-between-two-circles
  double r = ref_radius;
  double R = occ_radius;
  if(R < r){
    r = occ_radius;
    R = ref_radius;
  }
  double intersection_area = (r * r * acos((d * d + r * r - R * R) / (2.0 * d * r))) +
                             (R * R * acos((d * d + R * R - r * r) / (2.0 * d * R))) -
                             (0.5 * sqrt((-d + r + R) * (d + r - R) * (d - r + R) * (d + r + R)));

  return (ref_area - intersection_area) / ref_area;
}
