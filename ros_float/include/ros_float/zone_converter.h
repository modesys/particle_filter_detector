#ifndef ZONE_CONVERTER_H
#define ZONE_CONVERTER_H

#include <string>
#include <math.h>
#include <cmath>
#include <ctgmath>
#include <stdlib.h>
#include <stdio.h>
#include <stdexcept>

#include "utility.h"

const double RADIANS_PER_DEGREE = M_PI/180.0;
const double DEGREES_PER_RADIAN = 180.0/M_PI;

 // Grid granularity for rounding UTM coordinates to generate MapXY.
 const double grid_size = 100000.0;    // 100 km grid

 // WGS84 Parameters
 #define WGS84_A   6378137.0   // major axis
 #define WGS84_B   6356752.31424518  // minor axis
 #define WGS84_F   0.0033528107    // ellipsoid flattening
 #define WGS84_E   0.0818191908    // first eccentricity
 #define WGS84_EP  0.0820944379    // second eccentricity

 // UTM Parameters
 #define UTM_K0    0.9996               // scale factor
 #define UTM_FE    500000.0             // false easting
 #define UTM_FN_N  0.0                  // false northing, northern hemisphere
 #define UTM_FN_S  10000000.0           // false northing, southern hemisphere
 #define UTM_E2    (WGS84_E*WGS84_E)    // e^2
 #define UTM_E4    (UTM_E2*UTM_E2)      // e^4
 #define UTM_E6    (UTM_E4*UTM_E2)      // e^6
 #define UTM_EP2   (UTM_E2/(1-UTM_E2))  // e'^2


// definition of the World Geodetic System 84
struct WGS84_DATA
{
    const double semi_major_axis_a      = 6378137.0;            // by definition
    const double semi_minor_axis_b      = 6356752.314245;       // by definition
    const double first_eccentricity     = 0.0818191908;         // by calculation
    const double second_eccentricity    = 0.0820944338;         // by calculation
    const double angular_velocity_earth = 72.92115e-6;          // rad/s
    const double gravitational_constant = 3986004.418e8;        // by definition
};

struct UTM_DATA
{
    const double point_scale_factor  = 0.9996;                  // by convention
    const double equatorial_radius   = 6378137.0;               // meters also semi_major_axis_a
    const double northen_emisphere   = 0.0;                     // meter
    const double southern_hemisphere = 10000000.0;              // meter
    const double false_esting        = 500000.0;                // meter by convention
};

enum UTMidentifierLeter {
    X, W, V, U, T, S, R, Q, P, N,
    M, L, K, J, H, G, F, E, D, C, Z
};

struct UTM_LETTER_ZONE
{
    UTMidentifierLeter utmLetterZone;
};

enum UTMIdentifierZone {
    NORWAY, SVALBARD
};

struct UTM_ZONE
{
    UTMIdentifierZone utmZone;
};

class ZONE_converter
{
public:
  ZONE_converter();

  WGS84_DATA wgs84_data;
  UTM_DATA utm_data;

  void UTM(double lat, double lon, double x, double y);

  void fromLatLontoUTM(double Lat, double Lon, double &UTMNorthing, double &UTMEasting,
                       std::string &UTMZone);

//  void fromLatLongToUTM(double lat, double longitude);

  char adjustForNorway(double Lat, double Lon);
  void adjustForSvalbard(double Lat, double Lon);
  char allOtherZones(double lat);

private:
  UTM_LETTER_ZONE letter;
  UTM_ZONE zone;
  double latitude;
  double longitude;
  int current_zone;
  utility *utilFun;

};

#endif // ZONE_CONVERTER_H
