#include "zone_converter.h"

ZONE_converter::ZONE_converter(){}

void ZONE_converter::UTM(double lat, double lon, double eastingUtmzone, double northingUtmzone)
{    
    double m0_a11 = (std::pow(wgs84_data.first_eccentricity, 2)/4);
    double m0_a12 = (std::pow(wgs84_data.first_eccentricity, 4)/64);
    double m0_a13 = (std::pow(wgs84_data.first_eccentricity, 6))/256;
    double m0 = 1 - m0_a11 - 3*m0_a12 - 5*m0_a13;

    double m1_a11 = (std::pow(wgs84_data.first_eccentricity, 2))/8;
    double m1_a12 = (std::pow(wgs84_data.first_eccentricity, 4))/32;
    double m1_a13 = (std::pow(wgs84_data.first_eccentricity, 6))/1024;
    double m1 = -(3*m1_a11 + 3*m1_a12 + 45*m1_a13);

    double m2_a11 = (std::pow(wgs84_data.first_eccentricity, 4))/256;
    double m2_a12 = (std::pow(wgs84_data.first_eccentricity, 6))/1024;
    double m2 = 15*m2_a11 + 45*m2_a12;

    double m3_a11 = (std::pow(wgs84_data.first_eccentricity, 6))/3072;
    double m3 = -35*m3_a11;

    // calculation of the central meridian
    int centralMeridian = ((lon >= 0.0)
      ? (static_cast<int>(lon) - (static_cast<int>(lon)) % 6 + 3)
      : (static_cast<int>(lon) - (static_cast<int>(lon)) % 6 - 3));

//    double rlat  = degreeToRad_reader.D2R;
//    double rlon  = degreeToRad_reader.D2R;
//    double rlon0 = centralMeridian*degreeToRad_reader.D2R;

//    double slat = std::sin(rlat);
//    double clat = std::cos(rlat);
//    double tlat = std::tan(rlat);

//    double fn = (lat > 0) ? utm_data.northen_emisphere : utm_data.southern_hemisphere;

//    double T = tlat*tlat;
//    double C = (wgs84_data.first_eccentricity*wgs84_data.first_eccentricity)*clat*clat;
//    double A = (rlon - rlon0)*clat;
//    double M = (wgs84_data.semi_major_axis_a)*(m0*rlat + m1*std::sin(2*rlat) + m2*std::sin(4*rlat) + m3*std::sin(6*rlat));
//    // radius of curvature on the plane of the prime vertical
//    double Rn = wgs84_data.semi_major_axis_a/(std::sqrt(1 - std::pow((wgs84_data.first_eccentricity), 2)*slat*slat));
//     // radius of Curvature in the plane os the meridian
//    double Rc = ((wgs84_data.semi_major_axis_a)*(1 - ((wgs84_data.first_eccentricity)*(wgs84_data.first_eccentricity))))/(1 - ((wgs84_data.first_eccentricity)*(wgs84_data.first_eccentricity))*std::pow(std::sin(rlat), 2));
//    // computation of the easting-northing coordinate
//    eastingUtmzone = utm_data.point_scale_factor*Rn*(A + ((1-T+C)*(std::pow(A, 3)/6))+(5-18*T + std::pow(T,2) + 72*C - 58*(std::pow(wgs84_data.second_eccentricity, 2)))*(std::pow(A, 5))/120);
//    northingUtmzone = utm_data.point_scale_factor*((M - 0.0)+Rn*tlat*(((A*A)/2) + (((std::pow(A, 4))/24)*(5-T+9*C+4*C*C)) + (61 - 58*T + T*T + 600*C - 330*(std::pow(wgs84_data.second_eccentricity, 2))*((std::pow(A, 6))/720))));
//    (void) Rc;
//    (void) fn;
    return;
}

void ZONE_converter::fromLatLontoUTM(double Lat, double Long, double &UTMNorthing,
                                     double &UTMEasting, std::string &UTMZone)
{
//    const double a                     = wgs84_data.semi_major_axis_a;
//    const double scaleFactor           = utm_data.point_scale_factor;
//    const double firstecc              = wgs84_data.first_eccentricity;
//    const double secondecc             = wgs84_data.second_eccentricity;
//    const double a = 6378137.0;


    double a = WGS84_A;
    double eccSquared = UTM_E2;
    double k0 = UTM_K0;

    double LongOrigin;
    double eccPrimeSquared;
    double N, T, C, A, M;


    //double Rn, T, C, A, M;

    // ensure that longitude is between -180 and +179
//    double longTransf = (lon+180)- static_cast<int>((lon+180)/360)*360-180;
    //double latRadians2 = lat*degreeToRad_reader.D2R;
//    double latRadians = lat*(3.14/180);
    //std::cout<< latRadians << std::endl;
    //std::cout<< latRadians2 << std::endl;

    //double lonRadians = longTransf*degreeToRad_reader.D2R;
//    double lonRadians = lon*(3.14/180);


    double LongTemp = (Long+180)-static_cast<int>((Long+180)/360)*360-180;

    double LatRad = Lat*RADIANS_PER_DEGREE;
    double LongRad = LongTemp*RADIANS_PER_DEGREE;
    double LongOriginRad;


    //std::cout<< lonRadians << std::endl;


    int zoneNumber;
    zoneNumber = static_cast<int>((LongTemp + 180)/6) + 1;
    adjustForNorway(Lat, Long);
    adjustForSvalbard(Lat, Long);

    int longXYZ = (zoneNumber - 1)*6 - 180 + 3;
    //int lonXYZRadians = longXYZ * (int)radToDeg_reader.R2D;
    //double lonXYZRadians = longXYZ * (3.14/180);
    LongOriginRad = LongOrigin * RADIANS_PER_DEGREE;


    // Calculation of the UTM zone from latitude and longitude
    char zoneBufer[] = {0,0,0,0};
    // Composes a string with the same text that would be printed if format was used on printf,
    // but instead of being printed, the content is stored as string in the buffer
    std::snprintf(zoneBufer, sizeof (zoneBufer), "%d%c", zoneNumber, allOtherZones(Lat));
    UTMZone = std::string(zoneBufer);



    eccPrimeSquared = (eccSquared)/(1-eccSquared);

    N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
    T = tan(LatRad)*tan(LatRad);
    C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
    A = cos(LatRad)*(LongRad-LongOriginRad);

    M = a*((1 - eccSquared/4 - 3*eccSquared*eccSquared/64
                  - 5*eccSquared*eccSquared*eccSquared/256) * LatRad
                 - (3*eccSquared/8 + 3*eccSquared*eccSquared/32
                    + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
                 + (15*eccSquared*eccSquared/256
                    + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad)
                 - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));

    UTMEasting = static_cast<double>
            (k0*N*(A+(1-T+C)*A*A*A/6
                   + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
             + 500000.0);

    UTMNorthing = static_cast<double>
            (k0*(M+N*tan(LatRad)
                 *(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
                   + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));

    if (Lat < 0)
            {
              // 10000000 meter offset for southern hemisphere
              UTMNorthing += 10000000.0;
            }


//    Rn = a/(std::sqrt((1-((firstecc*firstecc)*((std::sin(latRadians))*(std::sin(latRadians)))))));
//    std::cout<< Rn << std::endl;


//    double Rn1 = a/(std::sqrt((1-((0.0818191908*0.0818191908)*((std::sin(latRadians))*(std::sin(latRadians)))))));
//    std::cout<< Rn1 << std::endl;


////    double sin_phi = std::sin(latRadians); 0.0818191908
////    double squared_sin = sin_phi*sin_phi;
////    double denom = (1 - eccentricitySquared*squared_sin);
////    double squareRoot = std::sqrt(denom);
////    Rn = a / squareRoot;
//    double tag = std::cos(latRadians);
//    double e2 = 0.0820944338*0.0820944338;

//    C = (e2)*(tag*tag);

//    T = std::tan(latRadians)*std::tan(latRadians);
//    //C = secondecc*secondecc*std::cos(latRadians)*std::cos(latRadians);
//    A = (lonRadians-lonXYZRadians)*std::cos(latRadians);

////    double ecc0_a11 = (1 - std::pow(firstecc, 2)/4 - 3*std::pow(firstecc, 4)/64 - 5*std::pow(firstecc, 6)/256)*latRadians;
////    double ecc0_a12 = (3*std::pow(firstecc, 2)/8 + 3*std::pow(firstecc, 4)/32 + 45*std::pow(firstecc, 6)/256)*std::sin(2*latRadians);
////    double ecc0_a13 = (15*std::pow(firstecc, 4)/256 + 45*std::pow(firstecc, 6)/1024)*std::sin(4*latRadians);
////    double ecc0_a14 = (34*std::pow(firstecc, 6)/3072)*std::sin(6+latRadians);

////    M = a*(ecc0_a11-ecc0_a12+ecc0_a13-ecc0_a14);


//    M = a*((1 - (secondecc*secondecc)/4 - 3*((secondecc*secondecc*secondecc*secondecc)/64
//     - 5*secondecc*secondecc*secondecc*secondecc*secondecc*secondecc/256) * latRadians
//     - (3*secondecc*secondecc/8 + 3*secondecc*secondecc*secondecc*secondecc/32
//     + 45*secondecc*secondecc*secondecc*secondecc*secondecc*secondecc/1024)*sin(2*latRadians)
//     + (15*secondecc*secondecc*secondecc*secondecc/256
//     + 45*secondecc*secondecc*secondecc*secondecc*secondecc*secondecc/1024)*sin(4*latRadians)
//     - (35*secondecc*secondecc*secondecc*secondecc*secondecc*secondecc/3072)*sin(6*latRadians)));

////    M = a*((1 - eccentricitySquared/4 - 3*eccentricityPowerFour/64 - 5*eccentricityPowerSix/256)*latRadians
////           - (3*eccentricitySquared/8 + 3*eccentricityPowerFour/32 + 45*eccentricityPowerSix/1024)*std::sin(2*latRadians)
////           + (15*eccentricityPowerFour/256 + 45*eccentricityPowerSix/1024)*std::sin(4*latRadians)
////           - (35*eccentricityPowerSix/3072)*std::sin(6*latRadians));

//    //double multiplierEast = scaleFactor*Rn;
//    //double m0_a11E = (A + (1-T+C)*(std::pow(A, 2)/6));
//    //UTMEasting = static_cast<double>(scaleFactor*Rn*(A + (1-T+C)*(std::pow(A, 2)/6))+((5 - 18*T + std::pow(T, 2) + 72*C - 58*(secondecc*2))*(std::pow(A, 5)/120)));

//    UTMEasting = (scaleFactor*Rn1*((A+(1-T+C)*((A*A*A)/6)) + (5-18*T+(T*T)+72*C-58*(secondecc*secondecc))*((A*A*A*A*A)/120)) + 500000.0);
//    //UTMEasting = scaleFactor*Rn*m0_a12E;
//    UTMNorthing =
//     (scaleFactor*(M+Rn1*tan(latRadians)
//     *(((A*A)/2)+((5-T+9*C+4*(C*C))*((A*A*A*A)/24))
//     + ((61-58*T+(T*T)+600*C-330*(secondecc*secondecc))*((A*A*A*A*A*A)/720)))));




////    double multiplierNorth = scaleFactor;
////    double m0_a11N = M;
////    double m0_a12N = Rn*std::tan(latRadians);
////    double m0_a13N = (((std::pow(A, 2))/2) + ((5 - T + 9*C + 4*std::pow(C, 2))*((std::pow(A, 4))/24)));
////    double m0_a14N = ((61 - 58*T + std::pow(T, 2) + 600*C - 330*eccentricityPrimeSquared)*((std::pow(A, 6))/720));
////    UTMNorthing = (multiplierNorth*(m0_a11N + m0_a12N*(m0_a13N + m0_a14N)));

//    if (Lat < 0)
//    {
//        // 10000000 meter offset for southern hemisphere
//        UTMNorthing = UTMNorthing + 10000000.0;
//    }
}

char ZONE_converter::adjustForNorway(double Lat, double Lon)
{
    double longTransf = (Lon + 180) - static_cast<int>((Lon+180)/360)*360-180;
    int zoneNumber;
    zoneNumber = static_cast<int>((longTransf + 180)/6) +1;
    if (Lat >= 56.0 && Lat < 64.0 && longTransf >= 3.0 && longTransf < 12.0)
        zoneNumber = 32;
}

void ZONE_converter::adjustForSvalbard(double Lat, double Lon)
{
    double longTransf = (Lon + 180) - static_cast<int>((Lon+180)/360)*360-180;
    int zoneNumber;
    zoneNumber = static_cast<int>((longTransf + 180)/6) +1;
    if(Lat >= 72.0 && Lat < 84.0)
    {
        if     (longTransf >= 0.0  && longTransf < 9.0)  zoneNumber = 31 ;
        else if(longTransf >= 9.0  && longTransf < 21.0) zoneNumber = 33;
        else if(longTransf >= 21.0 && longTransf < 33.0) zoneNumber = 35;
        else if(longTransf >= 33.0 && longTransf < 42.0) zoneNumber = 37;
    }
}

char ZONE_converter::allOtherZones(double lat)
{
      if ((72 > lat) && (lat >= 64))        return UTMidentifierLeter::W;
      else if ((56  > lat) && (lat >= 48 )) return UTMidentifierLeter::U;
      else if ((48  > lat) && (lat >= 40 )) return UTMidentifierLeter::T;
      else if ((40  > lat) && (lat >= 32 )) return UTMidentifierLeter::S;
      else if ((32  > lat) && (lat >= 24 )) return UTMidentifierLeter::R;
      else if ((24  > lat) && (lat >= 16 )) return UTMidentifierLeter::Q;
      else if ((16  > lat) && (lat >= 8  )) return UTMidentifierLeter::P;
      else if (( 8  > lat) && (lat >= 0  )) return UTMidentifierLeter::N;
      else if (( 0  > lat) && (lat >= -8 )) return UTMidentifierLeter::M;
      else if ((-8  > lat) && (lat >= -16)) return UTMidentifierLeter::L;
      else if ((-16 > lat) && (lat >= -24)) return UTMidentifierLeter::K;
      else if ((-24 > lat) && (lat >= -32)) return UTMidentifierLeter::J;
      else if ((-32 > lat) && (lat >= -40)) return UTMidentifierLeter::H;
      else if ((-40 > lat) && (lat >= -48)) return UTMidentifierLeter::G;
      else if ((-48 > lat) && (lat >= -56)) return UTMidentifierLeter::F;
      else if ((-56 > lat) && (lat >= -64)) return UTMidentifierLeter::E;
      else if ((-64 > lat) && (lat >= -72)) return UTMidentifierLeter::D;
      else if ((-72 > lat) && (lat >= -80)) return UTMidentifierLeter::C;
            // 'Z' is an error flag, the Latitude is outside the UTM limits
      else return UTMidentifierLeter::Z;
}








