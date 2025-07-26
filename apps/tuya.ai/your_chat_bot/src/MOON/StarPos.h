#ifndef __STARPOS_H__
#define __STARPOS_H__

double utc_to_julian_day(TIME_T utc_time);
void calculate_moon_position(double jd, double *moon_lon, double *moon_lat);
void ecliptic_to_equatorial(double lon_ecl, double lat_ecl, double jd, 
                           double *ra, double *dec);
double calculate_local_sidereal_time(double jd, double longitude);
void calculate_alt_az(double ra, double dec, double lst, double lat, 
                      double *alt, double *az);
float calculate_moon_direction(void);
float get_current_azimuth(void);
int check_moon_alignment(void);

#endif // __STARPOS_H__