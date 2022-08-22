/*
    AIS CONFIDENTIAL
*/

#ifndef AIS_DIMMING_MODULE_QUANT_IRRADIANCE_MODEL_H_
#define AIS_DIMMING_MODULE_QUANT_IRRADIANCE_MODEL_H_

#include <math.h>

using std::round;

namespace ais_dimming_module
{
/*
    @brief this class uses the quantitative data gathered from the following links. Provides an
    estimation of various Irradiances given a 180W Philips TUV bulb at 1.0m distance.
    https://docs.google.com/document/d/1iPvG2H5mHk1sqiFqKA7It3g0mWlUgnps_VzTrsVJ7Fg/edit#
    https://docs.google.com/spreadsheets/d/1FtCwwCx27FFRQZwiw-vbLEt3H9T9Hvlmu0NHakWf9gs/edit#gid=0
    https://docs.google.com/document/d/1sSNAi8VR5gOOEOIgHZ6LKEaLW8U_npFpUlKd7KKz9wc/edit#heading=h.q97laexkvll3

*/
class QuantitativeIrradianceModel
{
public:
    QuantitativeIrradianceModel();
    ~QuantitativeIrradianceModel();

    /**
     * @brief sets UV Lamp model parameters such as warmup_time, max stable irradiance, etc
     * 
     * @param max Max peak irradiance value
     * @param stable_max Max stable 
     * @param min minimum irradiance value 
     * @param min_time time for UV lamp to warm on
     */
    void setModel(double max, double stable_max, double min, double min_time);
    /**
     * @brief Get the Irradiance Percentage object
     * 
     * @param input_time input using ros::Duration.toSec(), time (s) 
     * @return double percentage from 0.0 to 1.0
     */
    double getIrradiancePercentage(double input_time);
    
    /**
     * @brief round to the nearest X.0 or Y.5 value
     * 
     * @param input_time time in secounds
     * @return double return value rounded to nearest X.0 ex: 1.25 -> 1.5, 1.5 -> 1.5, 1.99 -> 2.0
     */
    double roundToNearestDecimalHalf(double input_time);
    
    /**
     * @brief calculate the irradiance value based on input time using 4 linear equations
     * 
     * @param input_time time in seconds
     * @return double irradiance value (W/m^2)
     */
    double calculateIrradiance(double input_time);
    /**
     * @brief Calculate time (s) based on irradiance value. Returns first irradiance occurence
     * if more than 1 time value exists
     * 
     * @param irrad_value double Irradiance value
     * @return double time in seconds
     */
    double calculateTimeToIrradiance(double irrad_value);
    /**
     * @brief Get the Max Stable Irradiance for UV model
     * 
     * @return double 
     */
    double getMaxStableIrradiance();
    /**
     * @brief Get the Max Irradiance value for UV model (180W Philips Lamp)
     * 
     * @return double 
     */
    double getMaxIrradiance();

private:
    double MAX_POSSIBLE_IRRADIANCE;
    double MAX_STABLE_IRRADIANCE;
    double MIN_IRRADIANCE;
    double MIN_WARMUP_TIME;
}; 
}// End of ais_dimming_module namespace;
#endif // AIS_DIMMING_MODULE_QUANT_IRRADIANCE_MODEL_H_