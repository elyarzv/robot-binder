#include "ais_dimming_module/IrradianceModels/QuantitativeIrradianceModel.h"
using namespace ais_dimming_module;

QuantitativeIrradianceModel::QuantitativeIrradianceModel()
{
    setModel(658.6, 613.0, 30.0, 29.0);
}

QuantitativeIrradianceModel::~QuantitativeIrradianceModel()
{
}

void QuantitativeIrradianceModel::setModel(double max, double stable_max, double min, double min_time)
{
    MAX_POSSIBLE_IRRADIANCE = max; //Defined as stable Intensity
    MAX_STABLE_IRRADIANCE = stable_max;
    MIN_IRRADIANCE = min;
    MIN_WARMUP_TIME = min_time; //seconds
}

double QuantitativeIrradianceModel::getIrradiancePercentage(double input_time)
{
    return calculateIrradiance(input_time) / MAX_STABLE_IRRADIANCE;
}

double QuantitativeIrradianceModel::roundToNearestDecimalHalf(double input_time)
{
    return round(input_time * 2.0) / 2.0;
}

double QuantitativeIrradianceModel::calculateIrradiance(double input_time)
{   

    // Uses 4 equations using y=mx+b form.
    if (input_time < MIN_WARMUP_TIME)
    {
        return 0.0;
    }
    else if (input_time < 83.0)
    {
        return 2.4891 * input_time - 38.2931;
    }
    else if (input_time < 200.0)
    {
        return 4.1906 * input_time - 179.519;
    }
    else if (input_time < 500.0)
    {
        return -0.1516 * input_time + 688.933;
    }
    else
    {
        return MAX_STABLE_IRRADIANCE;
    }
}

double QuantitativeIrradianceModel::calculateTimeToIrradiance(double irrad_value)
{   
    // Inverse equation (x = (y-b)/m) equations used from calculateIrradiance() and Google Drive 
    // Documents listed in header file
    if (irrad_value <= 33.89)
    {
        return 0;
    } 
    else if (irrad_value <= 33.89)
    {
        return ((irrad_value + 38.2931) / 2.4891);
    }
    else if (irrad_value <= 168.3)
    {
        return ((irrad_value + 179.519) / 4.1906);
    }
    else
    {   
        // Since UV has a bump as it ramps up to maximum UV intensity, there exists overlapping
        // regions of time that correspond to a single UV intensity, we assume maximum UV intensity
        // at maximum time for now. 
        // return 200.0;
        return 500.0;
    }
}

double QuantitativeIrradianceModel::getMaxStableIrradiance()
{
    return MAX_STABLE_IRRADIANCE;
}

double QuantitativeIrradianceModel::getMaxIrradiance()
{
    return MAX_POSSIBLE_IRRADIANCE;
}