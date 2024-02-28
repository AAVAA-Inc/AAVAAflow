#pragma once

#include <math.h>
#include <string>
#include <vector>

#include "openbci_gain_tracker.h"
#include "openbci_serial_board.h"

class AAVAAv2 : public OpenBCISerialBoard
{
protected:
    CytonDaisyGainTracker gain_tracker;

    void read_thread ();

    const double IMU_SCALE = (double)(1. / 100.);
    const int SIZE_OF_DATA_FRAME = 52;
    const double ADS1299_Vref = 4.5; // reference voltage for ADC in ADS1299
    const double ADS1299_gain = 12.; // assumed gain setting for ADS1299
    const double EEG_SCALE = ADS1299_Vref / float ((pow (2, 23) - 1)) / ADS1299_gain * 1000000.0;

public:
    AAVAAv2 (struct BrainFlowInputParams params)
        : OpenBCISerialBoard (params, (int)BoardIds::AAVAA_V2_BOARD)
    {
    }

    int config_board (std::string config, std::string &response);
};
