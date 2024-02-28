#include <condition_variable>
#include <deque>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include "AAVAA_v3.h"
#include "board.h"
#include "board_controller.h"


class AAVAAEar : public Board
{
public:
    AAVAAEar (struct BrainFlowInputParams params);
    ~AAVAAEar ();

    int prepare_session ();
    int start_stream (int buffer_size, const char *streamer_params);
    int stop_stream ();
    int release_session ();
    int config_board (std::string config, std::string &response);
    int send_command (std::string config);
    std::string fix_command(std::string config);

    void read_data (
        simpleble_uuid_t service, simpleble_uuid_t characteristic, uint8_t *data, size_t size);
    std::deque<uint8_t> Incoming_BLE_Data_Buffer;
    std::string device_status;
    int device_version = 3;

protected:
    const double TIMESTAMP_SCALE_V3 = (double)(4. / 1000.); // 4 ms
    const double TIMESTAMP_SCALE_V4 = (double)(1. / 1000.); // 4 ms
    const double IMU_SCALE = (double)(1. / 100.);
    const double DETECTION_SCALE = 1.0;
    const int Ring_Buffer_Max_Size = 244 * 5;
    const int SIZE_OF_DATA_FRAME = 52;
    const double ADS1299_Vref = 4.5; // reference voltage for ADC in ADS1299
    const double ADS1299_gain = 12.; // assumed gain setting for ADS1299
    const double EEG_SCALE = ADS1299_Vref / float ((pow (2, 23) - 1)) / ADS1299_gain * 1000000.0;

    AAVAAv3 LeftBoard;
    AAVAAv3 RightBoard;
    bool initialized;
    bool is_streaming;
    std::mutex m;
    std::condition_variable cv;
    std::pair<simpleble_uuid_t, simpleble_uuid_t> notified_characteristics;
    std::pair<simpleble_uuid_t, simpleble_uuid_t> write_characteristics;
    std::string start_command_ = "\x62";
    std::string stop_command_ = "\x39";
    std::string start_command = "\x62";
    std::string stop_command = "\x39";
};
