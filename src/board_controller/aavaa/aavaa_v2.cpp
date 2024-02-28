#include <vector>

#include "custom_cast.h"
#include "aavaa_v2.h"
#include "serial.h"
#include "timestamp.h"

#define START_BYTE 0xA0
#define END_BYTE 0xC0


int AAVAAv2::config_board (std::string conf, std::string &response)
{
    if (gain_tracker.apply_config (conf) == (int)OpenBCICommandTypes::INVALID_COMMAND)
    {
        safe_logger (spdlog::level::warn, "invalid command: {}", conf.c_str ());
        return (int)BrainFlowExitCodes::INVALID_ARGUMENTS_ERROR;
    }
    int res = OpenBCISerialBoard::config_board (conf, response);
    if (res != (int)BrainFlowExitCodes::STATUS_OK)
    {
        gain_tracker.revert_config ();
    }
    return res;
}

void AAVAAv2::read_thread ()
{
    // format is the same as for cyton but need to join two packages together
    /*
        Byte 1: 0xA0
        Byte 2: Sample Number
        Bytes 3-5: Data value for EEG channel 1
        Bytes 6-8: Data value for EEG channel 2
        Bytes 9-11: Data value for EEG channel 3
        Bytes 12-14: Data value for EEG channel 4
        Bytes 15-17: Data value for EEG channel 5
        Bytes 18-20: Data value for EEG channel 6
        Bytes 21-23: Data value for EEG channel 6
        Bytes 24-26: Data value for EEG channel 8
        Aux Data Bytes 27-32: 6 bytes of data
        Byte 33: 0xCX where X is 0-F in hex
    */
    int res;
    int num_rows = board_descr["default"]["num_rows"];
    while (keep_alive)
    {
        // check start byte
        uint8_t *data_frame = new uint8_t[SIZE_OF_DATA_FRAME];
        res = serial->read_from_serial_port (data_frame, 1);
        if (res != 1)
        {
            safe_logger (spdlog::level::debug, "unable to read 1 byte");
            continue;
        }
        if (data_frame[0] != START_BYTE)
        {
            continue;
        }

        int remaining_bytes = SIZE_OF_DATA_FRAME - 1;
        int pos = 1;
        while ((remaining_bytes > 0) && (keep_alive))
        {
            res = serial->read_from_serial_port (data_frame + pos, remaining_bytes);
            remaining_bytes -= res;
            pos += res;
        }
        if (!keep_alive)
        {
            break;
        }

        if (data_frame[SIZE_OF_DATA_FRAME - 1] != END_BYTE)
        {
            safe_logger (spdlog::level::warn, "Wrong end byte @{} : {}",
                SIZE_OF_DATA_FRAME, data_frame[SIZE_OF_DATA_FRAME - 1]);
            continue;
        }

        double *package = new double[num_rows];
        for (int i = 0; i < num_rows; i++)
        {
            package[i] = 0.0;
        }
        // For Cyton Daisy Serial, sample IDs are sequenctial
        // (0, 1, 2, 3...) so even sample IDs are the first sample (daisy)
        // and odd sample IDs are the second sample (cyton)
        // after the second sample, we commit the package below

        std::vector<int> eeg_channels = board_descr["default"]["eeg_channels"];

        // package num
        package[board_descr["default"]["package_num_channel"].get<int> ()] = (double)data_frame[1];

        // eeg
        for (unsigned int i = 0; i < eeg_channels.size (); i++)
        {
            // all the eeg_channels are 4 bytes and we skip the START byte and package number byte
            float f_value;
            std::memcpy (&f_value, data_frame + 2 + 4 * i, sizeof (f_value));
            double d_value = static_cast<double> (f_value);
            package[eeg_channels[i]] = EEG_SCALE * d_value;
        }

        try
        {
            // safe_logger (spdlog::level::debug, error_message);
            package[board_descr["default"]["rotation_channels"][0].get<int> ()] =
                IMU_SCALE * cast_16bit_to_int32 (data_frame + 34);
            package[board_descr["default"]["rotation_channels"][1].get<int> ()] =
                IMU_SCALE * cast_16bit_to_int32 (data_frame + 36);
            package[board_descr["default"]["rotation_channels"][2].get<int> ()] =
                IMU_SCALE * cast_16bit_to_int32 (data_frame + 38);
        } catch (const std::exception &e)
        {
            std::string error_message = std::string ("Exception occurred: ") + e.what ();
            safe_logger (spdlog::level::warn, error_message);
        }

        // battery byte
        package[board_descr["default"]["battery_channel"].get<int> ()] = (double)data_frame[40];

        if (SIZE_OF_DATA_FRAME > 47)
        {
            // gesture byte
            package[board_descr["default"]["other_channels"][0].get<int> ()] =
                (double)data_frame[46];

            // gesture info 1
            package[board_descr["default"]["other_channels"][1].get<int> ()] = cast_16bit_to_int32 (data_frame + 47);

            // gesture info 2
            package[board_descr["default"]["other_channels"][2].get<int> ()] = cast_16bit_to_int32 (data_frame + 49);
        }

        // imu status byte
        package[board_descr["default"]["other_channels"][3].get<int> ()] = (double)data_frame[41];

        // timestamp
        package[board_descr["default"]["other_channels"][4].get<int> ()] =
                    *reinterpret_cast<uint32_t *> (data_frame + 42);

        package[board_descr["default"]["timestamp_channel"].get<int> ()] = get_timestamp ();

        push_package (package);
        delete[] package;
        delete[] data_frame;
    }
}