#include <string>
#include <regex>

#include "aavaa_earbud.h"
#include "custom_cast.h"
#include "get_dll_dir.h"
#include "timestamp.h"

#define START_BYTE 0xA0
#define END_BYTE 0xC0

#define AAVAA3_WRITE_CHAR "6e400002-c352-11e5-953d-0002a5d5c51b"
#define AAVAA3_NOTIFY_CHAR "6e400003-c352-11e5-953d-0002a5d5c51b"


int extractBoardVersion(const std::string& inputString) {
    // Define a regular expression pattern to match the desired format
    std::regex pattern("@AAVAA-(\\d+) ");

    // Search for the pattern in the input string
    std::smatch match;
    if (std::regex_search(inputString, match, pattern)) {
        // The captured group at index 1 contains the number
        std::string numberStr = match[1].str();

        // Convert the string to an integer
        int number = std::stoi(numberStr);

        // Return the extracted number
        return number;
    } else {
        // Return a default value or throw an exception based on your requirements
        // In this example, we return -1 to indicate an error
        return -1;
    }
}

AAVAAEar::AAVAAEar (struct BrainFlowInputParams params)
{
    initialized = false;
    is_streaming = false;
}

AAVAAEar::~AAVAAEar ()
{
    release_session ();
}

int AAVAAEar::prepare_session ()
{
    if (initialized)
    {
        safe_logger (spdlog::level::info, "Session is already prepared");
        return (int)BrainFlowExitCodes::STATUS_OK;
    }

    int res = LeftBoard.prepare_session();
    if (res == (int)BrainFlowExitCodes::STATUS_OK) {
        res = RightBoard.prepare_session();
    }
    if (res == (int)BrainFlowExitCodes::STATUS_OK) {
        initialized = true;
    }
    return res;
}

int AAVAAEar::start_stream (int buffer_size, const char *streamer_params)
{
    if (!initialized)
    {
        return (int)BrainFlowExitCodes::BOARD_NOT_CREATED_ERROR;
    }
    if (is_streaming)
    {
        return (int)BrainFlowExitCodes::STREAM_ALREADY_RUN_ERROR;
    }

    int res = LeftBoard.start_stream(buffer_size, streamer_params);
    if (res == (int)BrainFlowExitCodes::STATUS_OK) {
        res = RightBoard.start_stream(buffer_size, streamer_params);
    }
    if (res == (int)BrainFlowExitCodes::STATUS_OK) {
        is_streaming = true;
    }
    return res;
}

int AAVAAEar::stop_stream ()
{
    int res = LeftBoard.stop_stream();
    if (res == (int)BrainFlowExitCodes::STATUS_OK) {
        res = RightBoard.stop_stream();
    }
    if (res == (int)BrainFlowExitCodes::STATUS_OK) {
        is_streaming = false;
    }
    return res;
}

int AAVAAEar::release_session ()
{
    int res = (int)BrainFlowExitCodes::STATUS_OK;
    if (initialized)
    {
        res = LeftBoard.release_session();
        if (res == (int)BrainFlowExitCodes::STATUS_OK) {
            res = RightBoard.release_session();
        }
        if (res == (int)BrainFlowExitCodes::STATUS_OK) {
            initialized = false;
        }
    }
    return res;
}

int AAVAAEar::config_board (std::string config, std::string &response)
{
    safe_logger (spdlog::level::trace, "config requested: {}", config);
    if (!initialized)
    {
        return (int)BrainFlowExitCodes::BOARD_NOT_CREATED_ERROR;
    }
    if (config.empty ())
    {
        return (int)BrainFlowExitCodes::INVALID_ARGUMENTS_ERROR;
    }

    int res = (int)BrainFlowExitCodes::STATUS_OK;

    res = send_command (config);
    if ((config[0] == 'w') || ((config[0] == '~')))
    {
#ifdef _WIN32
        Sleep (250);
#else
        usleep (250000);
#endif
        response = device_status;
        device_status = "";
    }
    return res;
}

int AAVAAEar::send_command (std::string config)
{
    if (!initialized)
    {
        return (int)BrainFlowExitCodes::BOARD_NOT_CREATED_ERROR;
    }
    if (config.empty ())
    {
        return (int)BrainFlowExitCodes::INVALID_ARGUMENTS_ERROR;
    }
    int res = LeftBoard.send_command(config);
    if (res == (int)BrainFlowExitCodes::STATUS_OK) {
        res = RightBoard.send_command(config);
    }
    return res;
}

void AAVAAEar::read_data (
    simpleble_uuid_t service, simpleble_uuid_t characteristic, uint8_t *data, size_t size)
{
    safe_logger (spdlog::level::trace, "received {} number of bytes", size);

    if (!is_streaming)
    {
        safe_logger (spdlog::level::trace, "received a string with start byte {} {} {} {} {}",
            data[0], data[1], data[2], data[3], data[4]);
        if ((data[0] == 64) || (data[1] == 64))
        { // when the first or second byte is @
            char markerChar = 'S';
            std::string temp (reinterpret_cast<const char *> (data), size);
            if (data[0] == 64) {
                device_status = temp;
            } else {
                device_status = temp.substr (1);
            }
            if (device_status[7] != markerChar) {
                device_version = extractBoardVersion(device_status);
            }
            safe_logger (spdlog::level::trace, "received a status string {}, version {}", device_status, device_version);
        }
        return;
    }

    int num_rows = board_descr["default"]["num_rows"];

    std::vector<int> eeg_channels = board_descr["default"]["eeg_channels"];

    if (size == 244)
    {
        if (device_version == 3) {
            data += 3;
            size -= 3;
        }
        Incoming_BLE_Data_Buffer.insert (Incoming_BLE_Data_Buffer.end (), data, data + size);
    }
    else if (size > 1)
    {
        if (device_version == 3) {
            ++data;
            --size;
        }
        Incoming_BLE_Data_Buffer.insert (Incoming_BLE_Data_Buffer.end (), data, data + size);
    }

    while (Incoming_BLE_Data_Buffer.size () >= static_cast<size_t> (SIZE_OF_DATA_FRAME))
    {
        if (Incoming_BLE_Data_Buffer[0] != START_BYTE)
        {
            Incoming_BLE_Data_Buffer.pop_front (); // discard data, we have half a frame here
            continue;
        }

        uint8_t *data_frame = new uint8_t[SIZE_OF_DATA_FRAME];
        for (int i = 0; i < SIZE_OF_DATA_FRAME; ++i)
        {
            data_frame[i] = Incoming_BLE_Data_Buffer.front ();
            Incoming_BLE_Data_Buffer.pop_front ();
        }

        if (data_frame[SIZE_OF_DATA_FRAME - 1] != END_BYTE)
        {
            safe_logger (
                spdlog::level::warn, "Wrong End Byte: {}", data_frame[SIZE_OF_DATA_FRAME - 1]);
            continue;
        }

        double *package = new double[num_rows];
        for (int i = 0; i < num_rows; i++)
        {
            package[i] = 0.0;
        }

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

        package[board_descr["default"]["rotation_channels"][0].get<int> ()] =
            IMU_SCALE * cast_16bit_to_int32 (data_frame + 34);
        package[board_descr["default"]["rotation_channels"][1].get<int> ()] =
            IMU_SCALE * cast_16bit_to_int32 (data_frame + 36);
        package[board_descr["default"]["rotation_channels"][2].get<int> ()] =
            IMU_SCALE * cast_16bit_to_int32 (data_frame + 38);

        // battery byte
        package[board_descr["default"]["battery_channel"].get<int> ()] = (double)data_frame[40];

        if (SIZE_OF_DATA_FRAME > 47)
        {
            // gesture byte
            package[board_descr["default"]["other_channels"][0].get<int> ()] =
                (double)data_frame[46];

            // gesture info 1
            package[board_descr["default"]["other_channels"][1].get<int> ()] =
                DETECTION_SCALE * cast_16bit_to_int32 (data_frame + 47);

            // gesture info 2
            package[board_descr["default"]["other_channels"][2].get<int> ()] =
                DETECTION_SCALE * cast_16bit_to_int32 (data_frame + 49);
        }

        // imu status byte
        package[board_descr["default"]["other_channels"][3].get<int> ()] = (double)data_frame[41];

        // timestamp
        try
        {
            if (device_version == 3) {
                package[board_descr["default"]["other_channels"][4].get<int> ()] =
                    *reinterpret_cast<uint32_t *> (data_frame + 42) * TIMESTAMP_SCALE_V3;
            } else {
                package[board_descr["default"]["other_channels"][4].get<int> ()] =
                    *reinterpret_cast<uint32_t *> (data_frame + 42) * TIMESTAMP_SCALE_V4;
            }
        }
        catch (const std::exception &e)
        {
            std::string error_message = std::string ("Exception occurred: ") + e.what ();
            safe_logger (spdlog::level::warn, error_message);
        }

        package[board_descr["default"]["timestamp_channel"].get<int> ()] = get_timestamp ();

        push_package (package);
        delete[] package;
        delete[] data_frame;
    }
}
