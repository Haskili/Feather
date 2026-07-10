#include <iostream>
#include <string>
#include <cstring>
#include <cmath>
#include <format>
#include <unordered_map>
#include <optional>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

constexpr char* ADDRESS = "127.0.0.1";
constexpr int PORT = 30002;

struct Position {
    double latitude;
    double longitude;
    int CPR;
    time_t timestamp;
};

struct Operation {
    // ...
};

struct Aircraft {
    uint32_t identifier;
    std::optional<std::string> callsign;
    std::optional<double> latitude;
    std::optional<double> longitude;
    std::optional<double> velocity;
    std::optional<double> heading;
    std::optional<uint16_t> altitude;
    std::optional<uint8_t> version;

    std::optional<Position> latestPosition;
};

struct AVRPacket
{
    uint8_t downlinkFormat;
    uint8_t capability;
    uint32_t identifier;
    uint8_t typeCode;
    uint64_t payload;
};

/*
    NL() gives the NL zone of a given 'latitude' value
*/
int NL(const double& latitude) {
    static constexpr std::array<double, 59> table = {
        10.47047130, 14.82817437, 18.18626357, 21.02939493,
        23.54504487, 25.82924707, 27.93898710, 29.91135686,
        31.77209708, 33.53993436, 35.22899598, 36.85025108,
        38.41241892, 39.92256684, 41.38651832, 42.80914012,
        44.19454951, 45.54626723, 46.86733252, 48.16039128,
        49.42776439, 50.67150166, 51.89342469, 53.09516153,
        54.27817472, 55.44378444, 56.59318756, 57.72747354,
        58.84763776, 59.95459277, 61.04917774, 62.13216659,
        63.20427479, 64.26616523, 65.31845310, 66.36171008,
        67.39646774, 68.42322022, 69.44242631, 70.45451075,
        71.45986473, 72.45884545, 73.45177442, 74.43893416,
        75.42056257, 76.39684391, 77.36789461, 78.33374083,
        79.29428225, 80.24923213, 81.19801349, 82.13956981,
        83.07199445, 83.99173563, 84.89166191, 85.75541621,
        86.53536998, 87.00000000
    };

    // Perform binary search for appropriate value within 'table'
    std::array<double, 59>::const_iterator it = std::lower_bound(table.begin(), table.end(), latitude);
    return (59 - std::distance(table.begin(), it));
}

/*
    handleAVR() will take the 'packet' of information given,
    and attempt to extract useful information from it; This 
    information can be callsign, surface position, 
    airborne position, velocity, or operational status data.

    Once the information is extracted, it will be stored inside
    the given 'aircraft' structure for further use by the caller.
*/
void handleAVR(const AVRPacket& packet, struct Aircraft& aircraft) {

    // Handle callsign data
    if (packet.typeCode >= 1 && packet.typeCode <= 4) {

        /*
            Callsign frame data is structed as follows,

            - Category (3 bits @ [9, 20]): ...
            - Character A (6 bits @ [X, Y]): ...
            - Character B (6 bits @ [X, Y]): ...
            - Character C (6 bits @ [X, Y]): ...
            - Character D (6 bits @ [X, Y]): ...
            - Character E (6 bits @ [X, Y]): ...
            - Character F (6 bits @ [X, Y]): ...
            - Character G (6 bits @ [X, Y]): ...
            - Character H (6 bits @ [X, Y]): ...

            In terms callsign characters, the values are indices
            for a lookup table 'characterSet'. To decode the true
            callsign information, use the table.
        */
        
        // Find the 'category' for the 'aircraft'
        uint8_t category = static_cast<uint8_t>((packet.payload >> 48) & 0x07);;

        // Define the character mapping for callsign decoding
        static const char characterSet[] = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####_###############0123456789######";
        
        // For each of the 8 6-bit characters of the callsign 
        std::string callsign;
        for (int i = 0; i < 8; i++){

            // Shift the payload over the 
            // correct amount for this iteration 
            uint8_t character = (packet.payload >> (42 - (i * 6))) & 0x3F;
            
            // If the 'character' is valid, 
            // add it to the 'callsign' data
            if (characterSet[character] != '#' && characterSet[character] != '_')
                callsign.push_back(characterSet[character]);
        }

        // Finish by assigning the 'callsign' data
        // to the given 'aircraft'
        aircraft.callsign = callsign;
    }

    // Handle surface position data
    // -- TODO --
    else if (packet.typeCode >= 5 && packet.typeCode <= 8) {
        //
    }

    // Handle airborne position data
    else if ((packet.typeCode >= 9 && packet.typeCode <= 18) || (packet.typeCode >= 20 && packet.typeCode <= 22)) {
        
        /*
            The airborne position data is structured as follows,

            -   Altitude (12 bits @ [9, 20]): Aircraft altitude (barometric or GNSS)
            -         CPR (1 bit @ [22, 22]): Determines whether frame is even or odd
            -  Latitude (17 bits @ [23, 39]): Aircraft latitude (even / odd)
            - Longitude (17 bits @ [40, 56]): Aircraft longitude (even / odd)

            The aircraft positional data is an altitude, and a set of coordinates. 
            Based on the "CPR" of the frame, it's either an even or an odd frame.
            When you can combine both even and odd frames for position you get an 
            accurate set of coordinates for where the aircraft is.
        */

        // Find the 'altitude', rectify the value using
        // the Q-bit if needed, and then set the value
        // for the 'aircraft'
        //
        // NOTE: For TC [9, 18] the altitude is barometric,
        //       and for [20, 22] it's a GNSS height derived
        //       from global positioning satellite measured
        //       in meters, not feet.
        //
        uint16_t altitude = static_cast<uint16_t>((packet.payload >> 36) & 0xFFF);
        if (packet.typeCode >= 9 && packet.typeCode <= 18) {
            if ((altitude >> 4) & 1)
                altitude = (((altitude >> 5) << 4) | (altitude & 0xF)) * 25 - 1000;
        }
        else
            altitude *= 3.280839895;

        aircraft.altitude = altitude;

        // Grab the 'CPR' of the frame along 
        // with the 'latitude' and 'longitude'
        uint8_t CPR = static_cast<uint8_t>((packet.payload >> 34) & 0x1);
        uint32_t latitude = static_cast<uint32_t>((packet.payload >> 17) & 0x1FFFF);
        uint32_t longitude = static_cast<uint32_t>((packet.payload >> 0) & 0x1FFFF);
        time_t timestamp = std::time(nullptr);

        // Check if there's data in 'aircraft.latestPosition',
        // if so then get the shortcut to the data to save on decoding
        const Position& alternatePosition = (aircraft.latestPosition != std::nullopt)? 
            aircraft.latestPosition.value() : Position{0, 0, 0, 0};       

        // and if so if we can use that data along with what
        // we just extract to find absolute latitude and longitude
        // of the 'aircraft'
        if (aircraft.latestPosition != std::nullopt 
            && alternatePosition.CPR != CPR
            && difftime(timestamp, alternatePosition.timestamp) < 10) {
            
            // Define the even and odd latitudes
            // via the 'CPR' values
            double latitudeEven = ((CPR == 0)? latitude : alternatePosition.latitude) / 131072.0;
            double longitudeEven = ((CPR == 0)? longitude : alternatePosition.longitude) / 131072.0;

            double latitudeOdd = ((CPR == 1)? latitude : alternatePosition.latitude) / 131072.0;
            double longitudeOdd = ((CPR == 1)? longitude : alternatePosition.longitude) / 131072.0;

            // Get the index for latitude
            int latitudeIndex = std::floor((59 * latitudeEven) - (60 * latitudeOdd) + 0.5);

            // Get latitude spacing
            latitudeEven = (360.0 / 60.0) * ((latitudeIndex % 60) + latitudeEven);
            latitudeOdd = (360.0 / 59.0) * ((latitudeIndex % 59) + latitudeOdd);

            // Normalize latitude values to [-90, 90]
            if (latitudeEven >= 270)
                latitudeEven -= 360;

            if (latitudeOdd >= 270)
                latitudeOdd -= 360;

            // Ensure both latitudes exist within the same NL zone
            // before continuing calculation
            if (NL(latitudeEven) == NL(latitudeOdd)) {

                // Define the absolute latitude of the 'aircraft'
                // as the newest valid 'latitude' value
                double latitudeAbsolute = (CPR == 0)? latitudeEven : latitudeOdd;

                // Get the absolute value of the longitude
                int latitudeNL = NL(latitudeAbsolute);
                int ni = (CPR == 0)? std::max(latitudeNL, 1) : std::max(latitudeNL - 1, 1);
                int m = std::floor(longitudeEven*(latitudeNL - 1) - longitudeOdd*latitudeNL + 0.5);
                double longitudeAbsolute = (CPR == 0)? 
                    (360.0 / ni) * ((m % ni) + longitudeEven) :
                    (360.0 / ni) * ((m % ni) + longitudeOdd);

                // Normalize to [-180, 180]
                if (longitudeAbsolute >= 180)
                    longitudeAbsolute -= 360;

                // Update the state of 'aircraft' with
                // the newly calculated values
                aircraft.latitude = latitudeAbsolute;
                aircraft.longitude = longitudeAbsolute;
            }
        }

        // Update the latest positional frame for 'aircraft'
        aircraft.latestPosition = Position{latitude, longitude, CPR, timestamp};
    }

    // Handle velocity data
    else if (packet.typeCode == 19) {

        /*
            The velocity data is structured as follows,

            -                Subtype (3 bits @ [9, 20]): ...
            -     Vertical Direction (1 bit @ [37, 37]): ...
            -     Vertical Velocity (9 bits @ [38, 46]): ...
            -    East-West Direction (1 bit @ [14, 14]): ...
            -   East-West Velocity (10 bits @ [15, 24]): ...
            -  North-South Direction (1 bit @ [25, 25]): ...
            - North-South Velocity (10 bits @ [26, 35]): ...

            ...
        */

        // First extract the substype of the velocity data
        // to determine the type of velocity information
        // we're receiving
        uint8_t subtype = static_cast<uint8_t>((packet.payload >> 36) & 0x07);

        // Extract the directional and velocity values
        // from the payload
        uint8_t EWD = static_cast<uint8_t>((packet.payload >> 42) & 0x1);
        uint16_t EWV = static_cast<uint16_t>((packet.payload>> 32) & 0x3FF);
        int EWVR = (EWD == 0)? EWV : -1 * static_cast<int>(EWV);

        uint8_t NSD = static_cast<uint8_t>((packet.payload >> 31) & 0x1);
        uint16_t NSV = static_cast<uint16_t>((packet.payload >> 21) & 0x3FF);
        int NSVR = (NSD == 0)? NSV : -1 * static_cast<int>(NSV);

        uint8_t UDD = static_cast<uint8_t>((packet.payload >> 19) & 0x1);
        uint16_t UDV = static_cast<uint16_t>((packet.payload >> 10) & 0x1FF);
        int UDVR = (UDD == 0)? UDV : -1 * static_cast<int>(UDV);

        // Calculate the true ground speed of the 'aircraft'
        // assuming we're not looking at something supersonic
        uint16_t velocity = static_cast<uint16_t>(std::sqrt(pow(EWVR, 2) + pow(NSVR, 2)));

        // Calculate the track 'heading' of the 'aircraft'
        double heading = (std::atan2(static_cast<double>(EWVR), static_cast<double>(NSVR)) * 180.0) / M_PI;

        // Assign 'velocity' and 'heading' data to the 'aircraft'
        aircraft.velocity = velocity;
        aircraft.heading = heading;
    }

    // Handle operational status data
    // -- TODO --
    else if (packet.typeCode == 31) {
        //
    }
}

/*
    breakdownAVR() will take a given 112 bit 'message' in AVR format
    and break it down into it's essential parts,

    - Downlink Format: The format of the 'message' (e.g. 17 for extended squitter)
    -      Capability: The transponder capability (unused)
    -       Type Code: The code dictating the type of information in the 'message'
    -         Payload: The information of the 'message' itself

    Currently, it only handles extended squitter messages (DF17);
    Meaning everything else will get thrown out.
*/
std::optional<AVRPacket> breakdownAVR(const std::string& message) {

    // Initial check on input 'message' length before conversion
    if (message.length() != 28)
        return std::nullopt;

    // Get the 'data' from the hex string formatted 'message'
    unsigned __int128 data = 0;
    for (char character : message) {

        // Shift 'data' over by 4 bits so we can
        // insert information from the next 'character'
        data <<= 4;

        // Parse the current 'character' and digest
        // the information into 'data' via a conditional
        // bitwise OR operation
        character = std::toupper(static_cast<unsigned char>(character));
        if (character >= '0' && character <= '9')
            data |= character - '0';
        else if (character >= 'A' && character <= 'F')
            data |= character - 'A' + 10;
        else
            throw std::runtime_error("Invalid hex digit");
    }

    // Mask 'data' to check the downlink format of 'data'
    // to ensure we're only processing the correct types
    // of messages
    uint8_t downlinkFormat = static_cast<uint8_t>((data >> 107) & 0x1F);
    if (downlinkFormat != 17)
        return std::nullopt;

    // Get the following pieces of information about the given 'message'
    // via shifting and masking for the appropriate amount of bits
    // 
    // - Capability  (3 bits): Transponder capability
    // - Identifier (24 bits): ICAO aircraft address sending the 'message'
    // - Type Code   (5 bits): The type of information in the 'message'
    // - Payload    (56 bits): Information of the 'message'
    //
    uint8_t capability = static_cast<uint8_t>((data >> 104) & 0x07);
    uint32_t identifier = static_cast<uint32_t>((data >> 80) & 0xFFFFFF);
    uint8_t typeCode = static_cast<uint8_t>((data >> 75) & 0x1F);
    uint64_t payload = static_cast<uint64_t>((data >> 24) & 0xFFFFFFFFFFFFFFULL);

    // std::cout << std::hex << std::uppercase << identifier << std::dec << std::endl;
    // std::cout << std::format("{:06X}", identifier) << std::endl;

    // Pack the data and return it
    return AVRPacket{downlinkFormat, capability, identifier, typeCode, payload};
}

int initListener() {

    // Create a 'connection' to the dump1090 server
    int connection = socket(AF_INET, SOCK_STREAM, 0);
    if (connection < 0) {
        std::cerr << "Failed to create socket\n";
        return 1;
    }

    // Convert values (port & address)
    sockaddr_in server{};
    server.sin_family = AF_INET;
    server.sin_port = htons(PORT);
    if (inet_pton(AF_INET, ADDRESS, &server.sin_addr) <= 0) {
        std::cerr << "Invalid address\n";
        close(connection);
        return 1;
    }

    // Connect to the dump1090 server
    if (connect(connection, (sockaddr*)&server, sizeof(server)) < 0) {
        std::cerr << "Connection failed\n";
        close(connection);
        return 1;
    }

    // Define a reusable 'buffer' to handle data from the server
    char buffer[4096];
    
    // Define string to handle data from the 'buffer'
    std::string incoming;

    // Define a mapping to handle aircraft states
    std::unordered_map<uint32_t, Aircraft> aircraft;

    // Primary data reception loop
    while (true) {

        // Receive data;
        // If none found within 'buffer', exit for closure
        ssize_t bytes = recv(connection, buffer, sizeof(buffer), 0);
        if (bytes <= 0) {
            std::cout << "Connection closed.\n";
            break;
        }

        // Transfer the 'buffer' over
        incoming.append(buffer, bytes);
        
        // Process the entirety of the data within 'incoming'
        // by checking for newlines and processing everything
        // that came before that
        size_t pos;
        while ((pos = incoming.find('\n')) != std::string::npos) {

            // Get the substring of 'incoming' to get everything
            // up to that newline character
            std::string message = incoming.substr(0, pos);

            // Format the 'message' as needed for return-character
            if (!message.empty() && message.back() == '\r')
                message.pop_back();

            // Disregard any 'message' that isn't 112-bit format
            if (message.length() == 30) {

                // Break down the current 'message' data into
                // 'data' that we can further work with
                std::optional<AVRPacket> data = breakdownAVR(message.substr(1, 28));
                
                // If we got back data from breaking down 'message'
                // then handle the contents as required
                if (data != std::nullopt) {

                    // Assign a shortcut to the decoded value of 'data'
                    const AVRPacket& packet = data.value();
                    
                    // If the ICAO of the aircraft that sent 'message'
                    // isn't in 'aircraft', then this is the first time
                    // we've seen it and we need to enter it into the mapping
                    if (aircraft.find(packet.identifier) == aircraft.end())
                        aircraft.emplace(packet.identifier, Aircraft{packet.identifier});

                    // Handle the contents of the AVR packet 'data' and update
                    // the state of aircraft in question
                    handleAVR(packet, aircraft[packet.identifier]);

                    // Print out our updated 'state' information
                    const auto& state = aircraft[packet.identifier];
                    std::cout
                        << std::format("{:06X}", state.identifier) << ": "
                        << "'" << state.callsign.value_or("Unknown") << "' ("
                        << ((state.latitude != std::nullopt)? std::to_string(*state.latitude)  : "?") << ", "
                        << ((state.longitude != std::nullopt)? std::to_string(*state.longitude) : "?") << ") "
                        << ((state.velocity != std::nullopt)? std::to_string(*state.velocity) : "?") << "kt "
                        << ((state.heading != std::nullopt)? std::to_string(*state.heading)  : "?") << "° "
                        << ((state.altitude != std::nullopt)? std::to_string(*state.altitude) : "?") << "ft\n";
                }
            }

            // Prepare 'incoming' for next iteration
            // by erasing up to the newline character
            incoming.erase(0, pos + 1);
        }
    }

    // Close 'connection' to the host server
    close(connection);
    return 0;
}

int main(int argc, char* argv[]) {
    return initListener();
}