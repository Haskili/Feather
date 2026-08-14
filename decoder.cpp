#include <iostream>
#include <string>
#include <cstring>
#include <cmath>
#include <format>
#include <array>
#include <algorithm>
#include <ctime>
#include <cctype>
#include <unordered_map>
#include <fstream>
#include <optional>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

struct AVRPacket
{
    uint8_t downlinkFormat;
    uint8_t capability;
    uint32_t identifier;
    uint8_t typeCode;
    uint64_t payload;
    uint32_t CRC;
};

struct PositionFrame {
    uint32_t latitude;
    uint32_t longitude;
    uint8_t CPR;
    time_t timestamp;
};

struct Identification {
    std::string callsign;
    uint8_t category;
};

struct Velocity {
    double velocity;
    double heading;
};

struct Position {
    std::optional<double> latitude;
    std::optional<double> longitude;
    uint32_t altitude;
    std::optional<PositionFrame> LatestOdd;
    std::optional<PositionFrame> LatestEven;
};

struct OperationalStatus {
    bool airborne;
    uint8_t version;
    uint16_t capacity;
    std::optional<uint8_t> accuracy;
};

struct Aircraft {
    uint32_t identifier;
    std::optional<Identification> identification;
    std::optional<Position> position;
    std::optional<Velocity> velocity;
    std::optional<OperationalStatus> status;
};

constexpr std::array<const std::string_view, 18> aircraftCategoryEnumeration {
    "No Category Information Available",
    "Reserved",
    "Surface emergency vehicle",
    "Surface service vehicle",
    "Ground obstruction",
    "Glider / Sailplane",
    "Lighter-than-air",
    "Parachutist / Skydiver",
    "Ultralight / Hang-glider / Paraglider",
    "Unmanned Aerial Vehicle (UAV)",
    "Space Vehicle / Transatmospheric Vehicle",
    "Light (< 7000kg)",
    "Medium 1 ([7000kg, 34000kg])",
    "Medium 2 ([34000kg, 136000kg])",
    "High Vortex Aircraft",
    "Heavy (> 136000kg)",
    "High Performance (>5g & >400kt)",
    "Rotorcraft"
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

    if (std::abs(latitude) >= 87.0)
        return 1;

    // Perform binary search for appropriate value within 'table'
    std::array<double, 59>::const_iterator it = std::lower_bound(table.begin(), table.end(), std::abs(latitude));
    return (59 - std::distance(table.begin(), it));
}

/*
    getCategory() will get the category information associated
    with a given set of 'typeCode' and 'category' values
*/
uint8_t getCategory(const uint8_t& typeCode, const uint8_t& category) {
    
    // Define table used to return values to caller
    constexpr uint8_t categoryTable[5][8] = {
        {0,  0,  0,  0,  0,  0,  0,  0},
        {0,  1,  1,  1,  1,  1,  1,  1},
        {0,  2,  3,  4,  1,  1,  1,  1},
        {0,  5,  6,  7,  8,  1,  9, 10},
        {0, 11, 12, 13, 14, 15, 16, 17}
    };

    // Check bounds before indexing
    if (typeCode > 4 || category > 7)
        return 0;

    // Return the associated value in the 'categoryTable'
    return categoryTable[typeCode][category];
}

/*
    Callsign frame data is structed as follows,

    - Category (3 bits @ [9, 20]): Category of the aircraft
    - Character A (6 bits @ [X, Y]): First character index
    - Character B (6 bits @ [X, Y]): Second character index
    - Character C (6 bits @ [X, Y]): Third character index
    - Character D (6 bits @ [X, Y]): Fourth character index
    - Character E (6 bits @ [X, Y]): Fifth character index
    - Character F (6 bits @ [X, Y]): Six character index
    - Character G (6 bits @ [X, Y]): Seventh character index
    - Character H (6 bits @ [X, Y]): Eigth character index

    In terms of the callsign characters, the values are indices
    for a lookup table 'characterSet'. To decode the true
    callsign information, use the table as required.
*/
void extractCallsign(const AVRPacket& packet, Aircraft& aircraft) {
    
    // Use 'emplace()' if needed to ensure correctness
    if (!aircraft.identification.has_value())
        aircraft.identification.emplace();

    // Find the 'category' for the 'aircraft'
    uint8_t categoryRaw = static_cast<uint8_t>((packet.payload >> 48) & 0x07);
    uint8_t category = getCategory(packet.typeCode, categoryRaw);

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

    // Finish by updating the 'aircraft' with information found
    aircraft.identification->callsign = callsign;
    aircraft.identification->category = category;
}

/*
    The operational status data is structured as follows,

    - XYZ (X bits @ [X, Y]): ...

    ...
*/
void extractSurfacePosition(const AVRPacket& packet, Aircraft& aircraft) {

    // Use 'emplace()' if needed to ensure correctness
    if (!aircraft.velocity.has_value())
        aircraft.velocity.emplace();
    if (!aircraft.position.has_value())
        aircraft.position.emplace();

    // Extract the non-linear movement data first;
    // The table for (in terms of knots) which is as follows:
    //
    //        [0]: Not Available
    //        [1]: [0, 0.125)
    //     [2, 8]: [0.125, 1) @ 0.125kt steps
    //    [9, 12]: [1, 2) @ 0.25kt steps
    //   [13, 38]: [2, 15) @ 0.5kt steps
    //   [39, 93]: [15, 70) @ 1kt steps
    //  [94, 108]: [70, 100) @ 2kt steps 
    // [109, 123]: [100, 175) @ 5kt steps
    //      [124]: 175+
    // [125, 127]: Reserved
    //
    uint8_t movement = static_cast<uint8_t>((packet.payload >> 44) & 0x7F);

    // If possible, calculate the true 'velocity' of the aircraft
    // using the raw 'movement' value and the table above
    double velocity = 0.0;
    if (movement == 1)
        velocity = 0.0;
    else if (movement >= 2 && movement <= 8)
        velocity = 0.125 + (static_cast<double>(movement) - 2) * 0.125;
    else if (movement >= 9 && movement <= 12)
        velocity = 1.0 + (static_cast<double>(movement) - 9) * 0.25;
    else if (movement >= 13 && movement <= 38)
        velocity = 2.0 + (static_cast<double>(movement) - 13) * 0.5;
    else if (movement >= 39 && movement <= 93)
        velocity = 15.0 + (static_cast<double>(movement) - 39) * 1.0;
    else if (movement >= 94 && movement <= 108)
        velocity = 70.0 + (static_cast<double>(movement) - 94) * 2.0;
    else if (movement >= 109 && movement <= 123)
        velocity = 100.0 + (static_cast<double>(movement) - 109) * 5.0;
    else if (movement == 124)
        velocity = 175.0;
    else
        velocity = -1.0;

    // Extract the ground status indicator and raw track value
    uint8_t groundStatus = static_cast<uint8_t>((packet.payload >> 37) & 0x1);
    uint8_t track = static_cast<uint8_t>((packet.payload >> 36) & 0x7F);

    // Check the ground status indicator, and 
    // calculate the true track 'heading' if possible
    double heading = 0.0;
    if (groundStatus != 0)
        heading = (360.0 * static_cast<double>(track)) / 128.0;

    // Grab the 'CPR' of the frame along 
    // with the 'latitude' and 'longitude'
    uint8_t CPR = static_cast<uint8_t>((packet.payload >> 34) & 0x1);
    uint32_t latitude = static_cast<uint32_t>((packet.payload >> 17) & 0x1FFFF);
    uint32_t longitude = static_cast<uint32_t>((packet.payload >> 0) & 0x1FFFF);
    time_t timestamp = std::time(nullptr);

    // Create shortcut to previous position based on what we
    // need given the 'CPR' value of the latest packet
    const PositionFrame& previousPosition = (CPR == 0)?
        aircraft.position->LatestOdd.value_or(PositionFrame{}) : 
        aircraft.position->LatestEven.value_or(PositionFrame{});

    bool hasMatchingFrame = ((CPR == 0 && aircraft.position->LatestOdd))
                        || (CPR == 1 && aircraft.position->LatestEven);

    // If there's previous position data and it's within 10 seconds
    // of our current frame we can use it to perform global position
    // decoding to find absolute latitude and longitude of the 'aircraft'
    if (hasMatchingFrame && difftime(timestamp, previousPosition.timestamp) < 10) {

        // Define the even and odd latitudes
        // via the 'CPR' values
        double latitudeEven = ((CPR == 0)? latitude : previousPosition.latitude) / 131072.0;
        double longitudeEven = ((CPR == 0)? longitude : previousPosition.longitude) / 131072.0;

        double latitudeOdd = ((CPR == 1)? latitude : previousPosition.latitude) / 131072.0;
        double longitudeOdd = ((CPR == 1)? longitude : previousPosition.longitude) / 131072.0;

        // Get the index for latitude
        int latitudeIndex = std::floor((59 * latitudeEven) - (60 * latitudeOdd) + 0.5);

        // Get even and odd latitudes
        //
        // NOTE: This combines MULTIPLE steps within the instructions,
        //       and utilizes a constant (60.0 & 59.0, 'Nz') known 
        //       specifically for Mode-S communication; 
        //       
        //       Please revise as able when incorporating future types. 
        //
        latitudeEven = (90.0 / 60.0) * ((((latitudeIndex % 60) + 60) % 60) + latitudeEven);
        latitudeOdd = (90.0 / 59.0) * ((((latitudeIndex % 59) + 59) % 59) + latitudeOdd);

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
            
            // Get the zone of the absolute latitude
            int latitudeNL = NL(latitudeAbsolute);

            // Get longitude zone size
            // 
            // NOTE: In the formulas (e.g. Junzi Sun) we might see 
            //       that we calculate two 'n' values, one for 
            //       even & odd frames, and choose which one we want 
            //       afterwards; This is a similar approach but uses
            //       less variables but interrogates 'CPR' value twice
            //
            int n = (CPR == 0)? std::max(latitudeNL, 1) : std::max(latitudeNL - 1, 1);
            
            // Get index for longitude (refered to as 'm', see: Junzi Sun)
            int m = std::floor(longitudeEven*(latitudeNL - 1) - longitudeOdd*latitudeNL + 0.5);

            // Get the absolute value of the longitude
            double longitudeAbsolute = (CPR == 0)? 
                (90.0 / n) * ((((m % n) + n) % n) + longitudeEven) :
                (90.0 / n) * ((((m % n) + n) % n) + longitudeOdd);

            // Normalize to [-180, 180]
            if (longitudeAbsolute >= 180)
                longitudeAbsolute -= 360;

            // Finish by updating the 'aircraft' with information found
            aircraft.position->latitude = latitudeAbsolute;
            aircraft.position->longitude = longitudeAbsolute;
        }
    }

    // Finish by updating the 'aircraft' with information found
    if (movement >= 1 && movement <= 124)
        aircraft.velocity->velocity = velocity;
    if (groundStatus != 0)
        aircraft.velocity->heading = heading;

    if (CPR == 0)
        aircraft.position->LatestEven.emplace(PositionFrame{latitude, longitude, CPR, timestamp});
    else
        aircraft.position->LatestOdd.emplace(PositionFrame{latitude, longitude, CPR, timestamp});
}

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
void extractAirbornePosition(const AVRPacket& packet, Aircraft& aircraft) {

    // Use 'emplace()' if needed to ensure correctness
    if (!aircraft.position.has_value())
        aircraft.position.emplace();

    // Find the 'altitude', rectify the value using
    // the Q-bit if needed, and then set the value
    // for the 'aircraft'
    //
    // NOTE: For TC [9, 18] the altitude is barometric,
    //       and for [20, 22] it's a GNSS height derived
    //       from global positioning satellite measured
    //       in meters, not feet.
    //
    int32_t altitude = static_cast<int32_t>((packet.payload >> 36) & 0xFFF);
    if (packet.typeCode >= 9 && packet.typeCode <= 18) {
        if ((altitude >> 4) & 1)
            altitude = (((altitude >> 5) << 4) | (altitude & 0xF)) * 25 - 1000;

        else {
            // -- TODO --
        }
    }
    else
        altitude = std::lround(altitude * 3.280839895);

    // Grab the 'CPR' of the frame along 
    // with the 'latitude' and 'longitude'
    uint8_t CPR = static_cast<uint8_t>((packet.payload >> 34) & 0x1);
    uint32_t latitude = static_cast<uint32_t>((packet.payload >> 17) & 0x1FFFF);
    uint32_t longitude = static_cast<uint32_t>((packet.payload >> 0) & 0x1FFFF);
    time_t timestamp = std::time(nullptr);

    // Create shortcut to previous position based on what we
    // need given the 'CPR' value of the latest packet
    const PositionFrame& previousPosition = (CPR == 0)?
        aircraft.position->LatestOdd.value_or(PositionFrame{}) : 
        aircraft.position->LatestEven.value_or(PositionFrame{});

    bool hasMatchingFrame = ((CPR == 0 && aircraft.position->LatestOdd))
                        || (CPR == 1 && aircraft.position->LatestEven);

    // If there's previous position data and it's within 10 seconds
    // of our current frame we can use it to perform global position
    // decoding to find absolute latitude and longitude of the 'aircraft'
    if (hasMatchingFrame && difftime(timestamp, previousPosition.timestamp) < 10) {

        // Define the even and odd latitudes
        // via the 'CPR' values
        double latitudeEven = ((CPR == 0)? latitude : previousPosition.latitude) / 131072.0;
        double longitudeEven = ((CPR == 0)? longitude : previousPosition.longitude) / 131072.0;

        double latitudeOdd = ((CPR == 1)? latitude : previousPosition.latitude) / 131072.0;
        double longitudeOdd = ((CPR == 1)? longitude : previousPosition.longitude) / 131072.0;

        // Get the index for latitude (AKA 'j' in formulas)
        int latitudeIndex = std::floor((59 * latitudeEven) - (60 * latitudeOdd) + 0.5);

        // Get even and odd latitudes
        //
        // NOTE: This combines MULTIPLE steps within the instructions,
        //       and utilizes a constant (60.0 & 59.0, 'Nz') known 
        //       specifically for Mode-S communication; 
        //       
        //       Please revise as able when incorporating future types. 
        //
        latitudeEven = (360.0 / 60.0) * ((((latitudeIndex % 60) + 60) % 60) + latitudeEven);
        latitudeOdd = (360.0 / 59.0) * ((((latitudeIndex % 59) + 59) % 59) + latitudeOdd);

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

            // Get the zone of the absolute latitude
            int latitudeNL = NL(latitudeAbsolute);

            // Get longitude zone size
            // 
            // NOTE: In the formulas (e.g. Junzi Sun) we might see 
            //       that we calculate two 'n' values, one for 
            //       even & odd frames, and choose which one we want 
            //       afterwards; This is a similar approach but uses
            //       less variables but interrogates 'CPR' value twice
            //       as opposed to just once and storing both results
            //
            int n = (CPR == 0)? std::max(latitudeNL, 1) : std::max(latitudeNL - 1, 1);
            
            // Get index for longitude (refered to as 'm', see: Junzi Sun)
            int m = std::floor(longitudeEven*(latitudeNL - 1) - longitudeOdd*latitudeNL + 0.5);

            // Get the absolute value of the longitude
            double longitudeAbsolute = (CPR == 0)? 
                (360.0 / n) * ((((m % n) + n) % n) + longitudeEven) :
                (360.0 / n) * ((((m % n) + n) % n) + longitudeOdd);

            // Normalize to [-180, 180]
            if (longitudeAbsolute >= 180)
                longitudeAbsolute -= 360;

            // Finish by updating the 'aircraft' with information found
            aircraft.position->latitude = latitudeAbsolute;
            aircraft.position->longitude = longitudeAbsolute;
        }
    }

    // Else, we weren't able to utilize a previous frame for
    // globally unambiguous decoding;
    // 
    // Attempt to perform locally umabiguous position decoding
    // using just this message and a previously calculated
    // absolute position as a reference point
    else if (aircraft.position->latitude.has_value() && aircraft.position->longitude.has_value()) {
        // -- TODO --
    }

    // Update the latest positional frame for 'aircraft'
    aircraft.position->altitude = altitude;
    if (CPR == 0)
        aircraft.position->LatestEven.emplace(PositionFrame{latitude, longitude, CPR, timestamp});
    else
        aircraft.position->LatestOdd.emplace(PositionFrame{latitude, longitude, CPR, timestamp});
}

/*
    The velocity data is structured as follows for sub-types 1 & 2,

    -                Subtype (3 bits @ [9, 20]): Which speed category it falls under (super vs subsonic)
    -     Vertical Direction (1 bit @ [37, 37]): Y-direction
    -     Vertical Velocity (9 bits @ [38, 46]): Y-velocity
    -    East-West Direction (1 bit @ [14, 14]): X-direction
    -   East-West Velocity (10 bits @ [15, 24]): X-velocity
    -  North-South Direction (1 bit @ [25, 25]): Z-direction
    - North-South Velocity (10 bits @ [26, 35]): Z-velocity

    For each direction that we extract (NSEW & UD),
    we extract the velocity component seperately.
    The direction component tells us what direction
    we're going, and thus whether the velocity is
    negative. For example, if the East-West direction 
    is '1', then the direction is "West" and the 
    velocity is negative.
*/
void extractVelocity(const AVRPacket& packet, Aircraft& aircraft) {

    // Use 'emplace()' if needed to ensure correctness
    if (!aircraft.velocity.has_value())
        aircraft.velocity.emplace();

    // First extract the substype of the velocity data
    // to determine the type of velocity information
    // we're receiving
    uint8_t subtype = static_cast<uint8_t>((packet.payload >> 36) & 0x07);

    // Declare velocity and heading values,
    // which are universal regardless of 'subtype'
    uint16_t velocity = 0;
    double heading = 0.0;

    // If the 'subtype' indicates that we're looking at 
    // the ground speed of an aircraft
    if (subtype == 1 || subtype == 2) {

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
        velocity = static_cast<uint16_t>(std::hypot(EWVR, NSVR));

        // Calculate the track 'heading' of the 'aircraft'
        heading = (std::atan2(static_cast<double>(EWVR), static_cast<double>(NSVR)) * 180.0) / M_PI;

        // Rectify 'heading' if it's below 0.0
        heading += (heading >= 0.0? 0.0 : 360.0);
    }

    // Else, if we're being given a velocity packet where
    // the ground speed of the aircraft is not known
    else if (subtype == 3 || subtype == 4) {
        
        // Get the status of magnetic heading availability
        uint8_t headingStatus = static_cast<uint8_t>((packet.payload >> 42) & 0x1);

        // Get the magnetic heading (might not be available)
        uint16_t headingRaw = static_cast<uint16_t>((packet.payload >> 32) & 0x000003FF);
        heading = (headingStatus == 0)? 0 : static_cast<double>(headingRaw) * 360.0 / 1024.0;

        // Get the airspeed type (0: indicated, 1: true)
        uint8_t airspeedType = static_cast<uint8_t>((packet.payload >> 31) & 0x1);

        // Get the airspeed
        uint16_t airspeedRaw = static_cast<uint16_t>((packet.payload >> 21) & 0x000003FF);
        velocity = (subtype == 3)? airspeedRaw - 1 : 4 * (airspeedRaw - 1);
    }

    // Finish by updating the 'aircraft' with information found
    aircraft.velocity->velocity = velocity;
    aircraft.velocity->heading = heading;
}

/*
    The operational status data is structured as follows,

    - XYZ (X bits @ [X, Y]): ...

    ...
*/
void extractOperationalStatus(const AVRPacket& packet, Aircraft& aircraft) {

    // Use 'emplace()' if needed to ensure correctness
    if (!aircraft.status.has_value())
        aircraft.status.emplace();

    // Extract info common between both version 1 & 2
    uint8_t subtype = static_cast<uint8_t>((packet.payload >> 48) & 0x07);
    bool airborne = (subtype == 0);

    uint16_t capacityClass = 0;
    if (subtype == 0)
        capacityClass = static_cast<uint16_t>((packet.payload >> 32) & 0x3FFF);
    else if (subtype == 1)
        capacityClass = static_cast<uint16_t>((packet.payload >> 32) & 0xFFFF);

    // Extract the ADSB 'version' being used, and subsequent fields
    // based on said 'version';
    //
    // From version 1 -> 2 tere are several renamed fields and a SIL
    // "suppliment bit" is added at position 87. See for more details:
    // https://mode-s.org/1090mhz/content/ads-b/6-operation-status.html
    uint8_t version = static_cast<uint8_t>((packet.payload >> 13) & 0x07);
    uint8_t accuracy = 0;
    if ((version == 1) || (version == 2)) {
        accuracy = static_cast<uint8_t>((packet.payload >> 8) & 0x0F);

        if (version == 1) {
            // -- TODO --
        }
        
        else if (version == 2) {
            // -- TODO --
        }
    }

    // Finish by updating the 'aircraft' with information found
    aircraft.status->airborne = airborne;
    aircraft.status->version = version;
    aircraft.status->capacity = capacityClass;

    if ((version == 1) || (version == 2))
        aircraft.status->accuracy.emplace(accuracy);
}

/*
    handleAVR() will take the 'packet' of information given,
    and attempt to extract useful information from it; This 
    information can be a callsign, surface position, 
    airborne position, velocity, or operational status data.

    Once the information is extracted, it will be stored inside
    the given 'aircraft' structure for further use by the caller.
*/
void handleAVR(const AVRPacket& packet, Aircraft& aircraft) {

    // Handle callsign data
    if (packet.typeCode >= 1 && packet.typeCode <= 4)
        extractCallsign(packet, aircraft);

    // Handle surface position data
    else if (packet.typeCode >= 5 && packet.typeCode <= 8)
        extractSurfacePosition(packet, aircraft);

    // Handle airborne position data
    else if ((packet.typeCode >= 9 && packet.typeCode <= 18) || (packet.typeCode >= 20 && packet.typeCode <= 22))
        extractAirbornePosition(packet, aircraft);

    // Handle velocity data
    else if (packet.typeCode == 19)
        extractVelocity(packet, aircraft);

    // Handle operational status data
    else if (packet.typeCode == 31)
        extractOperationalStatus(packet, aircraft);
}

/*
    breakdownAVR() will take a given 112 bit 'message' in AVR format
    and break it down into it's essential parts,

    - Downlink Format: The format of the 'message' (e.g. 17 for extended squitter)
    -       Capability: The transponder capability (unused)
    -        Type Code: The code dictating the type of information in the 'message'
    -          Payload: The information of the 'message' itself

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
            return std::nullopt;
    }

    // Extract the stated CRC value for the 'message'
    uint32_t statedCRC = static_cast<uint32_t>(data & 0xFFFFFF);

    // Compute our own value for the CRC of the 'message'
    // by processing the first 88 bits 
    // (division by shifting & XOR'ing)
    //
    // NOTE: This might be wrong, my original idea was
    //       that we just need to recreate the steps that
    //       they needed to use create the CRC and compare
    //       the values; Apparently the TRUE version of this
    //       computes the value across the entire 112 bits
    //       and checks that the remainder is 0.
    //
    // NOTE: This needs to be compartmentalized anyways,
    //       so I'll leave the revision until the next version...
    //
    uint32_t computedCRC = 0;
    for (int i = 111; i >= 24; --i) {

        // Shift 'data' to the right 'i' spaces
        // and get the bit on end
        bool bitData = (data >> i) & 1;

        // Shift the current computed CRC value
        // 23 spaces and get the bit on the end
        bool bitCRC = (computedCRC >> 23) & 1;

        // Shift the computed CRC value left once
        computedCRC <<= 1;

        // Throw away upper (left) 8 bits by masking 
        // for 24 bits since we're working with a 'uint32_t'
        computedCRC &= 0xFFFFFF;

        // If XOR'ing results in a 1, then we need to
        // XOR our CRC value with the polynomial
        if (bitData ^ bitCRC)
            computedCRC ^= 0xFFF409;
    }

    // If the stated CRC value isn't the same
    // as what we computed, then the frame might
    // be corrupted and should be tossed
    if (statedCRC != computedCRC) 
        return std::nullopt;

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
    return AVRPacket{downlinkFormat, capability, identifier, typeCode, payload, statedCRC};
}

/*
    handleMessage() is designed as an intermediatary that handles the
    majority of the work, e.g. decoding and state management.

    Currently, it only handles extended squitter messages (DF17),
    everything else will get thrown out as empty 'data' packets
    in a return status of '2'.
*/
int handleMessage(std::unordered_map<uint32_t, Aircraft>& aircraft, const std::string& message) {

    // Check input 'message' structure before startup;
    // If not conforming to expectations then report 
    // status to the caller
    if (message.size() != 30 || message.front() != '*' || message.back() != ';')
        return 1;

    // Break down the 'message' and extract the raw 'data'
    // (e.g. nothing is read or analyzed yet)
    std::optional<AVRPacket> data = breakdownAVR(message.substr(1, 28));

    // If no 'data' could be extracted from 'message'
    // return status to caller
    if (!data.has_value())
        return 2;

    // Else, handle the contents of 'data' as required
    //
    // Being by assigning a shortcut to the decoded value 
    // of the extracted 'data'
    const AVRPacket& packet = data.value();

    // Try to emplace the aircraft in our state mapping,
    // which is keyed off of the 24-bit identifier of aircraft
    //
    // NOTE: Using 'auto' keyword here pains me, 
    //       but the alternative is *extremely* 
    //       visually cumbersome
    //
    auto& state = aircraft.try_emplace(packet.identifier, Aircraft{packet.identifier}).first->second;

    // Handle the contents of the AVR packet 'data' and update
    // the state of aircraft in question
    handleAVR(packet, state);

    // Print out our updated 'state' information
    const auto& identification = state.identification;
    const auto& position = state.position;
    const auto& velocity = state.velocity;
    std::cout
        << std::format("{:06X}", state.identifier) << ": '"
        << (identification? identification->callsign : "Unknown") << "' ("
        << ((position && position->latitude)? std::to_string(position->latitude.value()) : "?") << ", "
        << ((position && position->longitude)? std::to_string(position->longitude.value()) : "?") << ") "
        << (velocity? std::to_string(velocity->velocity) : "?") << "kt "
        << (velocity? std::to_string(velocity->heading) : "?") << "° "
        << (position? std::to_string(position->altitude) : "?") << "ft\n";

    // Send out update to database if needed
    // -- TODO --

    // Successful handling of 'message',
    // return status to caller
    return 0;
}

/*
    startListener() will take a given 'address' & 'port' and attempt
    to listen for AVR messages. If it finds a message it will pass
    it to a handler that manages everything else (e.g. acting on
    the data and updating a aircraft state-mapping).
*/
int startListener(const char* address, const int port) {

    // Create a 'connection' to the dump1090 server
    int connection = socket(AF_INET, SOCK_STREAM, 0);
    if (connection < 0) {
        std::cerr << "Failed to create socket\n";
        return 1;
    }

    // Convert values ('port' & 'address')
    sockaddr_in server{};
    server.sin_family = AF_INET;
    server.sin_port = htons(port);
    if (inet_pton(AF_INET, address, &server.sin_addr) <= 0) {
        std::cerr << "Invalid address\n";
        close(connection);
        return 2;
    }

    // Connect to the dump1090 server
    if (connect(connection, (sockaddr*)&server, sizeof(server)) < 0) {
        std::cerr << "Connection failed\n";
        close(connection);
        return 3;
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

            // Call the intermediary function to handle breaking down
            // the 'message' into readable parts, analyzing the parts,
            // and updating the state-mapping 'aircraft'
            int status = handleMessage(aircraft, message);

            // Handle errors reported as part of 'status'
            // -- TODO --

            // Prepare 'incoming' for next iteration
            // by erasing up to the newline character
            incoming.erase(0, pos + 1);
        }
    }

    // Close 'connection' to the host server
    close(connection);
    return 0;
}

/*
    startFileReader() will take a given input 'filename',
    and attempt to read it and feed the data to a handler.  
    
    The handler is what extracts information from the AVR, 
    in turn gives that information to another handler that 
    analyzes it, and finally manages updating the state-mapping 
    of all the aircraft data.
*/
int startFileReader(const char* filename) {
    
    // Define a mapping to handle aircraft states
    std::unordered_map<uint32_t, Aircraft> aircraft;

    // Define out line-feed string
    std::string line;

    // Attempt to open the file and read from it
    std::ifstream inputFile(filename);
    if (inputFile.is_open()) {

        // For each 'line' of the input file
        while (getline(inputFile, line)) {

            // Strip newline and return characters as needed
            line.erase(std::remove(line.begin(), line.end(), '\n'), line.cend());
            line.erase(std::remove(line.begin(), line.end(), '\r'), line.cend());

            // Pass the data off to a handler that
            // will manage extracting the data and
            // updating our state-mapping of 'aircraft'
            int status = handleMessage(aircraft, line);

            // Handle errors reported as part of 'status'
            // -- TODO --
        }

        // All data of 'inputFile' has been read,
        // and the file itself needs to be closed
        inputFile.close();
    }

    // File wasn't able to be opened,
    // return status to caller
    else {
        std::cerr << "Unable to open file: '" << filename << "'\n";
        return 1;
    }

    // File processing sucessful,
    // return status to caller
    return 0;
}