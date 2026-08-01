#include "decoder.cpp"

constexpr const char* help = R"(Feather - An ADS-B processing utility
Usage: feather [OPTION] [AUXILLARY-OPTION-INFO]...

Options:
    -h, --help
        Display this help message.

    -l, --listener <ADDRESS> <PORT>
        Connect to a network listener.

    -f, --file <FILE>
        Read ADS-B data from an input file with new-line seperated values.

    -t, --test
        Run the unit tests.

Examples:
    feather -l 127.0.0.1 30002
    feather -f example.in
    feather -t
)";

/*
    handleArguments() is an example of how to parse arguments
    and feed them into the "Feather" library functions
*/
int handleArguments(int argc, char* argv[]) {

    // For each extra argument
    for (int i = 1; i < argc; i++) {
        
        // If we're being asked for the help page
        // -- TODO --
        if (strcmp("-h", argv[i]) == 0 || strcmp("--help", argv[i]) == 0) {
            std::cout << help;
            return 0;
        }

        // ELse-if we're being asked for a listener
        //
        // NOTE: Default should be 127.0.0.1 on port 30002
        //       for standard 'dump1090_rs' configuration
        //
        else if (strcmp("-l", argv[i]) == 0 || strcmp("--listener", argv[i]) == 0) {
            
            // Check to ensure we have at least 2 arguments
            // after this to designate the address and port
            if (i + 2 < argc)
                return startListener(argv[i + 1], std::stoi(argv[i + 2]));
            
            // Else, incorrect number of arguments;
            // Report failure to caller
            std::cerr << "Bad arguments\n";
            return 1;
        }

        // ELse-if we're being asked for file I/O
        else if (strcmp("-f", argv[i]) == 0 || strcmp("--file", argv[i]) == 0) {
            
            // Check to ensure we have at least 1 argument
            // after this to designate the filename
            if (i + 1 < argc)
                return startFileReader(argv[i + 1]);

            // Else, incorrect number of arguments;
            // Report failure to caller
            std::cerr << "Bad arguments\n";
            return 1;
        }

        // ELse-if we're being asked for unit tests
        else if (strcmp("-t", argv[i]) == 0 || strcmp("--test", argv[i]) == 0) {
            
            // Run unit tests
            // -- TODO --

            // Else, incorrect number of arguments;
            // Report failure to caller
            std::cerr << "Bad arguments\n";
            return 1;
        }
    }

    return 1;
}

int main(int argc, char* argv[]) {

    // Check arguments to see what caller wants
    if (argc > 1)
        return handleArguments(argc, argv);

    // By default, listen on the standard address and port for 'dump1090_rs'
    return startListener("127.0.0.1", 30002);
}