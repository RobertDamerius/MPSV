#include <ApplicationArguments.hpp>


ApplicationArguments::ApplicationArguments(int argc, char** argv){
    // set default values
    console = false;
    help = false;

    // scan all arguments, ignore unknown arguments
    for(int i = 1; i < argc; ++i){
        std::string arg(argv[i]);
        console |= (0 == arg.compare("--console"));
        help |= (0 == arg.compare("--help"));
    }

    // print help if requested
    if(help){
        PrintHelp();
    }
}

void ApplicationArguments::PrintHelp(void){
    Print("\n");
    Print("Syntax: mpsv [--console] [--help]\n");
    Print("\n");
    Print("Options:\n");
    Print("    --console   Print stderr to the console instead of redirecting to a protocol file.\n");
    Print("    --help      Show this help page.\n");
    Print("\n");
}

