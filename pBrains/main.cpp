
#include <string>
#include <list>
#include "pBrains.h"
#include "MBUtils.h"

using namespace std;

int main(int argc, char *argv[])
{
  string mission_file;
  string run_command = argv[0];

  for(int i=1; i<argc; i++) {
    string argi = argv[i];
    if(strEnds(argi, ".moos") || strEnds(argi, ".moos++"))
      mission_file = argv[i];
    else if(i==2)
      run_command = argi;
  }

  pBrains App;

  App.Run(run_command.c_str(), mission_file.c_str());

  return(0);
}

