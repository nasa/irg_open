#include <stdio.h>
#include <stdlib.h>

#include "../irg_rqt_tools/PanTilt.h"
#include "../irg_rqt_tools/ImageTrigger.h"

using namespace irg_rqt_tools;

int main(int argc, char** argv)
{
  fprintf(stderr, "Hello World\n");
  
  PanTilt* panTilt = new PanTilt();
  
  ImageTrigger* imageTrigger = new ImageTrigger();
  
  exit(0);
}
