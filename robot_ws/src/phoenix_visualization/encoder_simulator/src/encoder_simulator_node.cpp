/*
    * Goal processor node entry point
    * Author : Abdullah Mohiuddin
    * Email : a.mohiuddin@ai-systems.ca
*/
#include <encoder_simulator_node.h>
using namespace std;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "encoder_simulator_node");
  encodersimulator::EncoderSimulatorNode esn(argc, argv);
  esn.run();
  return 0;
}
