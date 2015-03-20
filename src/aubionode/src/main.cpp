// Standard C/C++ libraries
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h> /* for strncpy */

//library for ros
#include <ros/ros.h>

//libraries for the control
#include "std_msgs/String.h"
#include <sstream>

#include "aubio-lib/src/aubio.h"
#include "rtaudio/RtAudio.h"

/*
/	sudo apt-get install expect-dev
*/

ros::Publisher aubio_node;

//(TODO : mettre le tout oriente objet)
aubio_pitch_t* PITCH_OBJECT;
float MIN_AMPLITUDE;
unsigned int WINDOW_SIZE;
unsigned int HOP_SIZE;
unsigned int SAMPLE_RATE;
unsigned int NB_CHANNELS;
char* PITCH_MODE;
char* PITCH_UNIT;
fvec_t* INPUT;
fvec_t* OUTPUT;

void setup()
{
    MIN_AMPLITUDE = 0.25;

    WINDOW_SIZE = 1024;
    HOP_SIZE = WINDOW_SIZE/4;
    SAMPLE_RATE = 44100;
    NB_CHANNELS = 1;

    PITCH_MODE = "default";
    PITCH_UNIT = "midi";

    INPUT = new_fvec(HOP_SIZE);
    OUTPUT = new_fvec(1);

    PITCH_OBJECT = new_aubio_pitch(PITCH_MODE, WINDOW_SIZE, HOP_SIZE, SAMPLE_RATE);
    aubio_pitch_set_unit(PITCH_OBJECT, PITCH_UNIT);
}


void destructor()
{
    del_aubio_pitch(PITCH_OBJECT);
    del_fvec(INPUT);
    del_fvec(OUTPUT);
    aubio_cleanup();
}


void processPitch(float* input, int bufferSize)
{
    float rmsAmplitude  = 0;

    for(int i = 0; i < bufferSize; i++)
    {
        //calculate the root mean square amplitude
        rmsAmplitude += sqrt(input[i]*input[i]);

        INPUT->data[i] = input[i];
    }

    rmsAmplitude /= bufferSize;

    if(rmsAmplitude > MIN_AMPLITUDE)
    {
        aubio_pitch_do(PITCH_OBJECT, INPUT, OUTPUT); // put pitch in variable OUPUT

        smpl_t new_pitch = fvec_get_sample(OUTPUT, 0);

        std::cout << new_pitch << std::endl;
    }
}


int captureOutput(void *outputBuffer, void *inputBuffer, unsigned int bufferSize,
                  double streamTime, RtAudioStreamStatus status, void *userData)
{
    if (status){std::cout << "Stream overflow detected!" << std::endl;}

    float* in = (float*) inputBuffer;

    processPitch(in, bufferSize);

    return 0;
}


void openStream()
{
    RtAudio adc;

    if(adc.getDeviceCount() < 1)
    {
        std::cout << "\nNo audio devices found!\n";
        exit( 0 );
    }

    RtAudio::StreamParameters parameters;
    parameters.deviceId = adc.getDefaultInputDevice();
    parameters.nChannels = NB_CHANNELS;
    parameters.firstChannel = 0;
    unsigned int sampleRate = SAMPLE_RATE;
    unsigned int bufferFrames = HOP_SIZE;

    try
    {
      adc.openStream(NULL, &parameters, RTAUDIO_FLOAT32, sampleRate, &bufferFrames, &captureOutput);
      adc.startStream();
    }
    catch(RtAudioError& e)
    {
      e.printMessage();
      exit( 0 );
    }

    char input;
    std::cout << "\nRecording ... press <enter> to quit.\n";
    std::cin.get( input );

    try
    {
      adc.stopStream();
    }
    catch (RtAudioError& e)
    {
      e.printMessage();
    }

    if(adc.isStreamOpen()){ adc.closeStream(); }
}

int main(int argc, char **argv)
{
    char rosname[100];
    std::string temp_arg;
    ros::init(argc, argv, rosname);
    ros::NodeHandle node;

    setup();

    openStream();

    destructor();

	return 0;
}

// RtAudio method https://www.music.mcgill.ca/~gary/rtaudio/probe.html
void getAudioDevicesInfo()
{
    RtAudio audio;
      // Determine the number of devices available
      unsigned int devices = audio.getDeviceCount();
      // Scan through devices for various capabilities
      RtAudio::DeviceInfo info;
      for ( unsigned int i=0; i<devices; i++ ) {
        info = audio.getDeviceInfo( i );
        if ( info.probed == true ) {
          // Print, for example, the maximum number of output channels for each device
          std::cout << "device = " << i;
          std::cout << ": maximum output channels = " << info.outputChannels << "\n";
        }
      }
}
