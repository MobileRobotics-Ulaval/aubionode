// Standard C/C++ libraries
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <iostream>
#include <string.h> /* for strncpy */

//library for ros
#include <ros/ros.h>

//libraries for the control
#include "std_msgs/String.h"
#include <sstream>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <aubionode/audioConstantsConfig.h>

#define AUBIO_UNSTABLE 1
#include "aubio-lib/src/aubio.h"
#include "rtaudio/RtAudio.h"

/*
/	sudo apt-get install expect-dev
*/

ros::Publisher aubio_node;

//(TODO : mettre le tout oriente objet)
aubio_pitch_t* PITCH_OBJECT;
aubio_onset_t* ONSET_OBJECT;
smpl_t ONSET_THRESHOLD;
unsigned int WINDOW_SIZE;
unsigned int HOP_SIZE;
unsigned int SAMPLE_RATE;
unsigned int NB_CHANNELS;
int SILENCE_THRESHOLD;
char* PITCH_MODE;
char* ONSET_MODE;
char* PITCH_UNIT;
fvec_t* INPUT;
fvec_t* OUTPUT;
fvec_t* ONSET;
fvec_t *note_buffer;
fvec_t *note_buffer2;
uint_t median = 35;
smpl_t curnote = 0.;
smpl_t newnote = 0.;
uint_t isready = 0;
int notes_array[3];
int nbr_notes = 0;
int timeout_sec = 4;
int timeout_start = 0;

/*
/ Fonction de timeout pour la  détection de notes successives
*/
int timeout(int delay, int restart)
{
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);

    if((tm.tm_sec - timeout_start) > delay || restart)
    {
        timeout_start=tm.tm_sec;
        std::cout << "Timeout" << std::endl;
        return 1;
    }
    return 0;
}

void setup()
{
    ONSET_THRESHOLD = -0.005; // default -0.0005
    SILENCE_THRESHOLD = -40; // default -20

    WINDOW_SIZE = 1024; // default 1024
    HOP_SIZE = WINDOW_SIZE/2;
    SAMPLE_RATE = 44100;
    NB_CHANNELS = 1;

    PITCH_MODE = "default"; //default, schmitt, fcomb, mcomb, specacf, yin, yinfft.
    ONSET_MODE = "energy"; //default, energy, hfc, complex, phase, specdiff, kl, mkl, specflux.
    PITCH_UNIT = "midi";

    INPUT = new_fvec(HOP_SIZE);
    OUTPUT = new_fvec(1);
    ONSET = new_fvec(1);

    if(median)
    {
      note_buffer = new_fvec (median);
      note_buffer2 = new_fvec (median);
    }

    PITCH_OBJECT = new_aubio_pitch(PITCH_MODE, WINDOW_SIZE, HOP_SIZE, SAMPLE_RATE);
    ONSET_OBJECT = new_aubio_onset(ONSET_MODE, WINDOW_SIZE, HOP_SIZE, SAMPLE_RATE);
    aubio_pitch_set_unit(PITCH_OBJECT, PITCH_UNIT);
    aubio_onset_set_threshold(ONSET_OBJECT, ONSET_THRESHOLD);

    timeout(timeout_sec,1);
}

void destructor()
{
    del_aubio_pitch(PITCH_OBJECT);
    del_aubio_onset(ONSET_OBJECT);
    del_fvec(INPUT);
    del_fvec(OUTPUT);
    del_fvec(ONSET);

    if(median)
    {
      del_fvec(note_buffer);
      del_fvec(note_buffer2);
    }

    aubio_cleanup();
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
          std::cout << ": maximum input channels = " << info.inputChannels;
      std::cout << ", name = " << info.name;
      std::cout << ", is default = " << info.isDefaultInput << std::endl;
        }
      }
}

/*
/ fonction qui recoit une note de process_block et cumul pour créer un triplet
*/
void send_noteon (smpl_t pitch, int velo)
{
  if(pitch < 40 || pitch > 84) //50=D3 and 84=C6
  {
      std::cout << "Strange pitch of " << pitch << " detected" << std::endl;
      return;
  }

  if(nbr_notes==0) timeout(timeout_sec,1); //restart timer

  else if(notes_array[nbr_notes-1]==pitch)
  {
      std::cout << "Same pitch detected" << std::endl;
      return;
  }

  if(!timeout(timeout_sec,0))
  {
    notes_array[nbr_notes]=pitch;

    if(nbr_notes==2)
    {
        int interval1 = notes_array[1]-notes_array[0];
        int interval2 = notes_array[2]-notes_array[1];
        std::cout << "INTERVALS: " << interval1 << " & " << interval2 << ", with last ";
        //send_command(interval1, interval2);
        nbr_notes=0;
    }
    else nbr_notes++;
  }
  else	//timed out!
  {
	nbr_notes=0;
	notes_array[nbr_notes]=pitch;
	nbr_notes++;
  }
  std::cout << "NOTE: " << pitch << "(Velo:" << velo << ")"<<std::endl;
}

/*
/ note_append, get_note et process_block viennent de l'exemple : http://aubio.org/doc/0.4.1/examples_2aubionotes_8c-example.html
*/
void note_append (fvec_t * note_buffer, smpl_t curnote)
{
  uint_t i = 0;
  for (i = 0; i < note_buffer->length - 1; i++)
  {
    note_buffer->data[i] = note_buffer->data[i + 1];
  }

  note_buffer->data[note_buffer->length - 1] = curnote;
  return;
}

uint_t get_note (fvec_t * note_buffer, fvec_t * note_buffer2)
{
  uint_t i;
  for (i = 0; i < note_buffer->length; i++)
  {
    note_buffer2->data[i] = note_buffer->data[i];
  }

  return fvec_median (note_buffer2);
}

void process_block (float* input, int bufferSize)
{
    smpl_t new_pitch, curlevel;
    fvec_zeros(OUTPUT);

    for(int i = 0; i < bufferSize; i++)
    {
        INPUT->data[i] = input[i];
    }

    aubio_onset_do(ONSET_OBJECT, INPUT, ONSET);
    aubio_pitch_do (PITCH_OBJECT, INPUT, OUTPUT);
    new_pitch = fvec_get_sample(OUTPUT, 0);

      if(median)
      {
        note_append(note_buffer, new_pitch);
      }
      /* curlevel is negatif or 1 if silence */
      curlevel = aubio_level_detection(INPUT, SILENCE_THRESHOLD);
      if (fvec_get_sample(ONSET, 0)) {
        /* test for silence */
        if (curlevel == 1.) {
          if (median) isready = 0;
          /* send note off */
          //send_noteon(curnote,0);
        } else {
          if (median) {
            isready = 1;
          } else {
            /* kill old note */
            //send_noteon(curnote,0);
            /* get and send new one */
            send_noteon(new_pitch,127+(int)floor(curlevel));
            curnote = new_pitch;
          }
        }
      }
      else {
        if (median) {
          if (isready > 0)
            isready++;
          if (isready == median)
          {
            /* kill old note */
            //send_noteon(curnote,0);
            newnote = get_note(note_buffer, note_buffer2);
            curnote = newnote;
            /* get and send new one */
            if (curnote>45){
              send_noteon(curnote, 127 + (int)floor(curlevel));
            }
          }
        } // if median
  }
}

int captureOutput(void *outputBuffer, void *inputBuffer, unsigned int bufferSize,
                  double streamTime, RtAudioStreamStatus status, void *userData)
{
    if (status){std::cout << "Stream overflow detected!" << std::endl;}

    float* in = (float*) inputBuffer;

    process_block(in, bufferSize);

    return 0;
}

void openStream()
{
    RtAudio adc;

    if(adc.getDeviceCount() < 1)
    {
        std::cout << "No audio devices found!" << std::endl;
        exit( 0 );
    }

    getAudioDevicesInfo();

    RtAudio::StreamParameters parameters;
    std::cout << "Selecting default audio input" << std::endl;
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

    std::cout << "Recording ... press <CTRL-C> to quit." << std::endl;;
    ros::spin();

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

void dynamicParametersCallback(aubionode::audioConstantsConfig &config, uint32_t level)
{
    ONSET_THRESHOLD = config.onset_threshold;
    SILENCE_THRESHOLD = config.silence_threshold;

    ROS_INFO("Parameters changed");
}

int main(int argc, char **argv)
{
    std::string rosname = "Aubionode";
    std::string temp_arg;
    ros::init(argc, argv, rosname);
    ros::NodeHandle node;

    dynamic_reconfigure::Server<aubionode::audioConstantsConfig> dynamicReconfigServer;
    dynamic_reconfigure::Server<aubionode::audioConstantsConfig>::CallbackType dynamicReconfigCallback;
    dynamicReconfigCallback = boost::bind(&dynamicParametersCallback, _1, _2);
    dynamicReconfigServer.setCallback(dynamicReconfigCallback);

    setup();

    openStream();

    destructor();

	return 0;
}

