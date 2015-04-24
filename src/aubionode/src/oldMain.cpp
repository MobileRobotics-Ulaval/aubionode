// Standard C/C++ libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctime>
#include <unistd.h>
#include <string.h> /* for strncpy */

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>

//library for ros
#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

//libraries for the array
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

//libraries for the control
#include "std_msgs/String.h"
#include <sstream>

#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

/*
/	sudo apt-get install expect-dev
/
*/
geometry_msgs::Pose dpose;
ros::Publisher aubio_node;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
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
        aubio_onset_do(ONSET_OBJECT, INPUT, OUTPUT);

        smpl_t new_pitch = fvec_get_sample(OUTPUT, 0);

        std::cout << new_pitch << std::endl;
    }
}

const char* get_ip()
{
  int fd;
 struct ifreq ifr;
 char *ip = new char[100];

 fd = socket(AF_INET, SOCK_DGRAM, 0);

 /* I want to get an IPv4 IP address */
 ifr.ifr_addr.sa_family = AF_INET;

 /* I want IP address attached to "eth0" */
 strncpy(ifr.ifr_name, "eth0", IFNAMSIZ-1);

 ioctl(fd, SIOCGIFADDR, &ifr);

 close(fd);

 /* display result */
 sprintf(ip,"%s", inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
 std::string s = ip;
 std::replace(s.begin(), s.end(), '.', '_');
 //ip=s.c_str();
 return s.c_str();
}

float remap(float s, float a1, float a2, float b1, float b2)
{
    return (b1 + (s-a1)*(b2-b1)/(a2-a1));
}

void get_aubio()
{
  char res[100];
  FILE *fp;
  float t=0,c=0,p=0,f=0;
  int count=0;
  float pbuf=0;

  /* Open the command for reading. */
  fp = popen("unbuffer /usr/bin/aubionotes", "r");
  if (fp == NULL) {
    printf("Failed to run command\n" );
    exit(1);
  }

  /* Read the output a line at a time - output it. */
  while (fgets(res, sizeof(res)-1, fp) != NULL) {//read(d, res, sizeof(res))>0){//fread (res,1,sizeof(res),fp)!= NULL){
    //printf("%s", res);
    sscanf(res,"Time=%f, chan=%f, pitch=%f, vol=%f",&t,&c,&p,&f);
    if(p!=0.0){
      if(count<=10){
	pbuf+=p;
	count++;
      }else{
	count=0;
	pbuf/=10.0;
	ROS_INFO("Pitch=%f\n",pbuf);
	
	dpose.position.x=remap(pbuf,50,150,0,6);
	dpose.position.x=remap(pbuf,50,150,0,6);
	
	//Convert from angle to Quaterion
	//Eigen::Quaterniond orientation = Eigen::Quaterniond(Eigen::AngleAxisd(remap(pbuf,50,150,-1.6,1.6), Eigen::Vector3d::UnitZ()));
        //dpose.orientation.x = orientation.x();
        //dpose.orientation.y = orientation.y();
        //dpose.orientation.z = orientation.z();
        //dpose.orientation.w = orientation.w();
	
	aubio_node.publish(dpose);
	pbuf=0;
      }
    }
    //printf("r");
    fflush(stdout);
  }

  /* close */
  pclose(fp);
  
  return;
}

int oldMain(int argc, char **argv)
{
    	char rosname[100];
	std::string temp_arg;
	//gethostname(rosname,100);
	sprintf(rosname,"aubionode_%s",get_ip());
	ros::init(argc, argv, rosname);
    	ros::NodeHandle node;
	int print=0;

	if (argc==2)
        {
          ROS_INFO("TARGET IS: %s", argv[1]);
        }
        else
        {
          ROS_ERROR("Failed to get param 'target' and 'pose source'");
	  	return 0;
        }
	temp_arg = argv[1];
	std::replace(temp_arg.begin(), temp_arg.end(), '.', '_');
	
	sprintf(rosname,"/%s/desired_deltapose",temp_arg.c_str());
    	aubio_node = node.advertise<geometry_msgs::Pose>("/desired_deltapose",1);
    	ros::Rate loop_rate(10);
	
	dpose.position.x=0;dpose.position.y=0;dpose.position.z=0;dpose.orientation.x=0;dpose.orientation.y=0;dpose.orientation.z=0;
	get_aubio();

	while (ros::ok())
	{
             	/////////////////////////////////
        	//if(print==5){ROS_INFO("Aubio output: %s",);
		//print=0;}
		//else {print++;}
	//        aubio_node.publish(dpose);
	//	ros::spinOnce();
	//	loop_rate.sleep();
	}
	return 0;
}
