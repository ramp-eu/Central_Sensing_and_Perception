// holonomic.cpp

// communicate via RS232 serial with a remote uController. 
// communicate with ROS using String type messages.
// subscribe to command messages from ROS
// publish command responses to ROS

// program parameters - ucontroller# (0,1), serial port, baud rate

//Thread main
//  Subscribe to ROS String messages and send as commands to uController
//Thread receive
//  Wait for responses from uController and publish as a ROS messages


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>

#include <unistd.h> // UNIX standard function definitions


#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Joy.h>


#define DEFAULT_BAUDRATE 38400
//#define DEFAULT_BAUDRATE 19200
//#define DEFAULT_BAUDRATE 9600

//#define DEFAULT_SERIALPORT "/dev/ttyUSB1"
#define DEFAULT_SERIALPORT "/dev/ttyUSB0"

bool bDeadMan=true;

//Global data
FILE *fpSerial = NULL;   	// serial port file pointer
int fd=-1; 								// file description for the serial port

ros::Publisher ucResponseMsg;

int ucIndex;          //ucontroller index number

//****************************************************************************
//	VEHICLE STATE
float x=0;
float y=0;
float th=0;
float vx=0;
float vy=0;
float vth=0;
//****************************************************************************
static float wheel[4]={0,0,0,0};
//****************************************************************************
//	VEHICLE PARAMS
double tanA=1;	// A straying angle A for our vehicle is estimated by a relationship between forward traveling distance and
								// lateral traveling distance par one wheel rotation.
				
								// For rollers mounted at 45 deg tanA=1, but due to friction it is not exactly 1 and should be found experimentaly
double prosli_0=0,prosli_1=0,prosli_2=0,prosli_3=0,prosli_ziro=0;
//****************************************************************************

ros::Time current_time, last_time;
ros::Publisher odom_pub;
ros::Subscriber cmd_vel;
ros::Subscriber joy_sub_;
ros::Subscriber joy_sub_2;

/** CRC calculator
 *************************************
 *	- 
 *************************************
 * @todo
 *************************************
 */
unsigned int X_crc_ccitt_update (unsigned int crc, unsigned char data)
{
	data ^= (unsigned char) crc & 0x00ff;
	data ^= data << 4;

	return ((((unsigned int)data << 8) | (unsigned char)(crc >> 8)) ^ (unsigned char)(data >> 4)
	^ ((unsigned int)data << 3));
}

/** CRC calculator
 *************************************
 *	- 
 *************************************
 * @todo
 *************************************
 */
unsigned short CRC16_BlockChecksum(char *buf, int nbytes)
{
	unsigned n, crc;
	
	crc = 0xffff;
	for (n = 0; n < nbytes; n++) {
		crc = X_crc_ccitt_update (crc, *(buf + n));
	}
	return(crc);
}

//Initialize serial port, return file descriptor
FILE *serialInit(char * port, int baud)
{
  int BAUD = 0;
  struct termios newtio;
  FILE *fp = NULL;

 //Open the serial port as a file descriptor for low level configuration
 // read/write, not controlling terminal for process,
  fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY );
  if ( fd<0 )
  {
    ROS_ERROR("serialInit: Could not open serial device %s",port);
    return fp;
  }

  // set up new settings
  memset(&newtio, 0,sizeof(newtio));
  newtio.c_cflag =  CS8 | CLOCAL | CREAD;  //no parity, 1 stop bit

  newtio.c_iflag = IGNCR;    //ignore CR, other options off
  newtio.c_iflag |= IGNBRK;  //ignore break condition

  newtio.c_oflag = 0;        //all options off

  newtio.c_lflag = ICANON;     //process input as lines

  // activate new settings
  tcflush(fd, TCIFLUSH);
  //Look up appropriate baud rate constant
  switch (baud)
  {
     //case 115200:
     //default:
     //   BAUD = B115200;
     //   break;
     case 38400:
     default:
        BAUD = B38400;
        break;
     case 19200:
        BAUD  = B19200;
        break;
     case 9600:
        BAUD  = B9600;
        break;
     case 4800:
        BAUD  = B4800;
        break;
     case 2400:
        BAUD  = B2400;
        break;
     case 1800:
        BAUD  = B1800;
        break;
     case 1200:
        BAUD  = B1200;
        break;
  }  //end of switch baud_rate
  if (cfsetispeed(&newtio, BAUD) < 0 || cfsetospeed(&newtio, BAUD) < 0)
  {
    ROS_ERROR("serialInit: Failed to set serial baud rate: %d", baud);
    close(fd);
    return NULL;
  }
  tcsetattr(fd, TCSANOW, &newtio);
  tcflush(fd, TCIOFLUSH);

  //Open file as a standard I/O stream
  fp = fdopen(fd, "r+");
  if (!fp) {
    ROS_ERROR("serialInit: Failed to open serial stream %s", port);
    fp = NULL;
  }
  return fp;
} //serialInit

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	if (joy->buttons[1]==1)// || joy->buttons[2]==1)
	{
		bDeadMan=false;
	}
	else
	{
		bDeadMan=true;
	}
} //joyCallback

#define DLE_BYTE 0xF3
#define ETX_BYTE 0x5A

unsigned char bufferTx1[256];
unsigned char ucpInTx1=0;
unsigned char ucpOutTx1=0;
unsigned char checksumTx1;

union unFloat
{
	unsigned char uc[4];
  float f;
};

union unInt
{
	unsigned char uc[2];
	int i;
};

inline void PushTx1(unsigned char data)
{
   bufferTx1[ucpInTx1] = data;
   ucpInTx1++;
}

void BeginPacketCOM1(unsigned char PacketID)//, unsigned char PacketLength)
{
	ucpInTx1=0;
  PushTx1(DLE_BYTE);
  PushTx1(PacketID);
  checksumTx1=PacketID;
   
  //WITHOUT PACKET LENGTH
  //PushTx1(PacketLength);
  //checksumTx1+=PacketLength;
}

void EndPacketCOM1()
{
    checksumTx1=255-checksumTx1;
    PushTx1(checksumTx1);
    if (checksumTx1==DLE_BYTE)
   	    PushTx1(checksumTx1);

    PushTx1(DLE_BYTE);
    PushTx1(ETX_BYTE);
}

void PushByteCOM1(unsigned char c)
{
	PushTx1(c);
 	checksumTx1+=c;
  if (c==DLE_BYTE)
		PushTx1(c);
}

void PushIntCOM1(int i)
{
   unInt uiTemp;
   uiTemp.i=i;
   PushByteCOM1(uiTemp.uc[0]);
   PushByteCOM1(uiTemp.uc[1]);
}

void PushFloatCOM1(float f)
{
   unFloat uf;
   uf.f=f;
   PushByteCOM1(uf.uc[0]);
   PushByteCOM1(uf.uc[1]);
   PushByteCOM1(uf.uc[2]);
   PushByteCOM1(uf.uc[3]);
}

//Process ROS command message, send to uController
void ucCommandCallback(const geometry_msgs::Twist& cmd_vel)
{

	//ROS CONVENTION FOR BODY AXES ORIENTATION:
		// X-FORWARD
		// Y-LEFT
		// Z-UP -> POSITIVE ROTATION=TURNING LEFT (RIGHT HAND RULE) - RIGHT WHEELS FORWARDS - LEFT WHEELS BACKWARDS ...
	//FRONT-LEFT WHEEL BACKWARDS	+ REAR-LEFT WHEEL FORWARDS -> GOING LEFT (linear.y POSITIVE)
	//FRONT-RIGHT WHEEL BACKWARDS + REAR-RIGHT WHEEL FORWARDS -> GOING RIGHT (linear.y NEGATIVE)
	//static float wheel[4]={0,0,0,0};
	//static float r=0.5;//EQUIVALENT RADIUS OF ROTATION
	static float r=0.75;//EQUIVALENT RADIUS OF ROTATION
	ROS_INFO("cmd_vel x: %.1f y:%.1f z:%.1f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);  //odkomentirati
	
	//CONVERT VELOCITIES TO TARGET VELOCITY FOR EACH WHEEL
	wheel[0]=cmd_vel.linear.x-cmd_vel.linear.y-r*cmd_vel.angular.z;//FRONT LEFT
	wheel[1]=cmd_vel.linear.x+cmd_vel.linear.y+r*cmd_vel.angular.z;//FRONT RIGHT
	wheel[2]=cmd_vel.linear.x-cmd_vel.linear.y+r*cmd_vel.angular.z;//REAR RIGHT
	wheel[3]=cmd_vel.linear.x+cmd_vel.linear.y-r*cmd_vel.angular.z;//REAR LEFT
	
	BeginPacketCOM1(41);
	PushFloatCOM1(wheel[0]);
	PushFloatCOM1(wheel[1]);
	PushFloatCOM1(wheel[2]);
	PushFloatCOM1(wheel[3]);
	EndPacketCOM1();
	int sent=write(fd, bufferTx1, ucpInTx1);
	//ROS_INFO("Sent %d bytes: %.1f %.1f %.1f %.1f",sent,wheel[0],wheel[1],wheel[2],wheel[3]);
	
/*
	// for some reason this doesn't work any more - no signal on the cable, although number of sent bytes is > 0 ...
	fprintf(fpSerial, "v1%d\r", (int)(wheel[0]*1000));
	ROS_INFO("Sent %d bytes:v1 %d",sent,(int)(wheel[0]*1000));
	fprintf(fpSerial, "v2%d\r", (int)(wheel[1]*1000));
	ROS_INFO("Sent:v2%d",(int)(wheel[1]*1000));
	fprintf(fpSerial, "v3%d\r", (int)(wheel[2]*1000));
	ROS_INFO("Sent:v3%d",(int)(wheel[2]*1000));
	fprintf(fpSerial, "v4%d\r", (int)(wheel[3]*1000));
	ROS_INFO("Sent:v4%d",(int)(wheel[3]*1000));
*/

/*
	char buffer[16];
	int n=0;
	int sent=0;
	n=sprintf(buffer, "v1%d\r", (int)(wheel[0]*1500));
	sent=write(fd, buffer, n);
	//ROS_INFO("Sent %d bytes: %s",sent,buffer);
	n=sprintf(buffer, "v2%d\r", (int)(wheel[1]*1500));
	sent=write(fd, buffer, n);
	n=sprintf(buffer, "v3%d\r", (int)(wheel[2]*1500));
	sent=write(fd, buffer, n);
	n=sprintf(buffer, "v4%d\r", (int)(wheel[3]*1500));
	sent=write(fd, buffer, n);
*/
		
/*
	static float r=0.5;//EQUIVALENT RADIUS OF ROTATION
	double left_track=-cmd_vel.linear.x-r*cmd_vel.angular.z;
	double right_track=-cmd_vel.linear.x+r*cmd_vel.angular.z;
	
	// set limit to 1 m/s ...
	if (left_track>1.0) left_track=1.0;
	if (left_track<-1.0) left_track=-1.0;
	if (right_track>1.0) right_track=1.0;
	if (right_track<-1.0) right_track=-1.0;
	
	ROS_INFO("left/right track %.1f %.1f",left_track,right_track);
	
	// scale to -127 to 127
	char cleft_track=round(left_track*127);
	char cright_track=round(right_track*127);
	
	char buffer[12];
	buffer[0]=0x00;
	buffer[1]=0x40;	// MT/COMM/PAGE, MT = Machine Type MVF5 = 0x40,COMM = Communication flag from OCU, if &= 0x80 OCU is not ready
									// PAGE = Page identifier 0 - 0x07
	buffer[2]=0x00;	// SERIAL NUMBER
	buffer[3]=cleft_track;
	buffer[4]=cright_track;
	buffer[5]=0x00; //Left Joystick Z axes Value +-127
	buffer[6]=0x00; //Right Joystick X  axes Value +-127
	buffer[7]=0x00; //Right Joystick Y  axes Value +-127
	buffer[8]=0x00; //Right Joystick Z  axes Value +-127
	buffer[9]=0x00; //Command 1 to 8, depending on page
	
	unsigned short crc=CRC16_BlockChecksum(buffer, 10);
	unsigned short *pcrc=(unsigned short *)&buffer[10];
	*pcrc=0xfefe;//crc;
	
	if (!bDeadMan)
	{
		int n = write(fd, buffer, 12);
		if (n < 0)
		{
			std::cout<<"write() of bytes failed!"<<std::endl;
		}
		else
		{
			if (cleft_track!=0 || cright_track!=0)
			{
			//std::cout<<n<<" bytes written !"<<std::endl;
				ROS_INFO("cmd: %.2X.%.2X.%.2X.%.2X.%.2X.%.2X.%.2X.%.2X.%.2X.%.2X.%.2x.%.2x",buffer[0],buffer[1],buffer[2],
					(unsigned)(unsigned char)buffer[3],
					(unsigned)(unsigned char)buffer[4],
					buffer[5],buffer[6],buffer[7],buffer[8],buffer[9],
					(unsigned)(unsigned char)buffer[10],
					(unsigned)(unsigned char)buffer[11]);
			}
		}
  }
  else
  {
  	ROS_INFO("Dead men command block !");
  }
*/
  
} //ucCommandCallback


//Receive command responses from robot uController
//and publish as a ROS message
void *rcvThread(void *arg)
{
	int rcvBufSize = 32;
	char ucResponse[rcvBufSize];   //response string from uController
	char *bufPos;
	std_msgs::String msg;
	std::stringstream ss;
	
	static tf::TransformBroadcaster odom_broadcaster;

	ROS_INFO("rcvThread: receive thread running");
	
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	while (ros::ok()) 
	{
		//sleep(1);
		bufPos = fgets(ucResponse, rcvBufSize, fpSerial);
    	   	
		if (bufPos != NULL) 
		{
			current_time = ros::Time::now();	// NEW MESSAGE RECEIVED JUST NOW...
			
			//static float fData[5];
			static double fData[5]={0};
			//double prosli_0,prosli_1,prosli_2,prosli_3,prosli_ziro;
  		//ROS_INFO("uc%dResponse: %s", ucIndex, ucResponse);  //podaci s enkodera
  		std::string s=ucResponse;
  		
  		std::cout<<s;
  		
  		
  		size_t pos = 0;
			std::string token;
			int count=0;
			int brojac=0;
			bool uzmi = true;
			
			//while ((pos = s.find('\t')) != std::string::npos && count<5)
			while ((pos = s.find(' ')) != std::string::npos && count<6)  
			{
    				token = s.substr(0, pos);    			
				//ROS_INFO("SUBSTRING: %s",token.c_str());
    				//fData[count]= ::atof(token.c_str());
				fData[brojac]= atof(token.c_str());
    				s.erase(0, pos + 1);
				if (count > 0){
					brojac++;    
				}
    				count++;
			}
			//inicijalizacija ziroskopa na 0
			fData[4] = -(fData[4]-1515);
			//fData[4] = fData[4]-1515;
			if (fData[4] > 1000 || fData[4] < -1000){
			    fData[4] = prosli_ziro;
			}
			prosli_ziro=fData[4];
			
			 //ovaj dio vise nije potreban jer je rijeseno tako da ne uzima u obzir te podatke
			if (fData[0]>10 || fData[0]<-10){    //rjesenje buga da postavlja prvi kotac u 1515
			   fData[0]=prosli_0;
			}else prosli_0=fData[0];
			if (fData[1]>10 || fData[1]<-10){    //rjesenje buga da postavlja 2 kotac u 1515
			   fData[1]=prosli_1;
			}else prosli_1=fData[1];
			if (fData[2]>10 || fData[2]<-10){    //rjesenje buga da postavlja 3 kotac u 1515
			   fData[2]=prosli_2;
			}else prosli_2=fData[2];
			if (fData[3]>10 || fData[3]<-10){    //rjesenje buga da postavlja 4 kotac u 1515
			   fData[3]=prosli_3;
			}else prosli_3=fData[3];
			if (fData[4]==-1515){    //rjesenje buga da postavlja ziroskop u 1515
			   fData[4]=prosli_ziro;
			}else prosli_ziro=fData[4];
			/*			
			prosli_0=fData[0];
			prosli_1=fData[1];
			prosli_ziro=fData[4];
			*/

			//compute vehicle velocity from individual wheel velocities
			//racunanje preko podataka s enkodera
			float B=0.65; 

			vx=(fData[0]+fData[1]+fData[2]+fData[3])/4;
			vy=(-fData[0]+fData[1]-fData[2]+fData[3])*tanA/4;
			vth=(-fData[0]+fData[1]+fData[2]-fData[3])*B;
			//vth=fData[4]*3.14159265/180;// USE z-omega FROM ONBOARD GYROSCOPE, deg/sec to rad/sec

			//racunanje preko poslanih brzina 
			/*
			vx=(wheel[0]+wheel[1]+wheel[2]+wheel[3])/4;
			vy=(-wheel[0]+wheel[1]+wheel[2]-wheel[3])*tanA/4;
			vth=(-wheel[0]+wheel[1]-wheel[2]+wheel[3])*0.3;	
			*/
      			
			//compute odometry in a typical way given the velocities of the robot
			double dt = (current_time - last_time).toSec();
			double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
			double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
			double delta_th = vth * dt;

  		//x += delta_x;
  		//y += delta_y;
  		//th += delta_th;
		
		//if (dt < 0.001){
		//    uzmi = false;	//rijesen problem 2. prolaza i krivog postavljanja vrijednosti
		//}
		
		//if (uzmi){
		x = x + delta_x;
  		y = y + delta_y;
  		th = th + delta_th;

		//ROS_INFO("DECODED: %.2f %.2f %.2f %.2f %.2f dt:%f POS:%.2f %.2f",fData[0],fData[1],fData[2],fData[3],fData[4],dt,x,y);
		//ROS_INFO("DECODED: %.2f %.2f %.2f %.2f %.2f dt:%f x=%.2f y=%.2f th=%f",fData[0],fData[1],fData[2],fData[3],fData[4],dt,x,y,th);
		//ROS_INFO("dt:%f x=%.2f y=%.2f th=%f",dt,x,y,th);
  		//since all odometry is 6DOF we'll need a quaternion created from yaw
  		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  		//first, we'll publish the transform over tf
  		geometry_msgs::TransformStamped odom_trans;
  		odom_trans.header.stamp = current_time;
  		odom_trans.header.frame_id = "odom";
  		odom_trans.child_frame_id = "base_link";

  		odom_trans.transform.translation.x = x;
  		odom_trans.transform.translation.y = y;
  		odom_trans.transform.translation.z = 0.0;
  		odom_trans.transform.rotation = odom_quat;

  		//send the transform
  		odom_broadcaster.sendTransform(odom_trans);

  		//next, we'll publish the odometry message over ROS
  		nav_msgs::Odometry odom;
  		odom.header.stamp = current_time;
  		odom.header.frame_id = "odom";

  		//set the position
  		odom.pose.pose.position.x = x;
  		odom.pose.pose.position.y = y;
  		odom.pose.pose.position.z = 0.0;
  		odom.pose.pose.orientation = odom_quat;

  		//set the velocity
  		odom.child_frame_id = "base_link";
  		odom.twist.twist.linear.x = vx;
  		odom.twist.twist.linear.y = vy;
  		odom.twist.twist.angular.z = vth;

  		//publish the message
  		odom_pub.publish(odom);
		
  		
  		last_time = current_time;
		//}
    }
  }
  return NULL;
} //rcvThread


int main(int argc, char **argv)
{
  char port[20];    //port name
  int baud;     //baud rate 

  char topicSubscribe[20];
  char topicPublish[20];


  pthread_t rcvThrID;   //receive thread ID
  int err;

  //Initialize ROS
  ros::init(argc, argv, "holonomic");
  ros::NodeHandle rosNode;
  ROS_INFO("holonomic starting");


  //Open and initialize the serial port to the uController
  if (argc > 1) {
    if(sscanf(argv[1],"%d", &ucIndex)==1) {
      sprintf(topicSubscribe, "uc%dCommand",ucIndex);
      sprintf(topicPublish, "uc%dResponse",ucIndex);
    }
    else {
      ROS_ERROR("holonomic index parameter invalid");
      return 1;
    }
  }
  else {
    strcpy(topicSubscribe, "uc0Command");
    strcpy(topicPublish, "uc0Response");
  }

  strcpy(port, DEFAULT_SERIALPORT);
  if (argc > 2)
     strcpy(port, argv[2]);

  baud = DEFAULT_BAUDRATE;
  if (argc > 3) {
    if(sscanf(argv[3],"%d", &baud)!=1) {
      ROS_ERROR("holonomic baud rate parameter invalid");
      return 1;
    }
  }

  ROS_INFO("connection initializing (%s) at %d baud", port, baud);
   fpSerial = serialInit(port, baud);
  if (!fpSerial )
  {
    ROS_ERROR("unable to create a new serial port");
    return 1;
  }
  ROS_INFO("serial connection successful");
  
  //Create receive thread - it doesn't work well - it should be reprogrammed completely...
  odom_pub = rosNode.advertise<nav_msgs::Odometry>("odometrija", 100);
  
  err = pthread_create(&rcvThrID, NULL, rcvThread, NULL);
  if (err != 0) {
    ROS_ERROR("unable to create receive thread");
    return 1;
  }
  
  //Subscribe to ROS messages
  //cmd_vel = rosNode.subscribe("/husky/cmd_vel", 100, ucCommandCallback);
  //cmd_vel = rosNode.subscribe("/husky_velocity_controller/cmd_vel", 20, ucCommandCallback);
  cmd_vel = rosNode.subscribe("/husky_velocity_controller/cmd_vel", 20, ucCommandCallback);
	//joy_sub_= rosNode.subscribe("/joy_teleop/joy",20,joyCallback);   //nema potrebe za subscribe
	//joy_sub_2= rosNode.subscribe("/falconJoy",20,joyCallback);
	
	//joy_sub_ = rosNode.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
  //Setup to publish ROS messages
  //ros::Publisher odom_pub = rosNode.advertise<nav_msgs::Odometry>("odometrija", 100);   

  //Process ROS messages and send serial commands to uController
  ros::spin();
  fclose(fpSerial);
  ROS_INFO("holonomic stopping");
  
  return 0;
}
