/*

  CSC C85 - Embedded Systems - Project # 1 - EV3 Robot Localization
  
 This file provides the implementation of all the functionality required for the EV3
 robot localization project. Please read through this file carefully, and note the
 sections where you must implement functionality for your bot. 
 
 You are allowed to change *any part of this file*, not only the sections marked
 ** TO DO **. You are also allowed to add functions as needed (which must also
 be added to the header file). However, *you must clearly document* where you 
 made changes so your work can be properly evaluated by the TA.

 NOTES on your implementation:

 * It should be free of unreasonable compiler warnings - if you choose to ignore
   a compiler warning, you must have a good reason for doing so and be ready to
   defend your rationale with your TA.
 * It must be free of memory management errors and memory leaks - you are expected
   to develop high wuality, clean code. Test your code extensively with valgrind,
   and make sure its memory management is clean.
 
 In a nutshell, the starter code provides:
 
 * Reading a map from an input image (in .ppm format). The map is bordered with red, 
   must have black streets with yellow intersections, and buildings must be either
   blue, green, or be left white (no building).
   
 * Setting up an array with map information which contains, for each intersection,
   the colours of the buildings around it in ** CLOCKWISE ** order from the top-left.
   
 * Initialization of the EV3 robot (opening a socket and setting up the communication
   between your laptop and your bot)
   
 What you must implement:
 
 * All aspects of robot control:
   - Finding and then following a street
   - Recognizing intersections
   - Scanning building colours around intersections
   - Detecting the map boundary and turning around or going back - the robot must not
     wander outside the map (though of course it's possible parts of the robot will
     leave the map while turning at the boundary)

 * The histogram-based localization algorithm that the robot will use to determine its
   location in the map - this is as discussed in lecture.

 * Basic robot exploration strategy so the robot can scan different intersections in
   a sequence that allows it to achieve reliable localization
   
 * Basic path planning - once the robot has found its location, it must drive toward a 
   user-specified position somewhere in the map.

 --- OPTIONALLY but strongly recommended ---
 
  The starter code provides a skeleton for implementing a sensor calibration routine,
 it is called when the code receives -1  -1 as target coordinates. The goal of this
 function should be to gather informatin about what the sensor reads for different
 colours under the particular map/room illumination/battery level conditions you are
 working on - it's entirely up to you how you want to do this, but note that careful
 calibration would make your work much easier, by allowing your robot to more
 robustly (and with fewer mistakes) interpret the sensor data into colours. 
 
   --> The code will exit after calibration without running localization (no target!)
       SO - your calibration code must *save* the calibration information into a
            file, and you have to add code to main() to read and use this
            calibration data yourselves.
   
 What you need to understand thoroughly in order to complete this project:
 
 * The histogram localization method as discussed in lecture. The general steps of
   probabilistic robot localization.

 * Sensors and signal management - your colour readings will be noisy and unreliable,
   you have to handle this smartly
   
 * Robot control with feedback - your robot does not perform exact motions, you can
   assume there will be error and drift, your code has to handle this.
   
 * The robot control API you will use to get your robot to move, and to acquire 
   sensor data. Please see the API directory and read through the header files and
   attached documentation
   
 Starter code:
 F. Estrada, 2018 - for CSC C85 
 
*/

#include "EV3_Localization.h"

int map[400][4];            // This holds the representation of the map, up to 20x20
                            // intersections, raster ordered, 4 building colours per
                            // intersection.
int sx, sy;                 // Size of the map (number of intersections along x and y)
double beliefs[400][4];     // Beliefs for each location and motion direction
int mnl = -1;
int tr_ang;
int tr_rt;
int main(int argc, char *argv[])
{
 char mapname[1024];
 int dest_x, dest_y, rx, ry;
 unsigned char *map_image;
 
 memset(&map[0][0],0,400*4*sizeof(int));
 sx=0;
 sy=0;
 
 if (argc<4)
 {
  fprintf(stderr,"Usage: EV3_Localization map_name dest_x dest_y\n");
  fprintf(stderr,"    map_name - should correspond to a properly formatted .ppm map image\n");
  fprintf(stderr,"    dest_x, dest_y - target location for the bot within the map, -1 -1 calls calibration routine\n");
  exit(1);
 }
 strcpy(&mapname[0],argv[1]);
 dest_x=atoi(argv[2]);
 dest_y=atoi(argv[3]);

 if (dest_x==-1&&dest_y==-1)
 {
  calibrate_sensor();
  exit(1);
 }

 /******************************************************************************************************************
  * OPTIONAL TO DO: If you added code for sensor calibration, add just below this comment block any code needed to
  *   read your calibration data for use in your localization code. Skip this if you are not using calibration
  * ****************************************************************************************************************/
 
 
 // Your code for reading any calibration information should not go below this line //
 
 map_image=readPPMimage(&mapname[0],&rx,&ry);
 if (map_image==NULL)
 {
  fprintf(stderr,"Unable to open specified map image\n");
  exit(1);
 }
 
 if (parse_map(map_image, rx, ry)==0)
 { 
  fprintf(stderr,"Unable to parse input image map. Make sure the image is properly formatted\n");
  free(map_image);
  exit(1);
 }

 if (dest_x<0||dest_x>=sx||dest_y<0||dest_y>=sy)
 {
  fprintf(stderr,"Destination location is outside of the map\n");
  free(map_image);
  exit(1);
 }

 // Initialize beliefs - uniform probability for each location and direction
 for (int j=0; j<sy; j++)
  for (int i=0; i<sx; i++)
  {
   beliefs[i+(j*sx)][0]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][1]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][2]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][3]=1.0/(double)(sx*sy*4);
  }

 // Open a socket to the EV3 for remote controlling the bot.
 if (BT_open(HEXKEY)!=0)
 {
  fprintf(stderr,"Unable to open comm socket to the EV3, make sure the EV3 kit is powered on, and that the\n");
  fprintf(stderr," hex key for the EV3 matches the one in EV3_Localization.h\n");
  free(map_image);
  exit(1);
 }

 fprintf(stderr,"All set, ready to go!\n");
 
/*******************************************************************************************************************************
 *
 *  TO DO - Implement the main localization loop, this loop will have the robot explore the map, scanning intersections and
 *          updating beliefs in the beliefs array until a single location/direction is determined to be the correct one.
 * 
 *          The beliefs array contains one row per intersection (recall that the number of intersections in the map_image
 *          is given by sx, sy, and that the map[][] array contains the colour indices of buildings around each intersection.
 *          Indexing into the map[][] and beliefs[][] arrays is by raster order, so for an intersection at i,j (with 0<=i<=sx-1
 *          and 0<=j<=sy-1), index=i+(j*sx)
 *  
 *          In the beliefs[][] array, you need to keep track xof 4 values per intersection, these correspond to the belief the
 *          robot is at that specific intersection, moving in one of the 4 possible directions as follows:
 * 
 *          beliefs[i][0] <---- belief the robot is at intersection with index i, facing UP
 *          beliefs[i][1] <---- belief the robot is at intersection with index i, facing RIGHT
 *          beliefs[i][2] <---- belief the robot is at intersection with index i, facing DOWN
 *          beliefs[i][3] <---- belief the robot is at intersection with index i, facing LEFT
 * 
 *          Initially, all of these beliefs have uniform, equal probability. Your robot must scan intersections and update
 *          belief values based on agreement between what the robot sensed, and the colours in the map. 
 * 
 *          You have two main tasks these are organized into two major functions:
 * 
 *          robot_localization()    <---- Runs the localization loop until the robot's location is found
 *          go_to_target()          <---- After localization is achieved, takes the bot to the specified map location
 * 
 *          The target location, read from the command line, is left in dest_x, dest_y
 * 
 *          Here in main(), you have to call these two functions as appropriate. But keep in mind that it is always possible
 *          that even if your bot managed to find its location, it can become lost again while driving to the target
 *          location, or it may be the initial localization was wrong and the robot ends up in an unexpected place - 
 *          a very solid implementation should give your robot the ability to determine it's lost and needs to 
 *          run localization again.
 *
 *******************************************************************************************************************************/  

 // HERE - write code to call robot_localization() and go_to_target() as needed, any additional logic required to get the
 //        robot to complete its task should be here.
//int tl, tr,bl, br;
//scan_intersection(&tl, &tr, &bl, &br);
 int rob_x = 2;
 int rob_y = 3;
 int dir = 2;
 // call scan
//  int tl = 0, tr = 0 , br = 0 , bl = 0 ;
// scan_intersection(&tl, &tr, &br, &bl);
 robot_localization(&rob_x, &rob_y, &dir);
 printf("--------------------------------\n");
 printf("localisation is complete\n");
 printf("-----------------------------------\n");
 go_to_target(rob_x, rob_y, dir, dest_x, dest_y);

  //rotate(45);
 // Cleanup and exit - DO NOT WRITE ANY CODE BELOW THIS LINE

  
 BT_close();
 free(map_image);
 exit(0);
}

void rotate(int angle){
  if(abs(angle) > 360){
    // printf("For safety, angle should be in interval [-360, 360]");
  }
 int ang = 0;
 int rt = 0;
 BT_all_stop(1);
 BT_read_gyro(PORT_2, 1, &ang, &rt);
 //printf("start ang : %d\n", ang);
 int pow;
 if(angle > 0) pow = 12;
 else pow = -12;

 BT_turn(MOTOR_A, pow, MOTOR_D, -pow);
 while(ang < angle - 2){
  //printf("ang : %d\n", ang);
  BT_read_gyro(PORT_2, 0, &ang, &rt);
 }
 
 BT_all_stop(1); // Ensure the robot stops after turning

}

int checkColor(char sensor_port, int num){
  // num : the number of checking made

  int col[8] = {0,0,0,0,0,0,0,0};
  for(int i = 0; i < num; i++){
    int d = BT_read_colour_sensor(sensor_port);
    if(d >= 0 && d < 8){
      col[d]++;
    }
  }

  int max = 0;
  for(int j= 1; j < 8; j++){
    if(col[j] > col[max]){
      max = j;
    }
  }
  return max;
}

int find_street(void)   {
 /*
  * This function gets your robot onto a street, wherever it is placed on the map. You can do this in many ways, but think
  * about what is the most effective and reliable way to detect a street and stop your robot once it's on it.
  * 
  * You can use the return value to indicate success or failure, or to inform the rest of your code of the state of your
  * bot after calling this function
  */   
  while(1) {
    int d = checkColor(PORT_3, 9);
    if (d == 1 || d== 4) {
      BT_all_stop(1);
      return 1;
    } else {
      int ang = 0;
      int rt = 0;
      BT_read_gyro(PORT_2, 1, &ang, &rt);
      BT_turn(MOTOR_A, -6, MOTOR_D, 6);
      while (abs(ang) < 40) {
        BT_read_gyro(PORT_2, 0, &ang, &rt);
        int max = checkColor(PORT_3, 9);
        if (max == 1 || max == 4) { // Black detected
          BT_all_stop(1);
          return 1;
        }
      }
      BT_all_stop(1);

      // Rotate slowly to the right up to 90 degrees (45 degrees from the original position)
      ang = 0;
      BT_read_gyro(PORT_2, 1, &ang, &rt);
      BT_turn(MOTOR_A, 6, MOTOR_D, -6);
      while (abs(ang) < 80) {
        BT_read_gyro(PORT_2, 0, &ang, &rt);
        int max = checkColor(PORT_3, 9);
        if (max == 1 || max == 4) { // Black detected
          BT_all_stop(1);
          return 1;
        }
      }
      BT_all_stop(1);

      // If no street is found, move forward a bit and try again
      BT_drive(MOTOR_A, MOTOR_D, -10);
      usleep(500000); // Move forward for 0.5 seconds
      BT_all_stop(1);
    }
  }
  return 0;
}

int drive_along_street(void)
{
 /*
  * This function drives your bot along a street, making sure it stays on the street without straying to other pars of
  * the map. It stops at an intersection.
  * 
  * You can implement this in many ways, including a controlled (PID for example), a neural network trained to track and
  *
  *  follow streets, or a carefully coded process of scanning and moving. It's up to you, feel free to consult your TA
  * or the course instructor for help carrying out your plan.
  * 
  * You can use the return value to indicate success or failure, or to inform the rest of your code of the state of your
  * bot after calling this function.
  */   
  return(0);
}

int scan_intersection(int *tl, int *tr, int *br, int *bl)
{
 /*
  * This function carries out the intersection scan - the bot should (obviously) be placed at an intersection for this,
  * and the specific set of actions will depend on how you designed your bot and its sensor. Whatever the process, you
  * should make sure the intersection scan is reliable - i.e. the positioning of the sensor is reliably over the buildings
  * it needs to read, repeatably, and as the robot moves over the map.
  * 
  * Use the APIs sensor reading calls to poll the sensors. You need to remember that sensor readings are noisy and 
  * unreliable so * YOU HAVE TO IMPLEMENT SOME KIND OF SENSOR / SIGNAL MANAGEMENT * to obtain reliable measurements.
  * 
  * Recall your lectures on sensor and noise management, and implement a strategy that makes sense. Document your process
  * in the code below so your TA can quickly understand how it works.
  * 
  * Once your bot has read the colours at the intersection, it must return them using the provided pointers to 4 integer
  * variables:
  * 
  * tl - top left building colour
  * tr - top right building colour
  * br - bottom right building colour
  * bl - bottom left building colour
  * 
  * The function's return value can be used to indicate success or failure, or to notify your code of the bot's state
  * after this call.
  */
 
  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/

 // Rotate 45 degrees to face NE
 int ang = 0;
 int rt = 0;
 BT_read_gyro(PORT_2, 1, &ang, &rt);
 int pow = 12;
 //BT_turn(MOTOR_A, pow, MOTOR_D, -pow); // Slower turn
 rotate(45);
 usleep(1000);

 BT_all_stop(1); // Ensure the robot stops after turning
// robot already at the NW


 // Collect readings for 50 samples and determine the mode
 int mode = checkColor(PORT_3, 50);
 *tl = mode;

 // Print the value
 printf("Top-left building color: %d\n", *tl);

 // Repeat the process for the other three directions
 for (int i = 0; i < 3; i++) {
  // Rotate 90 degrees
  rotate(90);
  
  mode = checkColor(PORT_3, 50);
  // Store the mode in the appropriate variable and print the value
  if (i == 0) {
      *bl = mode;
      printf("Bottom-left building color: %d\n", *bl);
  } else if (i == 1) {
      *br = mode;
      printf("Bottom-right building color: %d\n", *br);
  } else if (i == 2) {
      *tr = mode;
      printf("Top-right building color: %d\n", *tr);
  }
 }
 //  printf("TESSSS\n");
 
 // Rotate back to the original direction (N)
 rotate(45);

 BT_all_stop(1); // Ensure the robot stops after turning
 return(0);
}

int turn_at_intersection(int turn_direction)
{
 /*
  * This function is used to have the robot turn either left or right at an intersection (obviously your bot can not just
  * drive forward!). 
  * 
  * If turn_direction=0, turn right, else if turn_direction=1, turn left.
  * 
  * You're free to implement this in any way you like, but it should reliably leave your bot facing the correct direction
  * and on a street it can follow. 
  * 
  * You can use the return value to indicate success or failure, or to inform your code of the state of the bot
  */
  return(0);
}

int robot_localization(int *robot_x, int *robot_y, int *direction)
{
 /*  This function implements the main robot localization process. You have to write all code that will control the robot
  *  and get it to carry out the actions required to achieve localization.
  *
  *  Localization process:
  *
  *  - Find the street, and drive along the street toward an intersection
  *  - Scan the colours of buildings around the intersection
  *  - Update the beliefs in the beliefs[][] array according to the sensor measurements and the map data
  *  - Repeat the process until a single intersection/facing direction is distintly more likely than all the rest
  * 
  *  * We have provided headers for the following functions:
  * 
  *  find_street()
  *  drive_along_street()
  *  scan_intersection()
  *  turn_at_intersection()
  * 
  *  You *do not* have to use them, and can write your own to organize your robot's work as you like, they are
  *  provided as a suggestion.
  * 
  *  Note that *your bot must explore* the map to achieve reliable localization, this means your intersection
  *  scanning strategy should not rely exclusively on moving forward, but should include turning and exploring
  *  other streets than the one your bot was initially placed on.
  * 
  *  For each of the control functions, however, you will need to use the EV3 API, so be sure to become familiar with
  *  it.
  * 
  *  In terms of sensor management - the API allows you to read colours either as indexed values or RGB, it's up to
  *  you which one to use, and how to interpret the noisy, unreliable data you're likely to get from the sensor
  *  in order to update beliefs.
  * 
  *  HOWEVER: *** YOU must document clearly both in comments within this function, and in your report, how the
  *               sensor is used to read colour data, and how the beliefs are updated based on the sensor readings.
  * 
  *  DO NOT FORGET - Beliefs should always remain normalized to be a probability distribution, that means the
  *                  sum of beliefs over all intersections and facing directions must be 1 at all times.
  * 
  *  The function receives as input pointers to three integer values, these will be used to store the estimated
  *   robot's location and facing direction. The direction is specified as:
  *   0 - UP
  *   1 - RIGHT
  *   2 - BOTTOM
  *   3 - LEFT
  * 
  *  The function's return value is 1 if localization was successful, and 0 otherwise.
  */
 
  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/

int dr = 0;

fprintf(stderr, "start\n");
int pow = 12;
  //find_street();
int tl, tr, br, bl;
tl = 0;
tr = 0;
tr = 0;
bl = 0;
int shift = 1;
BT_drive(MOTOR_A,MOTOR_D, 10);
int kb = 0;
int turn = 0;
int yel = 0;
//int tr_ang;
while(kb == 0) {
  //continue;
  //printf("tr_ang : %d\n", tr_ang);
  int num = 9;
  int max = checkColor(PORT_2, num);
  if(max == 1 && dr == 0){
    BT_drive(MOTOR_A,MOTOR_D, 12);
    dr = 1;
  }
  else if(max == 5){
    BT_all_stop(1);
    printf("rotating 180");
    rotate(180);
    
    dr = 0;
    // turn =1;
  }
  // THIS WHERE WE ROTATE

  else if(max == 4 && dr == 1){
    if(turn == 1) {sleep (1);  printf("rotate 90"); rotate(90); turn = 0; dr = 0; 
    continue;}
    // else if(turn == 2) {sleep(1); printf("rotate -90"); rotate(-90); dr = 0;
    // continue;}

      sleep(1);
      scan_intersection(&tl, &tr, &br, &bl);
      printf("tl: %d, tr: %d, br: %d, bl: %d\n", tl, tr, br, bl);
      double tot = 0;      
      if (shift == 1) {
      for (int j=0; j<sy; j++)
          for (int i=0; i<sx; i++)
          {
            int b = i + j*sx;
            if(i < sx) beliefs[b][3] = beliefs[b+1][3];
            if(i > 0) beliefs[b][1] = beliefs[b-1][1];
            if(j < sy) beliefs[b][0] = beliefs[b+sx][0];
            if(j > 0) beliefs[b][2] = beliefs[b-sx][2];
          }
      for (int j=0; j<sy; j++)
          for (int i=0; i<sx; i++)
          {
            int b = i + j*sx;
            if(i == sx-1) beliefs[b][3] = 0.001;
            if(i == 0) beliefs[b][1] = 0.001;
            if(j == sy-1) beliefs[b][0] = 0.001;
            if(j == 0) beliefs[b][2] = 0.001;
          }
      }
      shift = 1;
      for (int j=0; j<sy; j++)
          for (int i=0; i<sx; i++)
          {
            int b = i + j*sx;
            // Update beliefs based on sensor readings and map data
            double match_prob = 0.95;
            double mismatch_prob = 0.05;

            if(map[b][0] == tl && map[b][1] == tr && map[b][2] == br && map[b][3] == bl){
              beliefs[b][0] *= match_prob;
              beliefs[b][1] *= mismatch_prob;
              beliefs[b][2] *= mismatch_prob;
              beliefs[b][3] *= mismatch_prob;
            }
            else if(map[b][1] == tl && map[b][2] == tr && map[b][3] == br && map[b][0] == bl){
              beliefs[b][0] *= mismatch_prob;
              beliefs[b][1] *= match_prob;
              beliefs[b][2] *= mismatch_prob;
              beliefs[b][3] *= mismatch_prob;
            }
            else if(map[b][2] == tl && map[b][3] == tr && map[b][0] == br && map[b][1] == bl){
              beliefs[b][0] *= mismatch_prob;
              beliefs[b][1] *= mismatch_prob;
              beliefs[b][2] *= match_prob;
              beliefs[b][3] *= mismatch_prob;
            }
            else if(map[b][3] == tl && map[b][0] == tr && map[b][1] == br && map[b][2] == bl){
              beliefs[b][0] *= mismatch_prob;
              beliefs[b][1] *= mismatch_prob;
              beliefs[b][2] *= mismatch_prob;
              beliefs[b][3] *= match_prob;
            }
            else{
              beliefs[b][0] *= mismatch_prob;
              beliefs[b][1] *= mismatch_prob;
              beliefs[b][2] *= mismatch_prob;
              beliefs[b][3] *= mismatch_prob;
            }

          // implement shift up left down right

            tot += beliefs[b][0] + beliefs[b][1] + beliefs[b][2] + beliefs[b][3];
            
          }
      

      for (int j=0; j<sy; j++)
          for (int i=0; i<sx; i++)
          {
            int b = i + j*sx;
            for(int t = 0; t < 4; t++){
              beliefs[b][t] = beliefs[b][t]/tot;
              if(beliefs[b][t] >= 0.75){
                *(robot_x) = i;
                *(robot_y) = j;
                *direction = t;
                printf("robot_x: %d, robot_y: %d, direction: %d\n", *robot_x, *robot_y, *direction);
                return 1;
              }

            }
          printf("%d : [0] : %f\t [1] : %f\t [2] : %f\t : [3] : %f\n", b, beliefs[b][0], beliefs[b][1], beliefs[b][2], beliefs[b][3]);      
    
          }
    dr = 0;
  }
  else if(max != 1){
    find_street();
    printf("max : %d\n", max);
    fprintf(stderr, "check\n");
    dr = 0;
  }
  else if(dr == 0){
    BT_drive(MOTOR_A,MOTOR_D, -4);
  }
  // printf("max: %d\n", max);
}

 // Return an invalid location/direction and notify that localization was unsuccessful (you will delete this and replace it
 // with your code).
 return(0);
}

int go_to_target(int robot_x, int robot_y, int direction, int target_x, int target_y)
{
 /*
  * This function is called once localization has been successful, it performs the actions required to take the robot
  * from its current location to the specified target location. 
  *
  * You have to write the code required to carry out this task - once again, you can use the function headers provided, or
  * write your own code to control the bot, but document your process carefully in the comments below so your TA can easily
  * understand how everything works.
  *
  * Your code should be able to determine if the robot has gotten lost (or if localization was incorrect), and your bot
  * should be able to recover.
  * 
  * Inputs - The robot's current location x,y (the intersection coordinates, not image pixel coordinates)
  *          The target's intersection location
  * 
  * Return values: 1 if successful (the bot reached its target destination), 0 otherwise
  */   

  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/
  int dir = direction;
  printf("robot_x: %d, robot_y: %d, target_x: %d, target_y: %d\n", robot_x, robot_y, target_x, target_y);
  printf("robot_x: %d, target_x: %d", robot_x, target_x);
  if (target_x > robot_x) {
    BT_all_stop(1);
    if (dir == 0) rotate(270);
    else if (dir == 2) rotate(90);
    else if (dir == 3) rotate(180);
    dir = 1;
    int cp_x = robot_x;
    int dr = 0;
    while (cp_x < target_x) {
      printf("cp_x: %d\n", cp_x);
      int mx = checkColor(PORT_3, 9);
      if (mx == 1 && dr == 0) {
        BT_drive(MOTOR_A, MOTOR_D, 10);
        dr = 1;
      } else if (mx == 4 && dr == 1) {
        sleep(1);
        // int tl, tr, bl, br;
        // scan_intersection(&tl, &tr, &bl, &br);
        // int b = target_y * sx + cp_x;
        // if (map[b][1] != tl || map[b][2] != tr || map[b][3] != br || map[b][0] != bl) return 0;
        cp_x += 1;
        dr = 0;
      } else if (mx == 5) return 0;
      else if (mx != 1) {find_street(); dr = 0;}
    }
  } else if (target_x < robot_x) {
    if (dir == 0) rotate(90);
    else if (dir == 1) rotate(180);
    else if (dir == 2) rotate(270);
    dir = 3;
    int cp_x = robot_x;
    int dr = 0;
    while (cp_x > target_x) {
      printf("cp_x: %d\n", cp_x);
      int mx = checkColor(PORT_3, 9);
      if (mx == 1 && dr == 0) {
        BT_drive(MOTOR_A, MOTOR_D, 10);
        dr = 1;
      } else if (mx == 4 && dr == 1) {
        sleep(1);
        // int tl, tr, bl, br;
        // scan_intersection(&tl, &tr, &bl, &br);
        // int b = target_y * sx + cp_x;
        // if (map[b][3] != tl || map[b][0] != tr || map[b][1] != br || map[b][2] != bl) return 0;
        cp_x -= 1;
        dr = 0;
      } else if (mx == 5) return 0;
      else if (mx != 1) {find_street();  dr = 0;}
    }
  }
  if (target_y < robot_y) {
    if (dir == 1) rotate(90);
    else if (dir == 2) rotate(180);
    else if (dir == 3) rotate(270);
    dir = 0;
    int cp_y = robot_y;
    int dr = 0;
    while (cp_y > target_y) {
      printf("cp_y: %d\n", cp_y);
      int mx = checkColor(PORT_3, 9);
      if (mx == 1 && dr == 0) {
        BT_drive(MOTOR_A, MOTOR_D, 10);
        dr = 1;
      } else if (mx == 4 && dr == 1) {
        sleep(1);
        BT_all_stop(1);
        // int tl, tr, bl, br;
        // scan_intersection(&tl, &tr, &bl, &br);
        // int b = cp_y * sx + robot_x;
        // if (map[b][0] != tl || map[b][1] != tr || map[b][2] != br || map[b][3] != bl) return 0;
        cp_y -= 1;
        dr = 0;
      } else if (mx == 5) return 0;
      else if (mx != 1) {find_street(); dr = 0;}
    }
  } else if (target_y > robot_y) {
    if (dir == 0) rotate(180);
    else if (dir == 1) rotate(90);
    else if (dir == 3) rotate(270);
    dir = 2;
    int cp_y = robot_y;
    int dr = 0;
    while (cp_y < target_y) {
      printf("cp_y: %d\n", cp_y);
      int mx = checkColor(PORT_3, 9);
      if (mx == 1 && dr == 0) {
        BT_drive(MOTOR_A, MOTOR_D, 10);
        dr = 1;
      } else if (mx == 4 && dr == 1) {
        sleep(1);
        // int tl, tr, bl, br;
        // scan_intersection(&tl, &tr, &bl, &br);
        // int b = cp_y * sx + robot_x;
        // if (map[b][2] != tl || map[b][3] != tr || map[b][0] != br || map[b][1] != bl) return 0;
        cp_y += 1;
        dr = 0;
      } else if (mx == 5) return 0;
      else if (mx != 1) {find_street(); dr = 0;}
    }
  }
  return 1;



if (target_y < robot_y) {
  if (dir == 1) rotate(-90);
  else if (dir == 2) rotate(180);
  else if (dir == 3) rotate(90);
  dir = 0;
  int cp_y = robot_y;
  int dr = 0;
  while (cp_y > target_y) {
    int mx = checkColor(PORT_3, 9);
    if (mx == 1 && dr == 0) {
      BT_drive(MOTOR_A, MOTOR_D, 10);
      dr = 1;
    } else if (mx == 4 && dr == 1) {
      printf("sleeping\n");
      sleep(1);
      BT_all_stop(1);
      // int tl, tr, bl, br;
      // scan_intersection(&tl, &tr, &bl, &br);
      // int b = cp_y * sx + robot_x;
      // if (map[b][0] != tl || map[b][1] != tr || map[b][2] != br || map[b][3] != bl) return 0;
      cp_y -= 1;
      dr = 0;
      printf("cp_y: %d\n", cp_y);
    } else if (mx == 5) return 0;
    else if (mx != 1) {find_street(); dr = 0;}
  }
} else if (target_y > robot_y) {
  if (dir == 0) rotate(180);
  else if (dir == 1) rotate(90);
  else if (dir == 3) rotate(-90);
  dir = 2;
  int cp_y = robot_y;
  int dr = 0;
  while (cp_y < target_y) {
    int mx = checkColor(PORT_3, 9);
    if (mx == 1 && dr == 0) {
      BT_drive(MOTOR_A, MOTOR_D, 10);
      dr = 1;
    } else if (mx == 4 && dr == 1) {
      sleep(1);
      // int tl, tr, bl, br;
      // scan_intersection(&tl, &tr, &bl, &br);
      // int b = cp_y * sx + robot_x;
      // if (map[b][2] != tl || map[b][3] != tr || map[b][0] != br || map[b][1] != bl) return 0;
      cp_y += 1;
      dr = 0;
    } else if (mx == 5) return 0;
    else if (mx != 1) {find_street(); dr = 0;}
  }
}
BT_all_stop(1);
printf("y is done! \n");
printf("robot_x: %d \n", robot_x);
printf("target_x: %d \n", target_x);
if (target_x > robot_x) {
  printf("turn right \n");
  BT_all_stop(1);
  if (dir == 0) {printf("turnnn \n"); rotate(-90);}
  else if (dir == 2) rotate(-90);
  else if (dir == 3) rotate(180);
  dir = 1;
  int cp_x = robot_x;
  int dr = 0;
  while (cp_x < target_x) {
    int mx = checkColor(PORT_3, 9);
    if (mx == 1 && dr == 0) {
      BT_drive(MOTOR_A, MOTOR_D, 10);
      dr = 1;
    } else if (mx == 4 && dr == 1) {
      sleep(1);
      int tl, tr, bl, br;
      scan_intersection(&tl, &tr, &bl, &br);
      int b = target_y * sx + cp_x;
      if (map[b][1] != tl || map[b][2] != tr || map[b][3] != br || map[b][0] != bl) return 0;
      cp_x += 1;
      dr = 0;
    } else if (mx == 5) return 0;
    else if (mx != 1) {find_street(); dr = 0;}
  }
} else if (target_x < robot_x) {
  if (dir == 0) rotate(-90);
  else if (dir == 1) rotate(180);
  else if (dir == 2) rotate(90);
  dir = 3;
  int cp_x = robot_x;
  int dr = 0;
  while (cp_x > target_x) {
    int mx = checkColor(PORT_3, 9);
    if (mx == 1 && dr == 0) {
      BT_drive(MOTOR_A, MOTOR_D, 10);
      dr = 1;
    } else if (mx == 4 && dr == 1) {
      sleep(1);
      int tl, tr, bl, br;
      scan_intersection(&tl, &tr, &bl, &br);
      int b = target_y * sx + cp_x;
      if (map[b][3] != tl || map[b][0] != tr || map[b][1] != br || map[b][2] != bl) return 0;
      cp_x -= 1;
      dr = 0;
    } else if (mx == 5) return 0;
    else if (mx != 1) {find_street(); dr = 0;}
  }
}
return 1;
}

void calibrate_sensor(void)
{
 /*
  * This function is called when the program is started with -1  -1 for the target location. 
  *
  * You DO NOT NEED TO IMPLEMENT ANYTHING HERE - but it is strongly recommended as good calibration will make sensor
  * readings more reliable and will make your code more resistent to changes in illumination, map quality, or battery
  * level.
  * 
  * The principle is - Your code should allow you to sample the different colours in the map, and store representative
  * values that will help you figure out what colours the sensor is reading given the current conditions.
  * 
  * Inputs - None
  * Return values - None - your code has to save the calibration information to a file, for later use (see in main())
  * 
  * How to do this part is up to you, but feel free to talk with your TA and instructor about it!
  */   

  /************************************************************************************************************************
   *   OIPTIONAL TO DO  -   Complete this function
   ***********************************************************************************************************************/
  fprintf(stderr,"Calibration function called!\n");  
}

int parse_map(unsigned char *map_img, int rx, int ry)
{
 /*
   This function takes an input image map array, and two integers that specify the image size.
   It attempts to parse this image into a representation of the map in the image. The size
   and resolution of the map image should not affect the parsing (i.e. you can make your own
   maps without worrying about the exact position of intersections, roads, buildings, etc.).

   However, this function requires:
   
   * White background for the image  [255 255 255]
   * Red borders around the map  [255 0 0]
   * Black roads  [0 0 0]
   * Yellow intersections  [255 255 0]
   * Buildings that are pure green [0 255 0], pure blue [0 0 255], or white [255 255 255]
   (any other colour values are ignored - so you can add markings if you like, those 
    will not affect parsing)

   The image must be a properly formated .ppm image, see readPPMimage below for details of
   the format. The GIMP image editor saves properly formatted .ppm images, as does the
   imagemagick image processing suite.
   
   The map representation is read into the map array, with each row in the array corrsponding
   to one intersection, in raster order, that is, for a map with k intersections along its width:
   
    (row index for the intersection)
    
    0     1     2    3 ......   k-1
    
    k    k+1   k+2  ........    
    
    Each row will then contain the colour values for buildings around the intersection 
    clockwise from top-left, that is
    
    
    top-left               top-right
            
            intersection
    
    bottom-left           bottom-right
    
    So, for the first intersection (at row 0 in the map array)
    map[0][0] <---- colour for the top-left building
    map[0][1] <---- colour for the top-right building
    map[0][2] <---- colour for the bottom-right building
    map[0][3] <---- colour for the bottom-left building
    
    Color values for map locations are defined as follows (this agrees with what the
    EV3 sensor returns in indexed-colour-reading mode):
    
    1 -  Black
    2 -  Blue
    3 -  Green
    4 -  Yellow
    5 -  Red
    6 -  White
    
    If you find a 0, that means you're trying to access an intersection that is not on the
    map! Also note that in practice, because of how the map is defined, you should find
    only Green, Blue, or White around a given intersection.
    
    The map size (the number of intersections along the horizontal and vertical directions) is
    updated and left in the global variables sx and sy.

    Feel free to create your own maps for testing (you'll have to print them to a reasonable
    size to use with your bot).
    
 */    
 
 int last3[3];
 int x,y;
 unsigned char R,G,B;
 int ix,iy;
 int bx,by,dx,dy,wx,wy;         // Intersection geometry parameters
 int tgl;
 int idx;
 
 ix=iy=0;       // Index to identify the current intersection
 
 // Determine the spacing and size of intersections in the map
 tgl=0;
 for (int i=0; i<rx; i++)
 {
  for (int j=0; j<ry; j++)
  {
   R=*(map_img+((i+(j*rx))*3));
   G=*(map_img+((i+(j*rx))*3)+1);
   B=*(map_img+((i+(j*rx))*3)+2);
   if (R==255&&G==255&&B==0)
   {
    // First intersection, top-left pixel. Scan right to find width and spacing
    bx=i;           // Anchor for intersection locations
    by=j;
    for (int k=i; k<rx; k++)        // Find width and horizontal distance to next intersection
    {
     R=*(map_img+((k+(by*rx))*3));
     G=*(map_img+((k+(by*rx))*3)+1);
     B=*(map_img+((k+(by*rx))*3)+2);
     if (tgl==0&&(R!=255||G!=255||B!=0))
     {
      tgl=1;
      wx=k-i;
     }
     if (tgl==1&&R==255&&G==255&&B==0)
     {
      tgl=2;
      dx=k-i;
     }
    }
    for (int k=j; k<ry; k++)        // Find height and vertical distance to next intersection
    {
     R=*(map_img+((bx+(k*rx))*3));
     G=*(map_img+((bx+(k*rx))*3)+1);
     B=*(map_img+((bx+(k*rx))*3)+2);
     if (tgl==2&&(R!=255||G!=255||B!=0))
     {
      tgl=3;
      wy=k-j;
     }
     if (tgl==3&&R==255&&G==255&&B==0)
     {
      tgl=4;
      dy=k-j;
     }
    }
    
    if (tgl!=4)
    {
     fprintf(stderr,"Unable to determine intersection geometry!\n");
     return(0);
    }
    else break;
   }
  }
  if (tgl==4) break;
 }
  fprintf(stderr,"Intersection parameters: base_x=%d, base_y=%d, width=%d, height=%d, horiz_distance=%d, vertical_distance=%d\n",bx,by,wx,wy,dx,dy);

  sx=0;
  for (int i=bx+(wx/2);i<rx;i+=dx)
  {
   R=*(map_img+((i+(by*rx))*3));
   G=*(map_img+((i+(by*rx))*3)+1);
   B=*(map_img+((i+(by*rx))*3)+2);
   if (R==255&&G==255&&B==0) sx++;
  }

  sy=0;
  for (int j=by+(wy/2);j<ry;j+=dy)
  {
   R=*(map_img+((bx+(j*rx))*3));
   G=*(map_img+((bx+(j*rx))*3)+1);
   B=*(map_img+((bx+(j*rx))*3)+2);
   if (R==255&&G==255&&B==0) sy++;
  }
  
  fprintf(stderr,"Map size: Number of horizontal intersections=%d, number of vertical intersections=%d\n",sx,sy);

  // Scan for building colours around each intersection
  idx=0;
  for (int j=0; j<sy; j++)
   for (int i=0; i<sx; i++)
   {
    x=bx+(i*dx)+(wx/2);
    y=by+(j*dy)+(wy/2);
    
    fprintf(stderr,"Intersection location: %d, %d\n",x,y);
    // Top-left
    x-=wx;
    y-=wy;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][0]=3;
    else if (R==0&&G==0&&B==255) map[idx][0]=2;
    else if (R==255&&G==255&&B==255) map[idx][0]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Top-Left RGB=%d,%d,%d\n",i,j,R,G,B);

    // Top-right
    x+=2*wx;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][1]=3;
    else if (R==0&&G==0&&B==255) map[idx][1]=2;
    else if (R==255&&G==255&&B==255) map[idx][1]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Top-Right RGB=%d,%d,%d\n",i,j,R,G,B);

    // Bottom-right
    y+=2*wy;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][2]=3;
    else if (R==0&&G==0&&B==255) map[idx][2]=2;
    else if (R==255&&G==255&&B==255) map[idx][2]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Bottom-Right RGB=%d,%d,%d\n",i,j,R,G,B);
    
    // Bottom-left
    x-=2*wx;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][3]=3;
    else if (R==0&&G==0&&B==255) map[idx][3]=2;
    else if (R==255&&G==255&&B==255) map[idx][3]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Bottom-Left RGB=%d,%d,%d\n",i,j,R,G,B);
    
    fprintf(stderr,"Colours for this intersection: %d, %d, %d, %d\n",map[idx][0],map[idx][1],map[idx][2],map[idx][3]);
    
    idx++;
   }

 return(1);  
}

unsigned char *readPPMimage(const char *filename, int *rx, int *ry)
{
 // Reads an image from a .ppm file. A .ppm file is a very simple image representation
 // format with a text header followed by the binary RGB data at 24bits per pixel.
 // The header has the following form:
 //
 // P6
 // # One or more comment lines preceded by '#'
 // 340 200
 // 255
 //
 // The first line 'P6' is the .ppm format identifier, this is followed by one or more
 // lines with comments, typically used to inidicate which program generated the
 // .ppm file.
 // After the comments, a line with two integer values specifies the image resolution
 // as number of pixels in x and number of pixels in y.
 // The final line of the header stores the maximum value for pixels in the image,
 // usually 255.
 // After this last header line, binary data stores the RGB values for each pixel
 // in row-major order. Each pixel requires 3 bytes ordered R, G, and B.
 //
 // NOTE: Windows file handling is rather crotchetty. You may have to change the
 //       way this file is accessed if the images are being corrupted on read
 //       on Windows.
 //

 FILE *f;
 unsigned char *im;
 char line[1024];
 int i;
 unsigned char *tmp;
 double *fRGB;

 im=NULL;
 f=fopen(filename,"rb+");
 if (f==NULL)
 {
  fprintf(stderr,"Unable to open file %s for reading, please check name and path\n",filename);
  return(NULL);
 }
 fgets(&line[0],1000,f);
 if (strcmp(&line[0],"P6\n")!=0)
 {
  fprintf(stderr,"Wrong file format, not a .ppm file or header end-of-line characters missing\n");
  fclose(f);
  return(NULL);
 }
 fprintf(stderr,"%s\n",line);
 // Skip over comments
 fgets(&line[0],511,f);
 while (line[0]=='#')
 {
  fprintf(stderr,"%s",line);
  fgets(&line[0],511,f);
 }
 sscanf(&line[0],"%d %d\n",rx,ry);                  // Read image size
 fprintf(stderr,"nx=%d, ny=%d\n\n",*rx,*ry);

 fgets(&line[0],9,f);  	                // Read the remaining header line
 fprintf(stderr,"%s\n",line);
 im=(unsigned char *)calloc((*rx)*(*ry)*3,sizeof(unsigned char));
 if (im==NULL)
 {
  fprintf(stderr,"Out of memory allocating space for image\n");
  fclose(f);
  return(NULL);
 }
 fread(im,(*rx)*(*ry)*3*sizeof(unsigned char),1,f);
 fclose(f);

 return(im);    
}