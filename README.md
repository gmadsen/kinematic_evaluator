
KinEval
=======

The Kinematic Evaluator (KinEval) is a package containing a collection of HTML5/Javascript implementations for teaching robot kinematics, control, decision making, and dynamics.

To see kineval in action, follow [Kinematic Evaluator](https://gmadsen.github.io/kinematic_evaluator/home.html)

or open home.html in a web browser.  Firefox 29.0 through 41.0 works for sure.  Chrome and Opera will throw security errors when loading local files from JavaScript (which feels like forcing people into the cloud).  One method around this issue is to serve home.html from a web server.  If python is available on your system, this can be done by running python's SimpleHTTPServer: 

python -m SimpleHTTPServer

and loading the file from the following URL:

http://localhost:8000/home.html

or, alternatively, NodeJS's (https://nodejs.org/) simple-server from your working directory:

npm install simple-server
node simple-server

and loading the file from the following URL:

http://localhost:3000/

## Controls (approximate description):

- W/A/S/D - move forward/backward and turn right/left
- Q/R - strafe left right
- U/I - control current joint to rotate forward/backward
- H/J/K/L - switch control to parent/child/sibling joint
- O - servo all joints based on seconds of the system clock
- P - perform inverse kinematics iteration (hold down for continual motion)
- R/F - move inverse kinematics target up/down
- M - execute RRT planner
- N/B - set robot configuration to next/previous configuration in generated RRT plan

## Robots and Worlds:

The subdirectories "robots" and "worlds" contains descriptions of various robots and worlds as js files that can be used with KinEval.  These robots and worlds can be included by including the appropriate js files in home.html.  For example:

<script src="robots/robot_br2.js"></script> 
<script src="worlds/world_local_minima.js"></script> 




