Recommendation: add the following alias and use it from every new terminal.
> `alias cm="cd ~/catkin_ws ; catkin_make ; source devel/setup.bash`

Make sure to start a ROS master.
> `$ roscore`

Start CoppeliaSim from a terminal.
> `$ coppeliaSim.sh`

Look through the logs for `plugin 'ROSInterface': load succeeded.` and `plugin 'RemoteApi': load succeeded.`.

![1](img/Screenshot1.png)

Open the scene [coppeliasim_scene.ttt](../sim/coppeliasim_scene.ttt). The scene hierarchy display show the following:

![2](img/Screenshot2.png)

(The child script attached to the Hanoy object is [this non-threaded LUA](../sim/nonthreaded_childscript_hanoy.lua).)

Start the simulation (it is recommended to use the Newton engine and have real-time simulations on).

![3](img/Screenshot3.png)

The integrated output should state that the remote API is started:

![4](img/Screenshot4.png)

When that is the case, the nodes from the `hanoying_back` package can be started.
> `$ roslaunch hanoying_back back.launch`

![5](img/Screenshot5.png)

Start the GUI using the following line in a dedicated terminal:
> `$ rosrun hanoying_front gui.py`

This node should start a R-Viz window. Once it is fully started, look for the Add button in the bottom left.

![6](img/Screenshot6.png)

In the dialog that open, switch to the tab "By topic" and choose "Markers".

![7](img/Screenshot7.png)

The graphical view should now show the 3 towers (cyan beams), the 3 disks (red, green and blue cylinders) and the decision from the decision_system (`/decisys/decision`) in magenta.

![8](img/Screenshot8.png)

From a new terminal, start a simplified control node.
> `$ rosrun hanoying_front ctrl.py`

![9](img/Screenshot9.png)
