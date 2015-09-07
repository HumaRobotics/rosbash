#!/bin/bash
 


# ROS NETWORK CONFIGURATION

rosnetwork() { 
    env | egrep "ROS_MASTER_URI|ROS_IP|ROS_HOSTNAME" 
}

rosmaster() {    
    export ROS_MASTER_URI=http://$1:11311
    rosnetwork
    rosprompt
}

rosip() {    
    export ROS_IP=$1
    unset ROS_HOSTNAME
    rosnetwork
}

roshostname() {    
    export ROS_HOSTNAME=$1
    unset ROS_IP
    rosnetwork
}



# PROMPT AND SHELL

rosprompt() {
    MASTER=`echo $ROS_MASTER_URI  | sed 's/http:\/\///' | sed 's/:11311//'`

    # Extract top folder last componentent
    ROSPATHNAME=`(roscd;cd ..; pwd | sed -e "s/.*\///g"  )`
    export PS1='\[\033[0;31m\]$ROS_DISTRO \[\033[0;34m\]$ROSPATHNAME\[\033[0;32m\]@$MASTER\[\033[0m\]:\[\033[0;36m\]\w\[\033[0m\]> '
}


# Loads child Bash environment with bashrc, prompt, and any start command as a parameter
rosshell() {
    F=`mktemp`    
    echo source ~/.bashrc >> $F
    echo $* >> $F
    echo rosprompt >> $F
    #~ echo ros >> $F
    bash --rcfile $F    
}



# ENVIRONMENT AND SOURCING

alias ros=' env | egrep "ROS_.*=|PYTHONPATH|LD_LIBRARY" '
alias indigo='rosshell source /opt/ros/indigo/setup.bash'
alias hydro='rosshell source /opt/ros/hydro/setup.bash'
alias groovy='rosshell source /opt/ros/groovy/setup.bash'
alias devel='rosshell source devel/setup.bash'
alias install='rosshell source install/setup.bash'
alias install_deps="(roscd;cd ..;rosdep install --from-paths src --ignore-src --rosdistro hydro)"

alias rosrefresh='(roscd;cd ..; rospack profile)'
alias cm='(roscd;cd ..; catkin_make)'
alias catkin_eclipse='(roscd;cd ..; catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles")'
alias pydev='python $(rospack find mk)/make_pydev_project.py'
# BAXTER SHORTCUTS

alias be='rostopic pub -1 /robot/set_super_enable std_msgs/Bool True'
alias bd='rostopic pub -1 /robot/set_super_enable std_msgs/Bool False'
#alias de='rostopic pub -1 /darwin/setCmdEnable std_msgs/Bool True'
#alias dd='rostopic pub -1 /darwin/setCmdEnable std_msgs/Bool False'


alias gkill='killall gzserver ; killall gzclient ; pkill -9 -f "python /opt/ros/" '
alias rkill='pkill -9 -f "python /opt/ros/" ; gkill'


_urdfshow() {
    roslaunch urdf_tutorial display.launch gui:=true model:=$1
}

alias urdfshow=_urdfshow
alias make-eclipse-project='cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug'
alias rosdep_indigo='rosdep install -r --from-paths src --ignore-src --rosdistro indigo -y'