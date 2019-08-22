# Oklahoma State University

## ROS Package for the Privacy Sensitive Detection on Social Robots.  

Author: Francisco Erivaldo Fernandes Junior  

## Packages:  
- Nakedness detection using Inception on Tensorflow running on GPU.  
- Robot controller.  
- Navigation package.  
- PIR location receiver.  
- Robot talks.  

## Dependencies:  
In order to run all ROS packages related to the privacy detection, you need the following packages installed in the robot:
- RosAria;  
- hokuyo_node;  
- ROS navigation stack;  
- GSCAM.  

## HOW TO RUN THESE PACKAGES:  

1. First, we need to be able to connect into our robot using SSH without the need to type a password:
    1. In your main machine, type: ssh-keygen
    2. This will generate a file called id_rsa.pub in the folder ~/.ssh/
    3. Copy this folder to your robot's computer in the folder ~/.ssh/ with the following command: ssh-copy-id USER@ROBOT_IP
    4. Add the following lines to the file ~/.bashrc in your main machine:  

        > ssh-add &>/dev/null || eval `ssh-agent` &>/dev/null  # start ssh-agent if not present  
        > ssh-add ~/.ssh/id_rsa &>/dev/null # Load key  

    5. Try to connect to your robot and see if you still need to type a password: ssh ascc@192.168.1.117
    6. If you don't need to type a password, then everything worked fine, and we can try to run the packages.

    7. In your main machine, add the public keys of the robot's computer to your main machine with the following: ssh-keyscan ROBOT_IP >> ~/.ssh/known_hosts

    8. Now, edit the file /etc/host in your main machine and add the following line:  

        > ROBOT_IP  ROBOT_HOSTNAME  
        * For example, in my case I added: 
            > 192.168.1.117 i7-nuc

    9. Now, edit the file /etc/host in your robot's computer and add the following line:  

        > MAIN_COMPUTER_IP MAIN_COMPUTER_HOSTNAME  
        * For example, in my case I added: 
            > 192.168.1.124 pc-035

    10. Remember that STEP 1 just need to be done one time! After you can log using SSH without password, you don't need to do this step again! 

3. Now, we can start the ROS nodes in the main machine and in the robot:

    1. In your main machine, type the following commands in different terminals:
        1. Start navigation stack and coarse human localization:  

            > roslaunch ascc_homebot_2dnav start_navigation_privacy.launch  

        2. OPTIONAL: If you want to see the room map and navigation informations, you can run RVIZ:  
            > rosrun rviz rviz  
            - In the rviz screen, go to File -> Open Config, and choose the file 'rviz_navigation_config.rviz' located in the 'ascc_homebot_2dnav' package folder.  

        3. Start the 3D camera and nakedness detection using GoogLeNet:  
            > roslaunch nakedness_detection_googlenet nakedness_detection.launch  

        4. Start PocketSphinx and Privacy Controller:  
            > roslaunch privacy_controller start_controller.launch  


