/**
 * Software License Agreement (MIT License)
 *
 * @copyright Copyright (c) 2015 DENSO WAVE INCORPORATED
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <unistd.h>
#include <ros/ros.h>
#include <iostream>
#include <mutex>
#include <controller_manager/controller_manager.h>
#include "denso_robot_control/denso_robot_hw.h"

#include "std_msgs/String.h"

static int pause_flag = 0;

std::mutex m;

denso_robot_control::DensoRobotHW drobo;
ros::Time start;
controller_manager::ControllerManager *cm;

void cmdRequestCallback(const std_msgs::String::ConstPtr& msg)
{
   std::lock_guard<std::mutex> lock(m);
   if (msg->data == "pause"){
     ROS_INFO("pause control");
     pause_flag = 1;
   }else if (msg->data == "resume"){
     ROS_INFO("resume control");
     pause_flag = 0;
   }else if (msg->data == "reset"){
     ROS_INFO("reset");
     drobo.register_joint_handle();
     start = drobo.getTime();
   }else{
     ROS_INFO("Invalid command: %s", msg->data.c_str());
   }
   return;
}

/*

*/
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "denso_robot_control");
  ros::NodeHandle nh;

  //denso_robot_control::DensoRobotHW drobo;
  HRESULT hr = drobo.Initialize();
  if (nh.getNamespace() == "/cobotta"){
     drobo.init_cobotta_hand();
  }

  ros::Subscriber sub  = nh.subscribe("control_cmd", 10, cmdRequestCallback);

  if(SUCCEEDED(hr)) {
      controller_manager::ControllerManager cm(&drobo, nh);

      ros::Rate rate(1.0 / drobo.getPeriod().toSec());
      ros::AsyncSpinner spinner(1);
      spinner.start();

      //ros::Time start = drobo.getTime();
      start = drobo.getTime();
      ros::Time tm1 = drobo.getTime();
      ros::Duration dt = drobo.getPeriod();

      while(ros::ok())
      {
        ros::Time now = drobo.getTime();
        {
          std::lock_guard<std::mutex> lock(m);
          drobo.read(now, dt);
          cm.update(now, dt);

          ros::Duration diff = now - start;
          if(diff.toSec() > 5 && pause_flag == 0) {
            drobo.write(now, dt);
          } else {
            rate.sleep();
            continue;
          }

        }
        if (!drobo.is_SlaveSyncMode())
        {
          rate.sleep();
        }
//        ros::spinOnce();
      }
      spinner.stop();
  }

  return 0;
}
