#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <image_server/ImageAction.h>
 
//action完成后调用此函数
void doneCb(const actionlib::SimpleClientGoalState& state,
            const image_server::ImageResultConstPtr& result)
{
    ROS_INFO("Finsh Drawing!");
    //任务完成就关闭节点
    //ros::shutdown();
}
 
//action的目标任务发送给server且开始执行时，调用此函数
void activeCb()
{
   ROS_INFO("Goal is active! Begin to draw circles.");
}
 
//action任务在执行过程中，server对过程有反馈则调用此函数
void feedbackCb(const image_server::ImageFeedbackConstPtr& feedback)
{
    //将服务器的反馈输出（读到第几页书）
    ROS_INFO("Drawing circle: %d", feedback->process_seconds);
}
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "drawing_client");
    
    //创建一个action的client，指定action名称为”draw”
    actionlib::SimpleActionClient<image_server::ImageAction> client("draw", true);

    ROS_INFO("Waiting for action server to start");
    client.waitForServer();
    ROS_INFO("Action server started");
    
    // create a goal: draw 20 circles
    image_server::ImageGoal goal;
    goal.total_seconds = 20;
    
    //把action的任务目标发送给服务器，绑定上面定义的各种回调函数
    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ros::spin();

    return 0;
}