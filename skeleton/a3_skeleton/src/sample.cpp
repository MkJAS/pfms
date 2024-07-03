#include "sample.h"

/**
 * This sample code is provided to illustrate
 * - Subscribing to topics (
 * - Respond to an incoming service call
 * - Running a seperate thread
 */

Sample::Sample(ros::NodeHandle nh)
    : nh_(nh)
{
  //Example of subscribing to odometry
  sub1_ = nh_.subscribe("robot_0/odom", 1000, &Sample::odomCallback,this);
  //Subscribing to laser
  sub2_ = nh_.subscribe("robot_0/base_scan", 10, &Sample::laserCallback,this);
  //Subscribing to occupnacy grid
  sub3_ = nh_.subscribe("local_map/local_map", 1, &Sample::occupancyGridCallback,this);


  //Example of publishing, we publish markers here
  viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker",3,false);

  // Below is how to get parameters from command line, on command line they need to be _param:=value
  // For example _example:=0.1
  // ROS will obtain the configuration from command line, or assign a default value 0.1
  ros::NodeHandle pn("~");
  double example;
  pn.param<double>("example", example, 0.1);
  //ROS_INFO_STREAM("example:" << example);

  exampleBool_=false;// We will use this atomic bool to let us know when we have new data

  // STUDENTS !!! - change below to be consistent with your project specification
  //Allowing an incoming service via name: /my_service_name, and calling myExample function
  service_ = nh_.advertiseService("my_service_name", &Sample::myExample,this);

}

Sample::~Sample()
{

}

//! Change this depending on your service for the project, here we use FaceGoal (which is not used
//! in any of the projects, check the request and responce fields
bool Sample::myExample(project_setup::FaceGoal::Request  &req,
             project_setup::FaceGoal::Response &res)
{
  //When an incoming call arrives, we can respond to it here
  ROS_INFO_STREAM("request: [x,y]=[" << req.pose.x << "," << req.pose.y << "]");


  return true; //We ALWAYS return true indicate the service call sucseeded
}

// A callback for odometry
void Sample::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    // We store a copy of the pose and lock a mutex when updating
    std::unique_lock<std::mutex> lck (poseDataBuffer_.mtx);
    poseDataBuffer_.pose = msg->pose.pose;

}


void Sample::occupancyGridCallback(const nav_msgs::OccupancyGridPtr& msg)
{

  std::unique_lock<std::mutex> lck (ogMapBuffer_.mtx);
  ogMapBuffer_.grid = *msg;

}

void Sample::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
 // Not doign anything here for now
}


void Sample::seperateThread() {

  ros::Duration(2.0).sleep(); // sleep for two seconds

  /**
    * The below loop runs until ros is shutdown
    */

    //! For example, let's run the thread at 1/10 Hz
    ros::Rate rate_limiter(1/10.0);
    while (ros::ok()) {

        //For example, let's get the Grid
        std::unique_lock<std::mutex> lck1 (ogMapBuffer_.mtx);
        nav_msgs::OccupancyGrid grid = ogMapBuffer_.grid;
        // We plan to use an object of our class here
        GridProcessing gridProcessing(grid);
        lck1.unlock();

        //For example, let's get Odo
        std::unique_lock<std::mutex> lck2 (poseDataBuffer_.mtx);
        geometry_msgs::Pose pose = poseDataBuffer_.pose;
        lck2.unlock();

        geometry_msgs::Point test; // This is our test pooint in world coordinates
        test.x=1;
        test.y=0;

        ROS_INFO_STREAM("Example checking [x,y]=[" << test.x << "," << test.y << "]" <<
                        " to [x,y]=[" << pose.position.x << "," << pose.position.y << "]" );
        //Avoid using std::cout
        //std::cout << "Example checking x,y=" << local.x << "," << local.y;

        //If the points we want are in global coordinates, and the ogmap is local
        // (root at 0,0) so we have to
        //convert into robot centric coordinates
        geometry_msgs::Point testLocal;
        testLocal.x = test.x + pose.position.x;
        testLocal.y = test.y + pose.position.y;

        // The robot is at 0,0 in the OccupancyGrid
        geometry_msgs::Point zero;
        zero.x=0;
        zero.y=0;


        // Here we call a function of our gridProcessing object, passing required parameters
        bool reachable = gridProcessing.checkConnectivity(zero,testLocal);

        //Let's print to screen about the status of reaching the point
        //Using two verbosity levels
        if(reachable){
          ROS_INFO("Point reachable");
        }
        else{
           ROS_WARN("Point not reachable");
        }

        //We publish the point to screen using a function
        std_msgs::ColorRGBA color;
        color.a=0.5;//a is alpha - transparency 0.5 is 50%;
        color.r=0;
        color.g=1.0; // it will be green
        color.b=0;

        int marker_counter=0;
        //Let's also publish the markerArray, we create a few markers
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(produceMarkerSphere(zero,color,marker_counter));
        viz_pub_.publish(marker_array);

        rate_limiter.sleep();

    }
}


visualization_msgs::Marker Sample::produceMarkerSphere(geometry_msgs::Point point,
                                                            std_msgs::ColorRGBA color,
                                                          int& id ){

  visualization_msgs::Marker marker;

  //We need to set the frame
  // Set the frame ID and time stamp.
  marker.header.frame_id = "world";
  //single_marker_person.header.stamp = ros::Time();
  marker.header.stamp = ros::Time::now();


  //We set lifetime (it will dissapear in this many seconds)
  marker.lifetime = ros::Duration(10.0);
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "test";
  marker.id = id++;

  // The marker type, we use a cylinder in this example
  marker.type = visualization_msgs::Marker::CYLINDER;

  // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.position.z = 0.0;

  //Orientation, we are going to leave it as 0,0,0,1 for quaternion,
  //For an arrow you copudl consider passing yaw or quaternion for it
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 0.5x0.5x0.5 here means 0.5m side
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  //Alpha is stransparency (50% transparent)
  marker.color = color;

  return marker;

}
