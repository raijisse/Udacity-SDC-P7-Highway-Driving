# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   

## Model Documentation

### Receiving car data

`

        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // Get the size of the previous path poitns still valid
          int prev_size = previous_path_x.size();

          if(prev_size>0){
            car_s = end_path_s;
          }

`

From line 82 to 110 of main.cpp, we simply get data from the ego vehicle and we store all variables of interest, such as s, d (Frenet coordinates), x, y, speed and yaw. 

### Analyzing the surroundings
`
          // Initialize a value that will keep track of our car getting too close of other cars
          bool too_close = false;
          bool on_the_left = false;
          bool on_the_right = false;

          // For each car detected by the sensors we are going to see if
          // they are a potential infringement to our progress
          for (int i=0; i<sensor_fusion.size(); i++){

            int safety_distance = 30;
            // d coordinates
            float d = sensor_fusion[i][6];
            
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += ((double)prev_size*.02*check_speed);
            
            // Check where other cars are:
            // 1) Check if a car is in my lane
            if(d < (2+4*lane+2) && d > (2+4*lane-2)){ 
              // If it is in my lane, check the distance between the ego
              // vehicle and the other one. 
              if((check_car_s > car_s) && ((check_car_s - car_s) < safety_distance)){
                too_close = true;
                }
              }
            
            // 2) Check if a car is on the lane to my left
            else if(d < (4*lane) && d > (4*lane-4)){
              // Check if there is enough space to pass the vehicle in front of me
              if((check_car_s > (car_s-safety_distance)) && ((check_car_s < (car_s+safety_distance)))){
                on_the_left = true;
                }              
            }

            // 3) Check if a car is on the lane to my right
            else if((d > (4*lane+4)) && (d < (4*lane + 8))){
              // Check if there is enough space to pass the vehicle in front of me
              if((check_car_s > (car_s-safety_distance)) && ((check_car_s < (car_s+safety_distance)))){
                on_the_right = true;
                }
            
            }
          }

`

From line 111 to 157 of main.cpp, we mainly analyze the surroundings to know whether there are potential risks around the ego vehicle. We instantiate three variables to check whether there are cars on the lane on the right of the vehicle, on the one on the left or whether there is car ahead of us (the threshold distance is set to 30 in Frenet coordinates). Based on this information, we are going to be able to do some decision making.


### Decision making
`

          // If there is a car ahead
          if (too_close == true){
            // and that the left lane is empty, we can go the left lane
            if ((on_the_left == false) && (lane >= 1)){
              lane -= 1;
            }
            // otherwise we try to go to the right lane
            else if ((on_the_right == false) && (lane <2)){
              lane += 1;
            }
            // If no passing is possible, we reduce speed
            else {
              ref_vel -= 0.224;
            }
          }
          // Finally, if there is no car ahead, we just check that the current speed is close
          // to the target speed, otherwise we accelerate
          else {
            if((ref_vel < 49)){
              ref_vel += 0.224;
            }
          }

`

The logic implemented here is pretty simple and straight forward.
First, we check whether the car ahead is too close (thanks to the variables `too_close` instantiated before). If there is a car ahead, we try to pass through the left, provided there is no car on the next lane to the left. Otherwise, we try to see if we can pass to the right, by checking whether there is space to do so on the next lane to the right. 
If no passing options is possible, we just decrease the speed and wait for an opportunity to open. 
If there are no cars ahead, we just keep lane and make sure we are going fast enough.

### Trajectory planning

`

          // Initialize the x and y waypoints
          vector<double> xpts;
          vector<double> ypts;

          // Keep track of x, y and yaw :
          // Starting point or previous path point
          double x_ref = car_x;
          double y_ref = car_y;
          double yaw_ref = deg2rad(car_yaw);
 `
 Now that we know what action to take, we need to know how to get there and generate a smooth trajectory to this point. 
 So we will initiate a vector of x and y points where the car is going to drive. Let's see how it is done.

 `

          // If we do not have two waypoints, create a second one from the starting point

          if(prev_size < 2){
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            // The first waypoint is an estimated one based on current yaw rate (tangent trajectory)
            xpts.push_back(prev_car_x);
            // The second one is the starting (current at t=0) position
            // Hence we populate waypoints for the first iteration
            xpts.push_back(car_x);
            // Likewise for y
            ypts.push_back(prev_car_y);
            ypts.push_back(car_y);


          }
          else {
            x_ref = previous_path_x[prev_size-1];
            y_ref = previous_path_y[prev_size-1];

            double x_ref_prev = previous_path_x[prev_size-2];
            double y_ref_prev = previous_path_y[prev_size-2];
            yaw_ref = atan2(y_ref-y_ref_prev, x_ref-x_ref_prev);

            xpts.push_back(x_ref_prev);
            xpts.push_back(x_ref);

            ypts.push_back(y_ref_prev);
            ypts.push_back(y_ref);
          }
  `
  
  For the first iteration, we create two points based on the starting point and an generated one before that. Otherwise, we add the two last registered path points.


`

          // In Frenet coordinates, add 30m evenly spaced waypoints
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          xpts.push_back(next_wp0[0]);
          xpts.push_back(next_wp1[0]);
          xpts.push_back(next_wp2[0]);

          ypts.push_back(next_wp0[1]);
          ypts.push_back(next_wp1[1]);
          ypts.push_back(next_wp2[1]);

          for (int i = 0; i < xpts.size(); ++i) {
            
            // Shift in rotation to make the maths easier
            double x_shift = xpts[i]-x_ref;
            double y_shift = ypts[i]-y_ref;

            xpts[i] = (x_shift * cos(0-yaw_ref) - y_shift * sin(0-yaw_ref));
            ypts[i] = (x_shift * sin(0-yaw_ref) + y_shift * cos(0-yaw_ref));
          }
`

Then, we generate 3 points, one at s+30m, one at s+60 and one at s+90 points, that will show us the way we have to drive (including lane).
So, in the end, we have a vector of previous waypoints, and three new waypoints at s+30,s+60 and s+90.


### Smoothing
Now that we have the waypoints, we have to make sure that the trajectory 
is smooth enough not to generate excessive jerk or unwanted behavior. To do so, we use smoothing splines from this [library](https://kluge.in-chemnitz.de/opensource/spline/). Smoothing splines is an interpolation techniques that will estimate a curve from a set of (x,y) points. The estimated curve will pass through all the points, while ensuring a certain degree of smoothness.

Once we have done that, we generate our trajectory (`next_x_vals`). In order to do so, we first use the previous waypoints to prevent re-generating waypoints that should still be valid, and that could generate excessive jerk if we regenerate all points each time. Hence, we will use the smoothed new trajectory only to complete the previous points vector, until our vector reach a size of 50 and represents a distance of 30m.

`
          tk::spline s;

          // Set xpts and ypts to the spline
          s.set_points(xpts, ypts);

          // Initialize the next x and y waypoints
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Append previous paths points
          for(int i=0; i < previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0; //m
          double target_y = s(target_x); //m
          double target_dist = sqrt((target_x*target_x) + (target_y*target_y));

          double x_add_on = 0;

          // Fill up the rest of our waypaths points
          for(int i = 1; i <= 50 - previous_path_x.size(); i++){

            //Compute the number of points we have to add
            double N = (target_dist / (0.02*ref_vel/2.24));
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double ref_x = x_point;
            double ref_y = y_point;

            //rotate back to normal
            x_point = (ref_x *cos(yaw_ref) - ref_y*sin(yaw_ref));
            y_point = (ref_x *sin(yaw_ref) + ref_y*cos(yaw_ref));
            
            x_point += x_ref;
            y_point += y_ref;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }

`

And there we have it : our new trajectory and decision making that sould be able to drive us along the highway withou collisions, unwanted jerk or excessive acceleration.


### Limitations

By lack of time, the implementation here is a very simple one that does the job. However, there are many improvements that could be done:
- model more precisely the behavior of other vehicles to be able to take better informed decisions regarding the behavior of other vehicles
- Optimize by implementing a cost function that would take much more parameters than what we have done in this implementation such as other driver behaviors, planning ahead, estimated speed along the different options etc.
For instance a behavior to correct could be to optimize speed as I noticed that the car often brakes too much before accelerating again, and it would probably be more efficient to adapt speed more smoothly.