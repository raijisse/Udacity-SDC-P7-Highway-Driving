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

### Smoothing


### Limitations

By lack of time, the implementation here is a very simple one that does the job. There are many improvements that can be done:
- modelize more precisely the behavior of other vehicles
- Optimize by implementing a cost function that would take much more parameters than what we have done in this implementation