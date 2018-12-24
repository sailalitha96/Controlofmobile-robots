classdef PotentialField < simiam.controller.Controller

 % Authors: Yuwei Wang , Sailalitha Gollapudi , Mohit Patel 

    properties
        u_att   % attractive force
        u_rep   % repulsive force
        
        % for PID
        E_k
        e_k_1
        
        % PID gains
        Kp
        Ki
        Kd
        
        % plot support
        p
        waypoint
        % sensor geometry
        calibrated
        sensor_placement
    end
    
    properties (Constant)
        inputs = struct('v', 0);
        outputs = struct('v', 0, 'w', 0)
    end
    
    methods
        
        function obj = PotentialField()
            obj = obj@simiam.controller.Controller('potential_field');            
            obj.calibrated = false;
            
            obj.Kp = 4;
            obj.Ki = 0.01;
            obj.Kd = 0.01;
            
            obj.E_k = 0;
            obj.e_k_1 = 0;
        end
        
        function outputs = execute(obj, robot, state_estimate, inputs, dt)
            
            % Compute the placement of the sensors
            if(~obj.calibrated)
                obj.set_sensor_geometry(robot);
            end
            
            %initialize for plotting the trajectory
            obj.waypoint = plot(robot.parent, 0,0,'.','color','r','MarkerSize',7);

            % Unpack state estimate
            [x, y, theta] = state_estimate.unpack();
            
            % Poll the current IR sensor values 1-5
            ir_distances = robot.get_ir_distances();
                        
            % Interpret the IR sensor measurements geometrically
            ir_distances_wf = obj.sf_to_wf(ir_distances, state_estimate);            
            
            
            %repulsive strength scaling function
            function rep = rep_scale(ir_dist)
                D = 0.2;
                rep = 1.5*((1/ir_dist) - (1/D))*(1/ir_dist^2);
            end
            
            rep_gains = arrayfun(@rep_scale, ir_distances);
            sensor_gains = [0.24  0.4  1  0.4  0.24];       % relative importance of each sensor
            sensor_gains = sensor_gains/norm(sensor_gains);
            u_i = -(ir_distances_wf-repmat([x;y],1,5)) * diag(rep_gains)* diag(sensor_gains) ;
            obj.u_rep = sum(u_i,2);
            
            x_g = inputs.x_g;
            y_g = inputs.y_g;
            obj.u_att = [x_g-x; y_g-y];
            obj.u_att = obj.u_att/norm(obj.u_att);  %direction of the attractive force
            
            % to ensure safety
           if any(ir_distances(2:4)< 0.1)
               u = obj.u_rep;              
           else     
               u = obj.u_att + 0.2 * obj.u_rep;
           end
           
            %% PID controller for heading control
            theta_d = atan2(u(2),u(1));
            e = theta_d-theta;
            e = atan2(sin(e),cos(e));   % to ensure that the error is within [-pi,pi]
            
            e_P = e;
            
            e_I = obj.E_k + e*dt;
            e_D = (e-obj.e_k_1)/dt;
              
            % PID control on w
            v = inputs.v;
            w = obj.Kp*e_P + obj.Ki*e_I + obj.Kd*e_D;
            
            % Save errors for next time step
            obj.E_k = e_I;
            obj.e_k_1 = e;
                                                            
            d_to_goal = norm(x_g-x, y_g-y);
            
            % make v a function of both w and distance to goal so that it slows down
            % when steering or reaching goal
            
            outputs.v = (1 - 1/(1 + 1000*d_to_goal^2)) * (1/(1+ 10 * w^2)) * v;
            if any(ir_distances(2:4)< 0.1)
                d = min(ir_distances(2:4));
                outputs.v = outputs.v * (1/(1+3*(1/d- 1/0.1)));              
            end
            outputs.w = w;
            
           % Plotting trajectory
           set(obj.waypoint,'XData', x);
           set(obj.waypoint, 'YData', y);
        end
        
        % Helper functions      
        %Transform IR distance from the sensor frame to world frame
        function ir_distances_wf = sf_to_wf(obj, ir_distances, state_estimate)
                    
            % first transform to robot frame    
            ir_distances_rf = zeros(3,5);
            for i=1:5
                x_s = obj.sensor_placement(1,i);
                y_s = obj.sensor_placement(2,i);
                theta_s = obj.sensor_placement(3,i);
                
                R = obj.get_transformation_matrix(x_s,y_s,theta_s);
                ir_distances_rf(:,i) = R*[ir_distances(i); 0; 1];
            end
            
            % 2. Apply the transformation to world frame.
            
            [x,y,theta] = state_estimate.unpack();
            
            R = obj.get_transformation_matrix(x,y,theta);
            ir_distances_wf = R*ir_distances_rf;
            
            ir_distances_wf = ir_distances_wf(1:2,:);
        end
        
        function set_sensor_geometry(obj, robot)
            obj.sensor_placement = zeros(3,5);
            for i=1:5
                [x, y, theta] = robot.ir_array(i).location.unpack();
                obj.sensor_placement(:,i) = [x; y; theta];
            end                        
            obj.calibrated = true;
        end
        
        function R = get_transformation_matrix(obj, x, y, theta)
            R = [cos(theta) -sin(theta) x; sin(theta) cos(theta) y; 0 0 1];
        end
        
        function reset(obj)
            % Reset accumulated and previous error
            obj.E_k = 0;
            obj.e_k_1 = 0;
        end
        
    end
    
end

