classdef FollowWall < simiam.controller.Controller

% Authors: Yuwei Wang , Sailalitha Gollapudi , Mohit Patel 

    properties
        u_rep
        u_att
        
        % for PID
        E_k
        e_prev
        
        % gains
        Kp
        Ki
        Kd
        
        % plot support
        p
        
        % sensor geometry
        calibrated
        sensor_placement
        
        v_t
        v_p
        v_pt
        
        waypoint
    end
    
    properties (Constant)
        inputs = struct('v', 0, 'direction', 'right');
        outputs = struct('v', 0, 'w', 0)
    end
    
    methods
        
        function obj = FollowWall()
            obj = obj@simiam.controller.Controller('follow_wall');            
            obj.calibrated = false;
            
            obj.Kp = 2.7;
            obj.Ki = 0;
            obj.Kd = 0;
            
            obj.E_k = 0;
            obj.e_prev = 0;
            
           obj.p = simiam.util.Plotter();
        end
        
        function outputs = execute(obj, robot, state_estimate, inputs, dt)
          
            %Initialize for Plotting
            if(~obj.calibrated)
                obj.set_sensor_geometry(robot);
                 
                hold(robot.parent, 'on');

                obj.v_t = plot(robot.parent, [0;0],[0;0],'b-', 'LineWidth', 2);
                obj.v_p = plot(robot.parent, [0;0],[0;0],'b-', 'LineWidth', 2);
                set(obj.v_t, 'ZData', [2;2]);
                set(obj.v_p, 'ZData', [2;2]);
            end
            obj.waypoint = plot(robot.parent, 0,0,'.','color','b','MarkerSize',7);
            
            % Unpack state estimate
            [x, y, theta] = state_estimate.unpack();
            % collect the current IR sensor values 1-5
            ir_distances = robot.get_ir_distances();          
            % interpret distances by converting from sensor frame to world frame
            ir_distances_wf = obj.sf_to_wf(ir_distances, state_estimate);       
            
            
            obj.u_att = [inputs.x_g-x; inputs.y_g-y];
            
                      
            %% compute u_t (vector that is parallel to the wall for following the wall)
            %% and u_p (vector perpendicular to the wall for keeping a safe distance)
            if(strcmp(inputs.direction,'left'))
                % Pick two right sensors with smallest ir_distances
                ir_distances = ir_distances(1:3).*[1.05 1 1]';
                [out idx] = sort(ir_distances);
                
                p_1 = ir_distances_wf(:,min(idx(1),idx(2)));
                p_2 = ir_distances_wf(:,max(idx(1),idx(2)));
                
                u_t = p_2 - p_1;   % parallel to the wall for following the wall
                u_t = u_t/norm(u_t);
                u_p = [0 1;-1 0] * u_t;  %rotate u_t cw by pi/2
            else
                % Pick two left sensors with the smallest ir_distances    
                ir_distances = ir_distances(3:5).*[1.05 1 1]';  
                [out idx] = sort(ir_distances);
                
                p_1 = ir_distances_wf(:,max(idx(1),idx(2))+2);
                p_2 = ir_distances_wf(:,min(idx(1),idx(2))+2);
                
                u_t = p_2 - p_1;   % parallel to the wall for following the wall
                u_t = u_t/norm(u_t);            
                u_p = [0 -1;1 0] * u_t; % rotate u_t ccw by pi/2
            end
            
            obj.u_rep = u_p;
            d_fw = inputs.d_fw; 
            d = norm(cross([x;y;0]-[p_1;0], [x;y;0]-[p_2;0]))/norm(p_2-p_1);  %distance from the robot to the wall
            k = 6;                                % control gain for distance keeping
            u_p = k * (d_fw - d) * u_p;         % properly scale u_p to ensure distance keeping
           
            u_fw = u_t + u_p;                    % combine u_t and u_p to get u_fw
                      
         %% PID for steering
           
           % Compute the heading and error for the PID controller
            theta_d = atan2(u_fw(2),u_fw(1));
            e = theta_d-theta;
            e = atan2(sin(e),cos(e));   % to ensure that the error is within [-pi,pi]
            
            e_P = e;
            
            e_I = obj.E_k + e*dt;
            e_D = (e-obj.e_prev)/dt;
              
            % PID control on w
            v = min(inputs.v, 0.18);
            w = obj.Kp*e_P + obj.Ki*e_I + obj.Kd*e_D;
            
            % Save errors for next time step
            obj.E_k = e_I;
            obj.e_prev = e;
           
           %Plotting tangential and perpendicular component for following
           %the wall
           set(obj.v_t, 'XData', [u_t(1)/4+p_1(1);p_1(1)]);
           set(obj.v_t, 'YData', [u_t(2)/4+p_1(2);p_1(2)]);
           set(obj.v_p, 'XData', [u_p(1)/2+x;x]);
           set(obj.v_p, 'YData', [u_p(2)/2+y;y]);
           
           % Plotting trajectory
           set(obj.waypoint,'XData', x);
           set(obj.waypoint, 'YData', y);
          
            
            outputs.v = v;
            outputs.w = w;
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
        
    end
    
end

