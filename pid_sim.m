% script which simulates the physics of a board that is hinged,
% with a motor that provides upwards thrust on the end

% Auguste Brown

close all

% define constants
board_length = 0.32; % meters
board_width = 0.038;
board_thickness = 0.015;
wood_density = 600; % kg / m^3
hinge_loc = 0; % location on the board where it will hinge
motor_loc = 0.31; % location of motor
f_coefficient = 0.5;
max_thrust = 0.9 * 9.81; % estimate about 0.9 kg max thrust
max_theta = 90;

board_mass = (board_length * board_width * board_thickness) * wood_density;
board_weight = board_mass * -9.81; % force points downward
motor_mass = 0.24; % kg
motor_weight = motor_mass * -9.81;

% define distances used to calculate moment arms
motor_dist = motor_loc - hinge_loc; % moment arm for motor (thrust is always perpendicular to member)
weight_dist = (board_length / 2) - hinge_loc; % distance center of mass of board to hinge

moment_inertia = (1 / 12) * board_mass * board_length ^2 ...
                  + board_mass * weight_dist ^2 ... % parallel axis theorem
                  + motor_mass * motor_dist ^2; % treat motor as point mass
              
friction = f_coefficient * (board_mass + motor_mass) * 9.81;

% define state variables and intiate to starting values
t = 0; % units: seconds
total_time = 5;
delta_t = 0.001;
num_indices = total_time / delta_t + 1;

thrust = zeros(num_indices, 1);
theta = zeros(num_indices, 1);
net_moment = zeros(num_indices, 1);
angular_v = zeros(num_indices, 1);
angular_a = zeros(num_indices, 1);
i = 1;

theta(1) = -3.14159 /2;
while t < total_time
    %define moment arms
    motor_arm = motor_dist * cos(theta(i));
    weight_arm = weight_dist * cos(theta(i));
    
    net_moment(i) = motor_dist * thrust(i) + motor_arm * motor_weight + weight_arm * board_weight;
    
%     if angular_v(i) > 0
%         net_moment(i) = net_moment(i) - friction * 0.01;
%     elseif angular_v(i) < 0
%         net_moment(i) = net_moment(i) + friction * 0.01;
%     end
    
    % update state
    angular_a(i) = net_moment(i) / moment_inertia;
    angular_v(i + 1) = angular_v(i) + (angular_a(i) * delta_t);
    theta(i + 1) = theta(i) + (angular_v(i) * delta_t) + 1/2 * (angular_a(i) * delta_t ^2);
    thrust(i + 1) = controller(0, theta, angular_v(i), max_thrust, i);
    
    t = t + delta_t;
    i = i + 1;
end

figure (1)
plot(thrust, 'b')
legend('thrust');


figure (2)
hold on
subplot(3, 1, 1)
plot(theta, 'b')
xlabel('Time (milliseconds)')
ylabel('Radians')
legend('angle');
subplot(3, 1, 2)
plot(angular_v, 'g')
xlabel('Time (milliseconds)')
legend('angular velocity');
ylabel('Radians per second')
subplot(3, 1, 3)
plot(angular_a, 'm')
xlabel('Time (milliseconds)')
ylabel('Radians per sec^2')
legend('angular acceleration');

function thrust = controller(theta_set, theta_act, vel_act, max_thrust, i)
    Kp = 200;
    Kd = 60;
    Ki = 1;
    diff = theta_set - theta_act(i);
    total = sum(theta_act);
    
    thrust_uncontained = Kp * diff + Kd * -vel_act + Ki * -total;
    
    if thrust_uncontained > max_thrust
        thrust = max_thrust;
    elseif thrust_uncontained < 0
        thrust = 0;
    else
        thrust = thrust_uncontained;
    end
end