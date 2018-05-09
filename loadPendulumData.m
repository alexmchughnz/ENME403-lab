function data = loadPendulumData(filename)
    load(filename);
    file = eval(filename)
    vars = file.Y;
    
    t = file.X.Data;    
    r = vars(6).Data;
    x = vars(1).Data;
    xdot = vars(3).Data;
    theta = vars(8).Data;    
    thetadot = vars(9).Data;
    K = [vars(2).Data(end), vars(4).Data(end)...
         vars(7).Data(end), vars(10).Data(end)];
    N = vars(5).Data(end);
    V = vars(11).Data;
    
    data = struct('t', t,...
                  'r', r,...
                  'x', x,...
                  'xdot', xdot,...
                  'theta', theta,...
                  'thetadot', thetadot,...
                  'K', K,...
                  'N', N,...
                  'V', V);
    
%     cart_position = example.Y(1).Data;
%     cart_position_gain = example.Y(2).Data;
%     cart_velocity = example.Y(3).Data;
%     cart_velocity_gain = example.Y(4).Data;
%     tracking_gain = example.Y(5).Data;
%     cart_position_command = example.Y(6).Data;
%     pendulum_position_gain = example.Y(7).Data;
%     pendulum_position = example.Y(8).Data;
%     pendulum_velocity = example.Y(9).Data;
%     pendulum_velocity_gain = example.Y(10).Data;
%     raw_motor_voltage = example.Y(11).Data;
    
    
end

