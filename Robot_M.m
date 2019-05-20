%% TRAJECTORY PLANNING MANNUAL MODE

clc;
clear all;

lib_name = 'dxl_x64_c';

if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

% Control table address
ADDR_MX_TORQUE_ENABLE       = 24;           % Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION       = 30;
ADDR_MX_PRESENT_POSITION    = 36;

% Protocol version
PROTOCOL_VERSION            = 1.0;          % See which protocol version is used in the Dynamixel

DXL_ID                      = 1;
BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM3';       

TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 10;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

port_num = portHandler(DEVICENAME);         
% Initialize PacketHandler Structs
packetHandler();

% Open port
if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end

% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% Enable Dynamixel Torque and Limit
for i = 1:7
    write1ByteTxRx(port_num, PROTOCOL_VERSION, i , ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
    fprintf('Dynamixel %d Connected.. \n',i);
    pause(0.05);
end

% Enter the number of points to be covered
x = input('Enter the number of points \n');

for i = 1:x
    fprintf('Move to Position %d \n',x);
    y = input('Do you want to pick or drop here? [p/d] or (c to continue) \n', 's');
    if y == 'p'
        for j = 1:6
            Pos(i,j)=read2ByteTxRx(port_num, PROTOCOL_VERSION, j, ADDR_MX_PRESENT_POSITION);
            pause(0.5);
            Pickpoint = i;
        end
    elseif y == 'd'
        for j = 1:6
            Pos(i,j)=read2ByteTxRx(port_num, PROTOCOL_VERSION, j, ADDR_MX_PRESENT_POSITION);
            pause(0.5);
            Droppoint = i;
        end
    elseif y == 'c'
        for j = 1:6
            Pos(i,j)=read2ByteTxRx(port_num, PROTOCOL_VERSION, j, ADDR_MX_PRESENT_POSITION);
            pause(0.5);
        end
    end
end

% Moving Velocity
v=input('Enter the Movement speed in Percentage \n');
v=(v/100)*1024;

for i = 1:7
    write2ByteTxRx(port_num, PROTOCOL_VERSION, i, 32, 200);
    pause(0.5);
end

% Trajectory Generation
% Reach the Initial Position
for i = 1:6
    write2ByteTxRx(port_num, PROTOCOL_VERSION, i , ADDR_MX_GOAL_POSITION, Pos(1,i));
    pause(0.5);
end

% Gripper Initial state
if Pickpoint == 1
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 7 , ADDR_MX_GOAL_POSITION, 700);
    pause(0.5);
else
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 7 , ADDR_MX_GOAL_POSITION, 400);
    pause(0.5);
end
    
% Pick and Drop Operation
for i = 1:x
    Traj = jtraj(Pos(i,:),Pos(i+1,:),250);
    for j = 1:6
        write2ByteTxRx(port_num, PROTOCOL_VERSION, j , ADDR_MX_GOAL_POSITION, Traj(i,j));
        pause(0.5);
    end
    if i == Pickpoint
        write2ByteTxRx(port_num, PROTOCOL_VERSION, 7 , ADDR_MX_GOAL_POSITION, 700);
        pause(1);
    elseif i == Droppoint
        write2ByteTxRx(port_num, PROTOCOL_VERSION, 7 , ADDR_MX_GOAL_POSITION, 400);
        pause(1);
    else
        pause(1);
    end
end
pause(1);
% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);