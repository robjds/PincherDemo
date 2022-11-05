%% Script to move a PincherX robot arm between two positions
% r.siddall@surrey.ac.uk, created 1/11/2022 using DynamixelSDK

clear

%% Initialise com port and motors

DEVICENAME = 'COM20'; % Check 'Device Manager' to see which COM port your device is using

run DynamixelStartup % Initialises all the variables needed to run the motor

% Open port
if (openPort(BaseMotor.port_num))
    fprintf('Serial port opened.\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end

% Set port baudrate
if (setBaudRate(BaseMotor.port_num, BaseMotor.BAUDRATE))
    fprintf('Baudrate set.\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

%% Set movement parameters

movementTime = 1000; % the desired time in ms to complete a movement (input 0 for max speed)

reach_position = [2159  2914  1603  1656]; % Reach position ([Base Shoulder Elbow Wrist])
fold_position = [2054   981  2871  1316]; % Retracted position ([Base Shoulder Elbow Wrist])
goal_position = reach_position;

gripStrength = 50; % Force to apply to gripper (as a % of max torque)
gripTime     = 1000; % Time in ms to apply torque for when gripping (don't make more than 1500)

%% Turn motor on and run

% Enable Dynamixel Torque
torqueOn(BaseMotor)
torqueOn(ShoulderMotor)
torqueOn(ElbowMotor)
torqueOn(WristMotor)
torqueOn(GripperMotor)

setMoveTime(BaseMotor,movementTime)
setMoveTime(ShoulderMotor,movementTime)
setMoveTime(ElbowMotor,movementTime)
setMoveTime(WristMotor,movementTime)

while 1
    moveYN = input('Input ''r'' to reach the arm, ''f'' to retract, ''s'' to skip moving, or ''e'' to quit.\n', 's');
    if moveYN == ESC_CHARACTER
        break;
    elseif moveYN == 's'    
        disp('Doing nothing...')
    else
        switch moveYN 
            case 'r'
                goal_position = reach_position;
            case 'f'
                goal_position = fold_position;
        end    
        % Write goal position
        moveMotor(BaseMotor,     goal_position(1))
        moveMotor(ShoulderMotor, goal_position(2))
        moveMotor(ElbowMotor,    goal_position(3))
        moveMotor(WristMotor,    goal_position(4))
        disp('Moving arm...')
        pause(1*movementTime/1000); % Wait for robot arm to move.
    end

    % Open/close gripper
    gripYN = input('Move Gripper? input ''o'' to open, ''c'' to close, anything else to skip.\n','s');
    if gripYN == 'o'
        moveGripper(GripperMotor,gripStrength);  % Turn on the gripper  motor
        pause(gripTime/1000);                    % Give the gripper  time to close
        moveGripper(GripperMotor,0);             % Turn on the gripper  motor
        disp('Gripper opened.')
    elseif gripYN == 'c'
        moveGripper(GripperMotor,-gripStrength); % Turn on the gripper  motor
        pause(gripTime/1000);                    % Give the gripper  time to close
        moveGripper(GripperMotor,0);             % Turn on the gripper  motor
        disp('Gripper closed.')
    else 
        disp('Doing nothing...')
    end
end

%% Turn off motor torque and close down com port

% Enable Dynamixel Torque
torqueOff(BaseMotor)
torqueOff(ShoulderMotor)
torqueOff(ElbowMotor)
torqueOff(WristMotor)
torqueOff(GripperMotor)

% Close port
closePort(BaseMotor.port_num);

% Unload Library
unloadlibrary(lib_name);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% END OF PROGRAM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Functions for motor operations (shouldn't need to modify)

%% Move a motor
function [] = moveMotor(MotorID,goalPosition)
    % Write goal position
    if (goalPosition > MotorID.MAXIMUM_POSITION)||(goalPosition < MotorID.MINIMUM_POSITION)
        disp('Requested movement is out of range')
    else    
        write4ByteTxRx(MotorID.port_num, MotorID.PROTOCOL_VERSION, MotorID.DXL_ID, MotorID.ADDR_GOAL_POSITION, typecast(int32(goalPosition), 'uint32'));
        dxl_comm_result = getLastTxRxResult(MotorID.port_num, MotorID.PROTOCOL_VERSION);
        dxl_error = getLastRxPacketError(MotorID.port_num, MotorID.PROTOCOL_VERSION);
        if dxl_comm_result ~= MotorID.COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(MotorID.PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(MotorID.PROTOCOL_VERSION, dxl_error));
        end
    end
end

%% Open gripper motor
function [] = moveGripper(MotorID,goalPWM)
    % Write goal position
    if abs(goalPWM) > 100
        disp('Requested grip force is out of range')
    else  
        goalPWM = round(goalPWM*885/100);
        write2ByteTxRx(MotorID.port_num, MotorID.PROTOCOL_VERSION, MotorID.DXL_ID, MotorID.ADDR_GOAL_PWM, typecast(int16(goalPWM), 'uint16'));
        dxl_comm_result = getLastTxRxResult(MotorID.port_num, MotorID.PROTOCOL_VERSION);
        dxl_error = getLastRxPacketError(MotorID.port_num, MotorID.PROTOCOL_VERSION);
        if dxl_comm_result ~= MotorID.COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(MotorID.PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(MotorID.PROTOCOL_VERSION, dxl_error));
        end
    end
end   

%% Set motor move rate
function [] = setMoveTime(MotorID,goalTime)
    % Write goal position
    if (goalTime > 32767)||(goalTime < 0)
        disp('Requested movement speed is out of range')
    else    
        write4ByteTxRx(MotorID.port_num, MotorID.PROTOCOL_VERSION, MotorID.DXL_ID, MotorID.ADDR_TIMESPAN, typecast(int32(goalTime), 'uint32'));
        dxl_comm_result = getLastTxRxResult(MotorID.port_num, MotorID.PROTOCOL_VERSION);
        dxl_error = getLastRxPacketError(MotorID.port_num, MotorID.PROTOCOL_VERSION);
        if dxl_comm_result ~= MotorID.COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(MotorID.PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(MotorID.PROTOCOL_VERSION, dxl_error));
        end
    end
end   

%% Turn off torque
function [] = torqueOff(MotorID)
    % Disable Dynamixel Torque
    write1ByteTxRx(MotorID.port_num, MotorID.PROTOCOL_VERSION, MotorID.DXL_ID, MotorID.ADDR_TORQUE_ENABLE, MotorID.TORQUE_DISABLE);
    dxl_comm_result = getLastTxRxResult(MotorID.port_num, MotorID.PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(MotorID.port_num, MotorID.PROTOCOL_VERSION);
    if dxl_comm_result ~= MotorID.COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(MotorID.PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(MotorID.PROTOCOL_VERSION, dxl_error));
    end
end

%% Turn on torque
function [] = torqueOn(MotorID)
    write1ByteTxRx(MotorID.port_num, MotorID.PROTOCOL_VERSION, MotorID.DXL_ID, MotorID.ADDR_TORQUE_ENABLE, MotorID.TORQUE_ENABLE);
    dxl_comm_result = getLastTxRxResult(MotorID.port_num, MotorID.PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(MotorID.port_num, MotorID.PROTOCOL_VERSION);
    if dxl_comm_result ~= MotorID.COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(MotorID.PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(MotorID.PROTOCOL_VERSION, dxl_error));
    end
end

%% Get position data
function [dataRead] = getData(MotorID,dataType)
    switch(dataType)
        case 'position'
            data_address = MotorID.ADDR_PRESENT_POSITION;
        case 'velocity'
            data_address = MotorID.ADDR_PRESENT_VELOCITY;
    end
    
    dataRead = read4ByteTxRx(MotorID.port_num, MotorID.PROTOCOL_VERSION, MotorID.DXL_ID,data_address);
    dxl_comm_result = getLastTxRxResult(MotorID.port_num, MotorID.PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(MotorID.port_num, MotorID.PROTOCOL_VERSION);
    if dxl_comm_result ~= MotorID.COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(MotorID.PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(MotorID.PROTOCOL_VERSION, dxl_error));
    end
end

%% Get load data
function [dataRead] = getLoad(MotorID)   
    dataRead = read2ByteTxRx(MotorID.port_num, MotorID.PROTOCOL_VERSION, MotorID.DXL_ID,MotorID.ADDR_PRESENT_LOAD);
    dataRead = (uint16(dataRead));
    dxl_comm_result = getLastTxRxResult(MotorID.port_num, MotorID.PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(MotorID.port_num, MotorID.PROTOCOL_VERSION);
    if dxl_comm_result ~= MotorID.COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(MotorID.PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(MotorID.PROTOCOL_VERSION, dxl_error));
    end
end