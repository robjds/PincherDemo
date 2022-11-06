%% Script to move a PincherX robot arm between two positions
% r.siddall@surrey.ac.uk, created 1/11/2022 using DynamixelSDK

clear

%% Initialise com port and motors

DEVICENAME = 'COM21'; % Check 'Device Manager' to see which COM port your device is using

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
no_of_revolutions = 4.5;
up_position   = round(4095*no_of_revolutions); % the postion to winch up to (4095 = 1 revolution)
down_position = 0;     % the postion to winch down to
goal_position = down_position;

%% Turn motor on and run

% Enable Dynamixel Torque
torqueOn(BaseMotor)
loggedData = [];

while 1
    moveYN = input('Input ''u'' to winch up, ''d'' to unspool, or ''e'' to quit.\n', 's');
    if moveYN == ESC_CHARACTER
        break;
    else
        loggedData = [];
        switch moveYN 
            case 'u'
                goal_position = up_position;
            case 'd'
                goal_position = down_position;
        end    
        % Write goal position
        tic
        moveMotor(BaseMotor,goal_position)
        while 1
            % Read present position
            time_now = toc*1e3;
            dxl_present_position = getData(BaseMotor,'position');
            dxl_present_velocity = getData(BaseMotor,'velocity');
            dxl_present_load     = getLoad(BaseMotor);
            loggedData = [loggedData, double([time_now; typecast(uint32(dxl_present_position), 'int32');...
                                              typecast(uint32(dxl_present_velocity), 'int32'); dxl_present_load])];
            
            if ~(abs(goal_position - typecast(uint32(dxl_present_position), 'int32')) > BaseMotor.DXL_MOVING_STATUS_THRESHOLD)
                %% Movement finished, plot output data
                disp(['Movement took ' num2str(round(toc*1e3)) ' milliseconds.'])
                disp(['Average velocity was ' num2str(round(mean(loggedData(3,:))*0.229*2*pi/60,2)) ' rad/s.'])
                tiledlayout(2,1)
                % Top plot
                nexttile
                plot(loggedData(1,:),loggedData(2,:)*21e-3*pi/4095,'LineWidth',2)
                xlabel('Time (ms)'); ylabel('Position (m)')
                title('Logged Dynamixel Data')
                grid minor
                % Middle plot
                nexttile
                plot(loggedData(1,:),loggedData(3,:)*0.229*2*pi/60,'LineWidth',2)
                xlabel('Time (ms)'); ylabel('Velocity (rad/s)')
                grid minor
                shg
                break;
            end
        end
    end

end

%% Turn off motor torque and close down com port

% Enable Dynamixel Torque
torqueOff(BaseMotor)

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
%     if (goalPosition > MotorID.MAXIMUM_POSITION)||(goalPosition < MotorID.MINIMUM_POSITION)
%         disp('Requested movement is out of range')
%         break
%     else    
        write4ByteTxRx(MotorID.port_num, MotorID.PROTOCOL_VERSION, MotorID.DXL_ID, MotorID.ADDR_GOAL_POSITION, goalPosition);
        dxl_comm_result = getLastTxRxResult(MotorID.port_num, MotorID.PROTOCOL_VERSION);
        dxl_error = getLastRxPacketError(MotorID.port_num, MotorID.PROTOCOL_VERSION);
        if dxl_comm_result ~= MotorID.COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(MotorID.PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(MotorID.PROTOCOL_VERSION, dxl_error));
        end
%     end
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
%     dataRead = (int16(dataRead));
    dxl_comm_result = getLastTxRxResult(MotorID.port_num, MotorID.PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(MotorID.port_num, MotorID.PROTOCOL_VERSION);
    if dxl_comm_result ~= MotorID.COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(MotorID.PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(MotorID.PROTOCOL_VERSION, dxl_error));
    end
end
