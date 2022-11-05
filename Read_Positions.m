%% Script to read the current positions of a pincher robot arm
% r.siddall@surrey.ac.uk, created 1/11/2022 using DynamixelSDK

%% Initialise serial port and motor(s)

clc;
clear all;

DEVICENAME = 'COM20'; % Check 'Device Manager' to see which COM port your device is using

run DynamixelStartup 

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
%% Read data and print

armPositions = [getData(BaseMotor,'position'),...
                getData(ShoulderMotor,'position'),...
                getData(ElbowMotor,'position'),...
                getData(WristMotor,'position')];

disp(['Arm joint positions are: ']);
disp(['Base Joint:     ' num2str(round(armPositions(1)*360/4095)) '° / ' num2str(armPositions(1)) ' bits'])     
disp(['Shoulder Joint: ' num2str(round(armPositions(2)*360/4095)) '° / ' num2str(armPositions(2)) ' bits'])     
disp(['Elbow Joint:    ' num2str(round(armPositions(3)*360/4095)) '° / ' num2str(armPositions(3)) ' bits'])     
disp(['Wrist Joint:    ' num2str(round(armPositions(4)*360/4095)) '° / ' num2str(armPositions(4)) ' bits'])     
disp(['For copy/paste: [' num2str(armPositions) ']']); disp([' '])

%% Close port

% Close port
closePort(BaseMotor.port_num);

% Unload Library
unloadlibrary(lib_name);
fprintf('Serial port closed.\n');
close all;
clear all;

%% Functions for motor operations
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