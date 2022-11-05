%% Script to startup dynamixel serial connection
% r.siddall@surrey.ac.uk, created 1/11/2022 using DynamixelSDK
% DEVICENAME must be defined before this script is run.

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

BaseMotor.ADDR_TORQUE_ENABLE    = 64;
BaseMotor.ADDR_GOAL_POSITION    = 116;
BaseMotor.ADDR_GOAL_PWM         = 100;
BaseMotor.ADDR_PRESENT_POSITION = 132;
BaseMotor.ADDR_TIMESPAN         = 112;
BaseMotor.ADDR_PRESENT_VELOCITY = 128;
BaseMotor.ADDR_PRESENT_LOAD     = 126;
BaseMotor.MINIMUM_POSITION      = 0; % Dynamixel will rotate between this value
BaseMotor.MAXIMUM_POSITION      = 4095; % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
BaseMotor.BAUDRATE              = 1000000;
BaseMotor.PROTOCOL_VERSION      = 2.0;          

% Factory default ID of all DYNAMIXEL is 1
BaseMotor.DXL_ID                      = 1; 

% Common Control Table Address and Data 
BaseMotor.ADDR_OPERATING_MODE         = 11;          
BaseMotor.OPERATING_MODE              = 3;            % value for operating mode for position control                                
BaseMotor.TORQUE_ENABLE               = 1;            % Value for enabling the torque
BaseMotor.TORQUE_DISABLE              = 0;            % Value for disabling the torque
BaseMotor.DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

BaseMotor.COMM_SUCCESS                = 0;            % Communication Success result value
BaseMotor.COMM_TX_FAIL                = -1001;        % Communication Tx Failed

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows

BaseMotor.port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = BaseMotor.COMM_TX_FAIL;           % Communication result

% This script creates a struct called 'BaseMotor' with all the
% variables needed for communication. Duplicate this struct to use more
% motors, updating ID values and movement limits.

%% Create Additional Motors for other arm joints
ShoulderMotor = BaseMotor; 
ShoulderMotor.DXL_ID = 2; 
ShoulderMotor.MINIMUM_POSITION = 784; 
ShoulderMotor.MAXIMUM_POSITION = 3265;

ElbowMotor = BaseMotor; 
ElbowMotor.DXL_ID = 3; 
ElbowMotor.MINIMUM_POSITION = 671; 
ElbowMotor.MAXIMUM_POSITION = 3094;

WristMotor = BaseMotor; 
WristMotor.DXL_ID = 4; 
WristMotor.MINIMUM_POSITION = 910; 
WristMotor.MAXIMUM_POSITION = 3447;

GripperMotor = BaseMotor; 
GripperMotor.DXL_ID = 5; 

motorNames = categorical({'Base','Shoulder','Elbow','Wrist','Gripper'});
motorNames = reordercats(motorNames,{'Base','Shoulder','Elbow','Wrist','Gripper'});

%% Arm dimensions
