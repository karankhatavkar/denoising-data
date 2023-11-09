% Initialize variables for Bluetooth Model

% Input wave sample rat emust be 8kHz
Input_Fs=8000;

% Coefficients for speech interpolation
[y,interp_coeffs]=interp(ones(1,10),8);

% Header Info
Slave_Address= [1 0 1]';
Packet_Type= [1 0 1 0]';
Flow_Control= [1];
ARQ=[1];
Sequence=[1];
Access_Code=zeros(72,1); Access_Code(1:2:72)=1;
% Header_Info=[Slave_Address;Packet_Type;Flow_Control;ARQ;Sequence];

% 1,0,1,0, sequence
One_Zero_Payload=zeros(240,1);
One_Zero_Payload(1:2:240)=1;

% Seeds
hop_seed=randseed(1);
awgn_channel_seed=randseed(2);
awgn_802_seed=randseed(3);
rate_802_seed=randseed(4);
data_seed=randseed(5);

% Hop frequency if fixed
fixed_hop_freq=20;

%Misc
Num_Payload_Bits=80;


