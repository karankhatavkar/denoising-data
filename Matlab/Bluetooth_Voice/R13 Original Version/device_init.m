Slot_Ts = (1/1600);
switch HV_Type
    case 1
        Tx_Ts=Slot_Ts*2;
        Num_Slots_Rate=2;
        Num_Payload_Bits=80;
        Slot_Enable_Phase=0; % Slot Pair
    case 2
        Tx_Ts=Slot_Ts*4;
        Num_Slots_Rate=4;
        Num_Payload_Bits=160;
        if Slot_Pair == 3
            error('Slot 5&6 cannot be used as initial slot for HV2');
        end;
        Slot_Enable_Phase=Slot_Pair*2-2; % Slot Pair
    case 3
        Tx_Ts=Slot_Ts*6;
        Num_Slots_Rate=6;
        Num_Payload_Bits=240;
        Slot_Enable_Phase=Slot_Pair*2-2; % Slot Pair
end


