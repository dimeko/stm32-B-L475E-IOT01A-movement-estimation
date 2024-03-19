%% Script to read constantly the UART sychronously
clear all
close all
delete(instrfind)

addpath('./AES/AES');

[s_box, inv_s_box, w, poly_mat, inv_poly_mat] = aes_init;

COM_PORT_NUMBER = "/dev/tty.usbmodem2121103";
baud_rate = 9600;
data_bits = 8;
comm_conn = serial(COM_PORT_NUMBER,'BaudRate',baud_rate,'DataBits',data_bits);

datain = '';
dataout = '1';
count = 1;
fopen(comm_conn);

start_time = datetime;
fwrite(comm_conn,dataout);
prev_v = '';
hold on;
while (1)    
    % Read the available UART data as characters and store the result
    ciphertext = fread(comm_conn,16,'uint8');
    
    datain = inv_cipher (ciphertext, w, inv_s_box, inv_poly_mat);
    clc;
    if count ==1
        prev_v = char(datain(7));
    else
        if char(datain(7)) ~= prev_v
            prev_v = datain(7);
        end
    end
    if prev_v == 83
        mov_rec_str = "standing";
    elseif prev_v == 87
        mov_rec_str = "walking"; 
    elseif prev_v == 82
        mov_rec_str = "running"; 
    else
        mov_rec_str = "fell"; 
    end
    mov_rec_print_str = sprintf('Movement: %s', mov_rec_str);
    disp(mov_rec_print_str);
    if datain(6) == 1  is_in_h_comf_str = "Yes"; else is_in_h_comf_str = "No"; end
    h_comf_print_str = sprintf('Is in human comfort: %s', is_in_h_comf_str);
    disp(h_comf_print_str);
    

    if(datain(1) > 0)
        sign = 1;
    else
        sign = -1;
    end

    % append temperature to temps - remember to concatenate the bytes
    temps(count)= sign * (datain(2)*100 + datain(3))/100;

    % append temperature to hums - remember to concatenate the bytes
    hums(count)= (datain(4)*100 + datain(5))/100;
	
    % append the current time to times
    times(count) = datetime - start_time;

    % plot temps vs times
    subplot(2, 1, 1);
    plot(times(1:count) , temps(1:count));
    xlabel("Time");
    ylabel("Temperture");

    % plot hums vs times
    subplot(2, 1, 2);
    plot(times(1:count) , hums(1:count));
    xlabel("Time");
    ylabel("Humidity");
    drawnow;
    % disp()
    count = count + 1;
end