%% Script to read constantly the UART sychronously
clear all
close all
delete(instrfind)

addpath('./AES/AES');

[s_box, inv_s_box, w, poly_mat, inv_poly_mat] = aes_init;

COM_PORT_NUMBER = "/dev/tty.usbmodem2121103";
baud_rate = 9600;
data_bits = 8;
myComPort = serial(COM_PORT_NUMBER,'BaudRate',baud_rate,'DataBits',data_bits);

datain = '';
dataout = '1';
count = 1;
fopen(myComPort);

gxs = '';gys = ''; gzs = '';
axs = '';ays = ''; azs = '';
start_time = datetime;
hold on;

% t = zeros(size(timestamp));
prev_acc = zeros(3);
prev_gyro = zeros(3);

while (1)

    % Here is the Accelerometer
    fwrite(myComPort,dataout);
    ciphertext = fread(myComPort,32,'uint8');
    
    % Reshape the buffer to form pairs of uint8_t elements
    axs = '';
    for i=1:8
        if ciphertext(i) == 0
            break
        end
        axs = strcat(axs, char(ciphertext(i)));
    end
    % disp(str2num(axs))
    
    ays = '';
    for i=9:16
        if ciphertext(i) == 0
            break
        end
        ays = strcat(ays, char(ciphertext(i)));
    end
    % disp(str2num(ays))

    azs = '';
    for i=17:24
        if ciphertext(i) == 0
            break
        end
        azs = strcat(azs, char(ciphertext(i)));
    end
    % disp(str2num(azs))

    % This is the accelerometer
    fwrite(myComPort,dataout);
    
    ciphertext = fread(myComPort,32,'uint8');

    gxs = '';
    for i=1:10
        if ciphertext(i) == 0
            break
        end
        gxs = strcat(gxs, char(ciphertext(i)));
    end
    % disp(str2num(axs))
    
    gys = '';
    for i=11:20
        if ciphertext(i) == 0
            break
        end
        gys = strcat(gys, char(ciphertext(i)));
    end
    % disp(str2num(ays))

    gzs = '';
    for i=22:31
        if ciphertext(i) == 0
            break
        end
        gzs = strcat(gzs, char(ciphertext(i)));
    end

    % combinedAccel = sqrt(axs.^2+ays.^2+azs.^2);
    times(count) = datetime - start_time;
    
    xacc(count) = str2num(axs);
    yacc(count) = str2num(ays);
    zacc(count) = str2num(azs);
    % 
    subplot(3, 2, 1);
    plot(times(1:count) , xacc(1:count));
    xlabel("Time");
    ylabel("Acc X");
    subplot(3, 2, 3);
    plot(times(1:count) , yacc(1:count));
    xlabel("Time");
    ylabel("Acc Y");
    subplot(3, 2, 5);
    plot(times(1:count) , zacc(1:count));
    xlabel("Time");
    ylabel("Acc Z");

    

    xgyr(count) = str2num(gxs);
    ygyr(count) = str2num(gys);
    zgyr(count) = str2num(gzs);

    subplot(3, 2, 2);
    plot(times(1:count) , xgyr(1:count));
    xlabel("Time");
    ylabel("Gyro X");
    subplot(3, 2, 4);
    plot(times(1:count) , ygyr(1:count));
    xlabel("Time");
    ylabel("Gyro Y");
    subplot(3, 2, 6);
    plot(times(1:count) , zgyr(1:count));
    xlabel("Time");
    ylabel("Gyro Z");


    pause(0.2);
    count = count + 1;

end