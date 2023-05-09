% CROMATECH -- Arduino Serial Interfacing Script for S-ROM PoC
% MATLAB Version: R2021b
% Code Version: 
%       1.0 -- single 9-axis IMU reading -- LC
%       1.1 -- dual 9-axis IMU reading -- EB


% initial setup steps:
% 1. change COM port on line 13 to your device specific arduino connection 
% 2. ensure arduino code is updated to output csv's (value1,value2,value3, etc)
% 3. ensure arduino code writes CR and LF 
%    i.e. add serial.write(13) and serial.write(10) to the code
%    this is because these terminators are what trigger the callback to
%    read a line from the serial port in matlab




clear; clc; 

 arduinoObj = serialport("COM5", 115200); %Windows
% arduinoObj = serialport("/dev/cu.usbmodem11101",115200);
    arduinoObj.Timeout = 60;

%/ttys1",115200); % In the case you have a mac .KeySerial1
% time = datetime("now");
% fileName0 = sprintf("IMUExperiments/WithCal/imu0.csv", time);
% fileName1 = sprintf("IMUExperiments/WithCal/imu1.csv", time);

txt = 'x';

time = datetime("now");
time_string = sprintf("%d_%d_%i",hour(time), minute(time));

val = 0;


configureTerminator(arduinoObj,"CR/LF");
flush(arduinoObj);

while(1)

    
    fileName0 = sprintf("IMUExperiments/Test7/imu0_%s_%d.csv", time_string, val);
    fileName1 = sprintf("IMUExperiments/Test7/imu1_%s_%d.csv", time_string, val);
    if(val > 0)
        fclose(fileID);
        fclose(fileID2);
        set(arduinoObj, 'Userdata',[])
    end 

    fileID = fopen(fileName0,'w');
    fileID2 = fopen(fileName1,'w');

    if(txt == 'x') %startup extra messages
        disp(readline(arduinoObj)); %Press any key to continue...
        write(arduinoObj,'r',"char"); %key pressed
        disp(readline(arduinoObj)); %Initialization of the sensor returned: All is well.
        disp(readline(arduinoObj)); %All is well.
        disp(readline(arduinoObj)); %Device connected!
        disp(readline(arduinoObj)); %DMP enabled!
        disp(readline(arduinoObj)); %Start Drain
    end

    while(readline(arduinoObj) ~= "Waiting For Calibration Start")
        %do nothing
        disp("Waiting for cal message")
        %disp(readline(arduinoObj))
    end

    % Print the CSV Header Files
    fprintf(fileID,'%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n','time','IMU#','roll', 'pitch', 'yaw', 'accelx','accely','accelz','compassx','compassy','compassz');
    fprintf(fileID2,'%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n','time','IMU#','roll', 'pitch', 'yaw', 'accelx','accely','accelz','compassx','compassy','compassz');
    
    arduinoObj.UserData = struct("Roll",[],"Pitch",[],"Yaw",[],"Roll2",[],"Pitch2",[],"Yaw2",[],"Extra",[],"Count1",2, "Count2",2,"fileID",fileID,"fileID2",fileID2);

    arduinoObj.UserData.Roll = zeros(1,1000000); %TODO CALCULATE PROPER SIZE
    arduinoObj.UserData.Pitch = zeros(1,1000000); %TODO CALCULATE PROPER SIZE
    arduinoObj.UserData.Yaw = zeros(1,1000000); %TODO CALCULATE PROPER SIZE
    
    arduinoObj.UserData.Roll2 = zeros(1,1000000); %TODO CALCULATE PROPER SIZE
    arduinoObj.UserData.Pitch2 = zeros(1,1000000); %TODO CALCULATE PROPER SIZE
    arduinoObj.UserData.Yaw2 = zeros(1,1000000); %TODO CALCULATE PROPER SIZE
    

    close all;
    figure(1)
    tiledlayout(6,6)
    nexttile([3 6])
    y1 = plot(NaN,NaN);
    title('Roll Angle')
    xlabel('Elapsed Time [s]')
    ylabel('Angle From Rest')
    y1.LineWidth = 2;
    grid;
    ylim([-180 180])
    yticks([-180 -135 -90 -45 0 45 90 135 180])
    xlim([0 120])
    [arduinoObj.UserData.PlotObj1] = y1; %Appends data to structure
    
    nexttile([3 3])
    y2 = plot(NaN,NaN);
    title('Pitch Angle')
    xlabel('Elapsed Time [s]')
    ylabel('Angle From Rest')
    y2.LineWidth = 2;
    grid;
    ylim([-180 180])
    yticks([-180 -135 -90 -45 0 45 90 135 180])
    xlim([0 120])
    [arduinoObj.UserData.PlotObj2] = y2;
    
    nexttile([3 3])
    y3 = plot(NaN,NaN);
    title('Yaw Angle')
    xlabel('Elapsed Time [s] ') %note that this hasnt been done yet
    ylabel('Angle From Rest')
    y3.LineWidth = 2;
    grid;
    ylim([-180 180])
    yticks([-180 -135 -90 -45 0 45 90 135 180])
    xlim([0 120])
    [arduinoObj.UserData.PlotObj3] = y3;
   
    
    figure(2)
    tiledlayout(6,6)
    
    nexttile([3 6])
    y1_5 = plot(NaN,NaN);
    title('Roll Angle')
    xlabel('Elapsed Time [s]') %note that this hasnt been done yet
    ylabel('Angle From Rest')
    grid;
    ylim([-180 180])
    yticks([-180 -135 -90 -45 0 45 90 135 180])
    xlim([0 120])
    y1_5.LineWidth = 2;
    [arduinoObj.UserData.PlotObj1_5] = y1_5; %Appends data to structure
    
    nexttile([3 3])
    y2_5 = plot(NaN,NaN);
    title('Pitch Angle')
    xlabel('Elapsed Time [s]') %note that this hasnt been done yet
    ylabel('Angle From Rest')
    y2_5.LineWidth = 2;
    grid;
    ylim([-180 180])
    yticks([-180 -135 -90 -45 0 45 90 135 180])
    xlim([0 120])
    [arduinoObj.UserData.PlotObj2_5] = y2_5;
    
    nexttile([3 3])
    y3_5 = plot(NaN,NaN);
    title('Yaw Angle')
    xlabel('Elapsed Time [s] ') %note that this hasnt been done yet
    ylabel('Angle From Rest')
    y3_5.LineWidth = 2;
    grid;
    ylim([-180 180])
    yticks([-180 -135 -90 -45 0 45 90 135 180])
    xlim([0 120])
    [arduinoObj.UserData.PlotObj3_5] = y3_5;

    write(arduinoObj,'r',"char");
    disp(readline(arduinoObj));
    disp(readline(arduinoObj));
    disp(readline(arduinoObj));
    tic 
    configureCallback(arduinoObj,"terminator",@readIMUData); %Start getting data
    
    prompt = "Close plots then press any character to continue: ";
    txt = input(prompt,"s");
    configureCallback(arduinoObj, "off");
    if(txt == 'x')
        fclose(fileID);
        fclose(fileID2);
        close all;
        break;
    end

    write(arduinoObj,'d',"char"); %stop exercise

    val = val + 1;
end

