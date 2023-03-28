filename1 = 'IMUExperiments/WithCal/imu0_25-Mar-2023 17:03:26ExtRotat.csv';
filename2 = 'IMUExperiments/WithCal/imu1_25-Mar-2023 17:03:26ExtRotat.csv';

filename3 = 'IMUExperiments/WithCal/imu0_25-Mar-2023 16:59:27FlexCompandIncorrect.csv';
filename4 = 'IMUExperiments/WithCal/imu1_25-Mar-2023 16:59:27FlexCompandIncorrect.csv';

% filename1 = 'IMUExperiments/NoCal/imu0_25-Mar-2023-14-25-46Abd.csv';
% filename2 = 'IMUExperiments/NoCal/imu1_25-Mar-2023-14-25-46Abd.csv';
% 
% filename3 = 'IMUExperiments/NoCal/imu0_25-Mar-2023-14-29-04AbdTrunk.csv';
% filename4 = 'IMUExperiments/NoCal/imu1_25-Mar-2023-14-29-04AbdTrunk.csv';


IMU0 = readmatrix(filename1);
IMU1 = readmatrix(filename2);

IMU0_Err = readmatrix(filename3);
IMU1_Err = readmatrix(filename4);


Time = IMU0(:,1);
IMUNum = IMU0(:,2);
Roll = IMU0(:,3);
Pitch = IMU0(:,4);
Yaw = IMU0(:,5);

Time2 = IMU1(:,1);
IMUNum2 = IMU1(:,2);
Roll2 = IMU1(:,3);
Pitch2 = IMU1(:,4);
Yaw2 = IMU1(:,5);

Time_Err = IMU0_Err(:,1);
IMUNum_Err = IMU0_Err(:,2);
Roll_Err = IMU0_Err(:,3);
Pitch_Err = IMU0_Err(:,4);
Yaw_Err = IMU0_Err(:,5);

Time2_Err = IMU1_Err(:,1);
IMUNum2_Err = IMU1_Err(:,2);
Roll2_Err = IMU1_Err(:,3);
Pitch2_Err = IMU1_Err(:,4);
Yaw2_Err = IMU1_Err(:,5);

TimeV = Time(1:end-1)+diff(Time)./2;
RollV = Derivative(Time,Roll);
PitchV = Derivative(Time,Pitch);
YawV = Derivative(Time,Yaw);


Time2V = Time2(1:end-1)+diff(Time2)./2;
Roll2V = Derivative(Time2,Roll2);
Pitch2V = Derivative(Time2,Pitch2);
Yaw2V = Derivative(Time2,Yaw2);


TimeErrV = Time_Err(1:end-1)+diff(Time_Err)./2;
Roll_ErrV = Derivative(Time_Err,Roll_Err);
Pitch_ErrV = Derivative(Time_Err,Pitch_Err);
Yaw_ErrV = Derivative(Time_Err,Yaw_Err);

TimeErr2V = Time2_Err(1:end-1)+diff(Time2_Err)./2;
Roll2_ErrV = Derivative(Time2_Err,Roll2_Err);
Pitch2_ErrV = Derivative(Time2_Err,Pitch2_Err);
Yaw2_ErrV = Derivative(Time2_Err,Yaw2_Err);

n = 1;

figure(n)
yyaxis left
plot(Time,Roll,'g-',Time2,Roll2,'b-');
ylabel("Roll angle")
yyaxis right
plot(TimeV,RollV,'r-',Time2V,Roll2V,'k-');
legend("Imu0","IMU1","Imu0-v","IMU1-v");
title("Roll and Roll Velocity")
xlabel("Time")
ylabel("Roll angle Velocity")

n=n+1;
figure(n)
yyaxis left
plot(Time,Pitch,'g-',Time2,Pitch2,'b-');
ylabel("Pitch angle")
yyaxis right
plot(TimeV,PitchV,'r-',Time2V,Pitch2V,'k-');
legend("Imu0","IMU1","Imu0-v","IMU1-v");
title("Pitch and Pitch Velocity")
xlabel("Time")
ylabel("Pitch angle Velocity")

n=n+1;
figure(n)
yyaxis left
plot(Time,Yaw,'g-',Time2,Yaw2,'b-');
ylabel("Yaw angle")
yyaxis right
plot(TimeV,YawV,'r-',Time2V,Yaw2V,'k-');
legend("Imu0","IMU1","Imu0-v","IMU1-v");
title("Yaw and Yaw Velocity")
xlabel("Time")
ylabel("Yaw angle Velocity")

n=n+1;
figure(n)
yyaxis left
plot(Time_Err,Roll_Err,'g-',Time2_Err,Roll2_Err,'b-');
ylabel("Roll Err angle")
yyaxis right
plot(TimeErrV,Roll_ErrV,'r-',TimeErr2V,Roll2_ErrV,'k-');
legend("Imu0","IMU1","Imu0-v","IMU1-v");
title("Roll Err and Roll Err Velocity")
xlabel("Time Err")
ylabel("Roll Err angle Velocity")

n=n+1;
figure(n)
yyaxis left
plot(Time_Err,Pitch_Err,'g-',Time2_Err,Pitch2_Err,'b-');
ylabel("Pitch Err angle")
yyaxis right
plot(TimeErrV,Pitch_ErrV,'r-',TimeErr2V,Pitch2_ErrV,'k-');
legend("Imu0","IMU1","Imu0-v","IMU1-v");
title("Pitch Err and Pitch Err Velocity")
xlabel("Time Err")
ylabel("Pitch Err angle Velocity")

n=n+1;
figure(n)
yyaxis left
plot(Time_Err,Yaw_Err,'g-',Time2_Err,Yaw2_Err,'b-');
ylabel("Yaw Err angle")
yyaxis right
plot(TimeErrV,Yaw_ErrV,'r-',TimeErr2V,Yaw2_ErrV,'k-');
legend("Imu0","IMU1","Imu0-v","IMU1-v");
title("Yaw Err and Yaw Err Velocity")
xlabel("Time Err")
ylabel("Yaw Err angle Velocity")



function[dYdT] = Derivative(Time,Yval)
    dYval = diff(Yval);
    dT = diff(Time);
    dYdT = dYval./dT;
end


