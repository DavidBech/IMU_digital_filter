close all ; clear ; clc;

% file = "./data/abduction_validate/reg/imu0/imu0_13_32__0.csv";


% csv_data = readtable(file);
% 
% angle0 = csv_data.roll;
% angle1 = csv_data.pitch;
% angle2 = csv_data.yaw;

% pyExec = 'C:\Users\David\anaconda3\envs\ensc413-s2022\python.exe';
% pyRoot = fileparts(pyExec);
% p = getenv('PATH');
% p = strsplit(p, ';');
% addToPath = {
%     pyRoot
%     fullfile(pyRoot, 'Library', 'mingw-w64', 'bin')
%     fullfile(pyRoot, 'Library', 'usr', 'bin')
%     fullfile(pyRoot, 'Library', 'bin')
%     fullfile(pyRoot, 'Scripts')
%     fullfile(pyRoot, 'bin')
%     };
% p = [addToPath(:); p(:)];
% p = unique(p, 'stable');
% p = strjoin(p, ';');
% setenv('PATH', p);

%pyenv('Version',pyExec)

test = pyrunfile("test_py.py", "outVect")

%




