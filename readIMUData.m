 function readIMUData(src, ~)
    
    if(isempty(src.UserData)) %if the plot window closed turn off the callback (program ends)
        disp("We left Joe Mama")
        %write(arduinoObj,'d',"char"); %stop exercise
%         write(src,'d',"char"); %stop exercise
%         configureCallback(src, "off");
        return;
    end
%     disp("We entered Joe Mama")
    % Read the ASCII data from the serialport object.
    line = readline(src);
%     disp(line)
    line = strip(line);
%     disp(line)
    %extract sensor data 
    IMUdata = sscanf(line, '%f,%f,%f,%f');

    if (~isempty(IMUdata))
        
        
        if (IMUdata(1)==0)
            Roll = IMUdata(2);
            Pitch = IMUdata(3);
            Yaw = IMUdata(4);
            


            %solution for -180 to 180 discontinuity 
            %for efficiency, can check for normal case (current angle and previous one have same sign) 
            %first to skip the other comparisons

            if (abs(Roll) > 45 && sign(Roll * src.UserData.Roll(src.UserData.Count1-1)) < 0)
                Roll = Roll + sign(src.UserData.Roll(src.UserData.Count1-1))*360;
            end
            if (abs(Pitch) > 45 && sign(Pitch * src.UserData.Pitch(src.UserData.Count1-1)) < 0)
                Pitch = Pitch + sign(src.UserData.Pitch(src.UserData.Count1-1))*360;
            end
            if (abs(Yaw) > 45 && sign(Yaw * src.UserData.Yaw(src.UserData.Count1-1)) < 0)
                Yaw = Yaw + sign(src.UserData.Yaw(src.UserData.Count1-1))*360;
            end

            src.UserData.Roll(src.UserData.Count1) = Roll;
            src.UserData.Pitch(src.UserData.Count1) = Pitch;
            src.UserData.Yaw(src.UserData.Count1) = Yaw;
        
            src.UserData.Count1 = src.UserData.Count1 + 1;    

            elapsedTime = toc; 
%             disp(src.UserData.fileID)
            fprintf(src.UserData.fileID,'%.2f,%s\n',[elapsedTime,line]);

            %update plot data by adding on the new points
            xdata = get(src.UserData.PlotObj1,'XData');
            ydata = get(src.UserData.PlotObj1,'YData');
            set(src.UserData.PlotObj1,'XData',[xdata elapsedTime], 'YData',[ydata Roll],"Color","blue");
    
            xdata2 = get(src.UserData.PlotObj2,'XData');
            ydata2 = get(src.UserData.PlotObj2,'YData');
            set(src.UserData.PlotObj2,'XData',[xdata2 elapsedTime], 'YData',[ydata2 Pitch],"Color","blue");
    
            xdata3 = get(src.UserData.PlotObj3,'XData');
            ydata3 = get(src.UserData.PlotObj3,'YData');
            set(src.UserData.PlotObj3,'XData',[xdata3 elapsedTime], 'YData',[ydata3 Yaw],"Color","blue");

        elseif (IMUdata(1)==1)

            Roll2 = IMUdata(2);
            Pitch2 = IMUdata(3);
            Yaw2 = IMUdata(4);
            

            if (abs(Roll2) > 45 && sign(Roll2 * src.UserData.Roll2(src.UserData.Count2-1)) < 0)
                Roll2 = Roll2 + sign(src.UserData.Roll2(src.UserData.Count2-1))*360;
            end
            if (abs(Pitch2) > 45 && sign(Pitch2 * src.UserData.Pitch2(src.UserData.Count2-1)) < 0)
                Pitch2 = Pitch2 + sign(src.UserData.Pitch2(src.UserData.Count2-1))*360;
            end
            if (abs(Yaw2) > 45 && sign(Yaw2 * src.UserData.Yaw2(src.UserData.Count2-1)) < 0)
                Yaw2 = Yaw2 + sign(src.UserData.Yaw2(src.UserData.Count2-1))*360;
            end
    
            src.UserData.Roll2(src.UserData.Count2) = Roll2;
            src.UserData.Pitch2(src.UserData.Count2) = Pitch2;
            src.UserData.Yaw2(src.UserData.Count2) = Yaw2;
        
            % Update the Count value of the serialport object.
            src.UserData.Count2 = src.UserData.Count2 + 1;
    
            elapsedTime = toc; 
%             disp(line)
            
            fprintf(src.UserData.fileID2,'%.2f,%s\n',[elapsedTime,line]);
        
            xdata = get(src.UserData.PlotObj1_5,'XData');
            ydata = get(src.UserData.PlotObj1_5,'YData');
            set(src.UserData.PlotObj1_5,'XData',[xdata elapsedTime], 'YData',[ydata Roll2],"Color","red");
    
            xdata2 = get(src.UserData.PlotObj2_5,'XData');
            ydata2 = get(src.UserData.PlotObj2_5,'YData');
            set(src.UserData.PlotObj2_5,'XData',[xdata2 elapsedTime], 'YData',[ydata2 Pitch2],"Color","red");

            xdata3 = get(src.UserData.PlotObj3_5,'XData');
            ydata3 = get(src.UserData.PlotObj3_5,'YData');
            set(src.UserData.PlotObj3_5,'XData',[xdata3 elapsedTime], 'YData',[ydata3 Yaw2],"Color","red");
        end

    end
    
end