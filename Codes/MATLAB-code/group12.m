%/********************************************************************************
% 	Authors :   Group 12
%                 Harshvardhan 08005022,
%                 Akhil Tak 08005029,
%                 Vivek Surana 08005030,
%                 Vinay Surana 08005031,
% 			      IIT Bombay
% 
% 	Project Title : This code is for a autonomous defensive robot implemented on firebird V.
% 					
% 	Copyright (c) 2010, ERTS Lab IIT Bombay erts@cse.iitb.ac.in               -*- c -*-
%    	All rights reserved.
% 
% 	Redistribution and use in source and binary forms, with or without
%    	modification, are permitted provided that the following conditions are met:
% 
%   	* Redistributions of source code must retain the above copyright
%      	notice, this list of conditions and the following disclaimer.
% 
%    	* Redistributions in binary form must reproduce the above copyright
%      	notice, this list of conditions and the following disclaimer in
%      	the documentation and/or other materials provided with the
%      	distribution.
% 
%    	* Neither the name of the copyright holders nor the names of
%     	contributors may be used to endorse or promote products derived
%    	from this software without specific prior written permission.
% 
%    	* Source code can be used for academic purpose. 
% 	 
% 	For commercial use permission form the author needs to be taken.
% 
%   	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
%   	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%   	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
%   	ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
%   	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
%   	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
%   	SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
%   	INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
%   	CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
%   	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%   	POSSIBILITY OF SUCH DAMAGE. 
% 
%   	Software released under Creative Commence cc by-nc-sa licence.
%   	For legal information refer to: 
%   	http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode\

%***************************************************************************************/


%clear all previous varialbe
clear all;
clc;

%no. of frames per sec of top camera
NFPSTC=50;

%framegrabinterval
FGI=2;

%<<<<<<<<<<<< FOR TOP CAMERA >>>>>>>>>>>>>>

%create video object named vid with adaptor name winvideo & device id 2 & format of input is YUY2_320x240.
% Adaptor name & device id can be found by going to (in matlab) start->tools->imageacquisition
vid=videoinput('winvideo',2,'YUY2_320x240');

% Set the properties of the video object
set(vid, 'FramesPerTrigger', Inf);

%setting properties of video object to get video in RGB format
set(vid, 'ReturnedColorspace', 'rgb')

%set which frame to pick next. FrameGrabInterval property specifies how often frames are stored from the video stream.
vid.FrameGrabInterval = FGI;

%show video
preview(vid);

%-------------------------------------------
%<<<<<<<<<<<<<<   FOR FRONT CAMERA   >>>>>>>>>>>>
%create video object named fvid with adaptor name winvideo & device id 3 & format of input is YUY2_720x480.
% Adaptor name & device id can be found by going to (in matlab) start->tools->imageacquisition
fvid=videoinput('winvideo',3,'YUY2_720x480');

% Set the properties of the video object
set(fvid, 'FramesPerTrigger', Inf);

%setting properties of video object to get video in RGB format
set(fvid, 'ReturnedColorspace', 'rgb')

%set which frame to pick next. FrameGrabInterval property specifies how often frames are stored from the video stream.
%fvid.FrameGrabInterval = 1;

%show video
preview(fvid);

%making serial communication for xbee
%creates a serial port object s associated with the serial port  specified by port inside serial(''). For serial communication between robot and computer.
s = serial('COM8');

%open serial port connection.
%opening the xbee communication
fopen(s);

%start the video aquisition here
%start(vid);

%if multiple balls detected, then which ball to refer is provided by this
currobjno=1;

%approximate time of servo motor to rotate from one angle to other. assume to be constant for simplicity
t=0.3;

%There are two servo motors having different axis.
%So for angle 0 to 180 degree the first(lower)servo motor's axis is used
% and for angle 80 to 360 the second servo motor's axis is used.
%below define the two axis which is to be determined before running this program
axis1=[127,131];
axis2=[137,131];

%the minimum xdiameter and ydiameter of the laser to be approvd on detection
xthresh=15;
ythresh=15;

%the threshold to detect green ball on the red background.
greenballthresh=234;

%the threshold to detect green ball on the image  by the front camera.
fgreenballthresh=100;
fredballthresh=100;

%the green color threshold for laser on the ball
greenthreshforlaser=250;

%the displacement vectors i.e coordinates at two diffrent point of time.
vx = [0 0];
vy = [0 0];

% Set a loop that runs infinitely.
while(1)
    % Get the snapshot of the current frame
    rgb_1 = getsnapshot(vid);
    %imview(rgb)
    [a b c]=size(rgb_1);
    %taking green component to detect green balls on red background.
    rgb2=rgb_1(:,:,2);
    
    %storing balck white image in I
    I=zeros(a,b);
    
    %forming binary ( black and white) image according to the green ball threshold.
    for m=1:a
        for n=1:b
            if (rgb2(m,n)>greenballthresh) % Set threshold. (It is calliberated using snapshot of arean with ball)
                I(m,n)=1;
            else
                I(m,n)=0;
            end
        end
    end
    
    % Label all the connected components in the image
    bw=bwareaopen(I,100);  %remove all connected components (objects) that have fewer than 100 pixels from I, producing another binary image bw.
    bw = bwconncomp(bw, 8);  % find the connected components in bw.
    % Here we do the image blob analysis.
    % We get a set of properties for each labeled region.
    stats = regionprops(bw, 'BoundingBox');  %measures a set of properties for each labeled region in the image bw
    % Display the image
    imshow(rgb_1)
    %This is a loop to bound the green balls(or enemies) in a rectangular box.
    length(stats) % it gives number of object in image. here objects are balls
    %if length of this array is greater than one => ball(s) is(are) %detected
    if (length(stats)>=1)
        
        %if less no. of object detected than at previous time , start from first object.
        if(currobjno>length(stats))
            currobjno=1;
        end
        %immediately take the second image to detect the object's velocity,
        %so that the approx. time delay is Framegrabinterval(FGI)/no.of.frames.persec.of.top.camera(NFPSTC) sec
        rgb_2 = getsnapshot(vid);
        
        %compute the position of the center of the ball in the first image
        %and display the same on the image
        hold on
        bb = stats(currobjno).BoundingBox;
        c1=bb(1)+bb(3)/2;
        c2=bb(2)+bb(4)/2;
        vx(1)=c1;
        vy(1)=c2;
        rectangle('Position',bb,'EdgeColor','r','LineWidth',2)
        plot(c1,c2,'r*')
        hold off
        
        %Now process the second image
        %take the green component to detect green ball according to threshold
        rgb2=rgb_2(:,:,2);
        
        %Make a black and white image to detect green ball by white color.
        for m=1:a
            for n=1:b
                if (rgb2(m,n)>greenballthresh) % Set threshold
                    I(m,n)=1;
                else
                    I(m,n)=0;
                end
            end
        end
        
        % Label all the connected components in the image.
        bw=bwareaopen(I,100);
        bw = bwconncomp(bw, 8);
        % Here we do the image blob analysis.
        % We get a set of properties for each labeled region.
        stats2 = regionprops(bw, 'BoundingBox');
        
        %look for the same object in the second image assuming that it does get changed
        if (length(stats2)>=currobjno)
            %compute the center of the ball in the second image
            bb2 = stats2(currobjno).BoundingBox;
            c1=bb2(1)+bb2(3)/2;
            c2=bb2(2)+bb2(4)/2;
            vx(2)=c1;
            vy(2)=c2;
            
            %Compute the velocity = displacement / time between grabiing two frames
            % calculating velocity using current position and previous position of object and using the fact how fast the top camera captures frames.
            % in our case framse per second is 50 & we grab every second frame.So for time diff of two frames captured is to=2*1/50=1/25 so velocity is calculated
            % as v=(currpos-prevpos)/to
            vel = [(vx(2)-vx(1))*NFPSTC/FGI,(vy(2)-vy(1))*NFPSTC/FGI];
            
            %Now calculate there actual future coordinates of the ball in the image
            %where the gun rotating on the bot and the ball will cross each other
            newcoord=[vx(2)+vel(1)*t,vy(2)+vel(2)*t];
            
            %Now shift the coordinate making accurate axis as center to
            %calculate the angle from the axis of the gun mounted on the
            %bot
            if(newcoord(2)>=0)
                newcoord=[newcoord(1)-axis2(1),newcoord(2)-axis2(2)];
            else
                newcoord=[newcoord(1)-axis1(1),newcoord(2)-axis1(2)];
            end
            
            %now compute the absolute angle according the axis of the robot.
            angle=0;
            if(newcoord(1)==0)
                if(newcoord(2)<=0)
                    angle=180;
                else
                    angle=0;
                end
            elseif(newcoord(1)<0 && newcoord(2) >=0)
                angle=atan(-1*newcoord(2)/newcoord(1))*57.3;
            elseif(newcoord(1)>0 && newcoord(2) >=0)
                angle=180-atan(newcoord(2)/newcoord(1))*57.3;
            elseif(newcoord(1)>0 && newcoord(2) <=0)
                angle=180+atan(-1.0*newcoord(2)/newcoord(1))*57.3;
            elseif(newcoord(1)<0 && newcoord(2) <=0)
                angle=360-atan(newcoord(2)/newcoord(1))*57.3;
            end
            
            %Since xbee can communicate only one byte at a time, therefore
            %we decided to send  half the angle computed and double it
            %there on the bot after recieving.As we are making the gun to
            %rotate -6 to +4 from this angle send even angle (by halving it)is sufficient.
            angleby2=floor(angle/2);
            if(angleby2==0)
                angleby2=1;
                %there is problem in xbee communication that when we send char from 128 t0 159, reciever recieves 63 for all these values.
                %so we send these values with some constant offset as we are
                %using 1 to 180 for sending the angles.
            elseif(angleby2>=128 && angleby2<160)
                angleby2=angleby2+80;
            end
            
            %send the angleby2 to the robot using xbee comm.
            fprintf(s,char(angleby2));
            
            
            %%now starts the camera mounted on the robot to detect is
            %%the enemy get shooted or not by detectin a laser dot on the
            %%ball.
            hit=0; %boolean to verify perfect shoot or not.
            %As the gun rotates for ten degree , the camera mounted on the
            %bot keeps taking the images and keeps checking for perfect hit.
            startt=cputime;
            endt=cputime;
            
            %take the snapshot of the front camera
            frgb_1=getsnapshot(fvid);
            
            %Now detect the ball in the image taken by front camera
            [a b c]=size(frgb_1);
            fI=zeros(a,b);
            
            %do till 1.2 sec as it takes total time of .5 sec by the bot to rotate 10 degrees including delays.
            while(endt-startt<1.2)
                frgb1=frgb_1(:,:,1);
                frgb2=frgb_1(:,:,2);
                
                %Make a black and white image in which ball is identify by
                %white color object later.
                for m=1:a
                    for n=1:b
                        if (frgb2(m,n)>fgreenballthresh && frgb1(m,n)<fredballthresh)% Set threshold
                            fI(m,n)=1;
                        else
                            fI(m,n)=0;
                        end
                    end
                end
                %Display the image
                %imshow(fI)
                
                %Label all the connected components in the image
                fbw=bwareaopen(fI,1000);
                fbw = bwconncomp(fbw, 8);
                %Here we do the image blob analysis.
                %We get a set of properties for each labeled region.
                fstats = regionprops(fbw, 'BoundingBox');
                
                %now on each object (balls) detect the laser dot on them by taking the subimage of the ball
                for fobject = 1:length(fstats)
                    fbb = fstats(fobject).BoundingBox;
                    %low1=floor(fbb(2))
                    high1=floor(fbb(2)+fbb(3));
                    if(high1>a)
                        high1=a;
                    end
                    %print the subimage coordinates
                    ceil(fbb(2))
                    ceil(high1)
                    ceil(fbb(1))
                    floor(fbb(1)+fbb(3))
                    
                    %Take the subimage according to the bounding box
                    %detected and taking acount only xdistance(fbb(3)) as the length
                    %of the side of the square subimage.
                    frgb_2=frgb_1(ceil(fbb(2)):high1,ceil(fbb(1)):floor(fbb(1)+fbb(3)),:);
                    
                    [a1 b1 c1]=size(frgb_2);
                    frgb22=frgb_2(:,:,2);
                    
                    fI2=zeros(a1,b1);
                    %Mark the laser as white and rest as the black.
                    for m=1:a1
                        for n=1:b1
                            if (frgb22(m,n)>greenthreshforlaser)% Set threshold
                                fI2(m,n)=1;
                            else
                                fI2(m,n)=0;
                            end
                        end
                    end
                    %Now search for a component(laser dot)
                    %Label all the connected components in the image
                    imshow(fI2);
                    fbw2 = bwareaopen(fI2,10);
                    fbw2 = bwconncomp(fbw2, 8);
                    %Here we do the image blob analysis.
                    %We get a set of properties for each labeled region.
                    fstats2 = regionprops(fbw2, 'BoundingBox');
                    if(length(fstats2)>=1)
                        hit=1; %laser detected
                        break;
                    end
                end
                endt=cputime;
                %if detected then stop searching again and again
                if(hit==1)
                    break;
                end
                
                %ELSE take a new snapshot to check laser on the ball if fail before and continue the while loop.
                frgb_1=getsnapshot(fvid);
            end
            pause(1.2-(endt-startt));
            if(hit==1)
                fprintf(s,char(250)); %buzzer on signal.
                hit=0;
            end
            pause(2);
        end
        %now it should try other enemy(or ball).
        currobjno=currobjno+1;
    end
end
stop(vid);
stop(fvid);
fclose(s);
% Flush all the image data stored in the memory buffer.
flushdata(vid);
flushdata(fvid);
% Clear all variables
clear all