% main program
% use camera to capture image, do glyph recognition frame by frame,
% find the four corners and center of the glyph as the feature points
% then do the coplaner posit algorithm to get the pose of the paper
% finally use the pose to map axis and cube into image frame

clc
close all
vidobj = videoinput('macvideo', 1);
triggerconfig(vidobj, 'manual');
preview(vidobj);
start(vidobj);

h=figure('Name','video');       
hold on; 

% world image plane
plane = [0 0 0; -1 1 0; 1 1 0; 1 -1 0; -1 -1 0];
plane = plane * 100;
focalLength = 640;
    
    
while ishandle(h)
    image = getsnapshot(vidobj);
    flushdata(vidobj);  
    rgbimg = ycbcr2rgb(image);
    resizeImg = imresize(rgbimg, 0.5);


    % glyph recogonition
    img = resizeImg;
    grayImage = rgb2gray(img);
    level = graythresh(img);
    BW = im2bw(img,level);
    
    imshow(resizeImg);
    hold on;

    %find boundary
    boundaryCount = 0;
    [B,L,N,A] = bwboundaries(BW);
%     figure; imshow(BW); hold on;
    for k=1:length(B),
        if(~sum(A(k,:)))
           for l=find(A(:,k))'
               boundary = B{l};
               s = size(boundary);
               if(s(1) < 100)
                   continue;
               end     
%                 plot(boundary(:,2),...
%                     boundary(:,1),'g','LineWidth',2);
               boundaryout = boundary;
               boundaryCount = boundaryCount + 1;
           end
        end
    end

    if(boundaryCount == 0)
%         pause(0);
        continue;
    end
    % find four corners
    x = boundaryout(:,2);
    y = boundaryout(:,1);
    xmin = min(x);
    xmax = max(x);
    ymin = min(y);
    ymax = max(y);
    indexXmin = find(x == xmin);
    indexXmax = find(x == xmax);
    indexYmin = find(y == ymin);
    indexYmax = find(y == ymax);
    left = boundaryout(indexXmin(1),:);
    right = boundaryout(indexXmax(1),:);
    top = boundaryout(indexYmin(1),:);
    bottom = boundaryout(indexYmax(1),:);
    p4 = [top;right;bottom;left];
    point4 = p4;
    point4(:,1) = p4(:,2);
    point4(:,2) = p4(:,1);
    center = sum(point4)/4;
    imagePoints = [center;point4];

%     plot(imagePoints(:,1), imagePoints(:,2),'ro');

    planeImage = imagePoints;
    planeImage = planeImage - ones(size(planeImage),1)*(size(BW)/2);
    % glyph recogonition end

    %pose estiamtion
    [rot, trans, e] = coplanarPosit(planeImage, plane, focalLength);

    % draw axis
    axis = [0 0 0; 100 0 0; 0 100 0; 0 0 100];
    [imageAxis valid]= poseTrans(axis, rot, trans, focalLength);
    imageAxis = imageAxis + ones(size(imageAxis),1) *(size(BW)/2);


    line([imageAxis(1,1),imageAxis(2,1)],[imageAxis(1,2),imageAxis(2,2)],'Color','r','LineWidth',2);
    line([imageAxis(1,1),imageAxis(3,1)],[imageAxis(1,2),imageAxis(3,2)],'Color','g','LineWidth',2);
    line([imageAxis(1,1),imageAxis(4,1)],[imageAxis(1,2),imageAxis(4,2)],'Color','b','LineWidth',2);

    % draw cube
    cb = [0 0 0; -1 -1 0; -1 1 0; 1 1 0; 1 -1 0; 1 -1 2; -1 -1 2; -1 1 2; 1 1 2];
    cb = cb*100;
    [iCube valid]= poseTrans(cb, rot, trans, focalLength);
    iCube = iCube + ones(size(iCube),1) *(size(BW)/2);

    for i = 2:8
    line([iCube(i,1),iCube(i+1,1)],[iCube(i,2),iCube(i+1,2)],'Color','r','LineWidth',2);
    end

    line([iCube(2,1),iCube(7,1)],[iCube(2,2),iCube(7,2)],'Color','r','LineWidth',2);
    line([iCube(2,1),iCube(5,1)],[iCube(2,2),iCube(5,2)],'Color','r','LineWidth',2);
    line([iCube(6,1),iCube(9,1)],[iCube(6,2),iCube(9,2)],'Color','r','LineWidth',2);
    line([iCube(4,1),iCube(9,1)],[iCube(4,2),iCube(9,2)],'Color','r','LineWidth',2);
    line([iCube(3,1),iCube(8,1)],[iCube(3,2),iCube(8,2)],'Color','r','LineWidth',2);

    
    drawnow;
%     pause(0);
    hold off

end
delete(vidobj);