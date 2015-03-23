% function for coplanar posit algorithm
% givin imput image feature points, 3D object feature points, focal length
% of the camera, and the center of the image
% output is the rotation, translation matrix of the object and the error of
% estiamtion.
% keep only one solution which have less error.
function [rotation, translation, E] = coplanarPosit(imagePoints, objectPoints, focalLength, center)

if nargin == 3 % center is not given
	center = [0, 0];
end


% center the image
imagePoints(:,1) = (imagePoints(:,1) - center(1));
imagePoints(:,2) = (imagePoints(:,2) - center(2));


nbPoints = size(imagePoints, 1);

% calculate object vectors, object matrix and vector u, the norm of object plane
objectPoints = objectPoints - ones(nbPoints,1) * objectPoints(1,:);
objectVectors = objectPoints - ones(nbPoints,1) * objectPoints(1,:);
objectMatrix = pinv(objectVectors); 
u = findu(objectVectors);

oldE = 999;

%count 0
imageVectors = imagePoints - ones(nbPoints,1) * imagePoints(1,:);

IJ = (objectMatrix * imageVectors)';
I0 = IJ(1,:);
J0 = IJ(2,:);

[I1, J1, I2, J2] = solve2(I0, J0, u);

disp(' ================ First Solution ================ ');

%first solution
IVect1 = I1;
JVect1 = J1;
ISquare1 = IVect1*IVect1';
JSquare1 = JVect1*JVect1';
scale11 = sqrt(ISquare1); scale21 = sqrt(JSquare1);
row11 = IVect1/scale11; row21 = JVect1/scale21;
row31 = cross(row11, row21);
rotation1 = [row11; row21; row31];
scale1 = (scale11 + scale21)/2.0;
translation1 = [imagePoints(1,:), focalLength]/scale1;


[imagepoint1, valid1] = poseTrans(objectPoints, rotation1, translation1, focalLength);

if(valid1)
%     disp('loop1 ');
    [rot1, trans1, error1] = coplanarLoop(imagePoints, objectPoints, objectVectors, objectMatrix, u, nbPoints, focalLength, rotation1, translation1);
end




disp(' ================ Second Solution ================ ');

% second solution   
IVect2 = I2;
JVect2 = J2;
ISquare2 = IVect2*IVect2';
JSquare2 = JVect2*JVect2';
scale12 = sqrt(ISquare2); scale22 = sqrt(JSquare2);
row12 = IVect2/scale12; row22 = JVect2/scale22;
row32 = cross(row12, row22);
rotation2 = [row12; row22; row32];
scale2 = (scale12 + scale22)/2.0;
translation2 = [imagePoints(1,:), focalLength]/scale2;


[imagepoint2, valid2] = poseTrans(objectPoints, rotation2, translation2, focalLength);

if(valid2)
%     disp('loop2 ');
    [rot2, trans2, error2] = coplanarLoop(imagePoints, objectPoints, objectVectors, objectMatrix, u, nbPoints, focalLength, rotation2, translation2);
end
    
% choose one solusion which give less error
if(error1 < error2)
    rotation = rot1;
    translation = trans1;
    E = error1;
else
    rotation = rot2;
    translation = trans2;
    E = error2;
end

	disp(' ================ Final Result ================ ');

rotation
translation
E

[imagepoint valid]= poseTrans(objectPoints, rotation, translation, focalLength);
imagepoint;

end %end function