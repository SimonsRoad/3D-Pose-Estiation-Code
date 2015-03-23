% function for posit algorithm
% givin imput image feature points, 3D object feature points, focal length
% of the camera, and the center of the image
% output is the rotation, translation matrix of the object
function [rotation, translation] = Posit(imagePoints, objectPoints, focalLength, center)

if nargin == 3 % center is not given
	center = [0, 0];
end

% center the image
imagePoints(:,1) = (imagePoints(:,1) - imagePoints(1,1));
imagePoints(:,2) = (imagePoints(:,2) - imagePoints(1,2));

converged = 0;
count = 0;
imageDifference = 999;

nbPoints = size(imagePoints, 1);

% calculate object vectors, object matrix and vector u, the norm of object plane
objectVectors = objectPoints - ones(nbPoints,1) * objectPoints(1,:);
objectMatrix = pinv(objectVectors);=

oldSOPImagePoints = imagePoints;

while ~converged
	if count == 0
		imageVectors = imagePoints - ones(nbPoints,1) * imagePoints(1,:);
	else % count>0, we compute a SOP image first for POSIT
		epsilon = (1 + (objectVectors * rotation(3,:)')/translation(3));
		epsilon2 = epsilon * ones(1, 2);
		SOPImagePoints = imagePoints .* epsilon2;
		diffImagePoints = abs(round(SOPImagePoints - oldSOPImagePoints));
		imageDifference = ones(1, nbPoints) * diffImagePoints * ones(2,1);  % add up all coordinates
		oldSOPImagePoints = SOPImagePoints;
		imageVectors = SOPImagePoints - ones(nbPoints,1) * SOPImagePoints(1,:);
	end % else
    
    % calculate the pose
	IJ = (objectMatrix * imageVectors)';
	IVect = IJ(1,:);
    JVect = IJ(2,:);
	ISquare = IVect*IVect';
	JSquare = JVect*JVect';
	scale1 = sqrt(ISquare); scale2 = sqrt(JSquare);
	row1 = IVect/scale1; row2 = JVect/scale2;
	row3 = cross(row1, row2);
	rotation = [row1; row2; row3];
	scale = (scale1 + scale2)/2.0;
	translation = [imagePoints(1,:), focalLength]/scale;
		
	converged = (count > 0 & imageDifference < 1);
	count = count + 1;

end % while
