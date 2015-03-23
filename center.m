% function to center image
function centerCoordinate = center(coordinate);
centerCoordinate = coordinate - ones(size(coordinate),1) * coordinate(1,:);
end