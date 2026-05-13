function meas = Measurement(pos_car, pos_landmarks)
 % Measurement equation
dx = pos_landmarks(1,:)-pos_car(1,:);
dy = pos_landmarks(2,:)-pos_car(2,:);
theta = atan2((dy),(dx));
theta = wrapToPi(theta);
meas = [sqrt((dx).^2 + (dy).^2);theta];
end
