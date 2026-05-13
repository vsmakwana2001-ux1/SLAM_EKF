function meas = Measurement(pos_car, pos_landmarks)
% Measurement equation
meas = [pos_landmarks(1,:)-pos_car(1);pos_landmarks(2,:)-pos_car(2)];
end
