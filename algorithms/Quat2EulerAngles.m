function [EA] = Quat2EulerAngles(Q)
% This function transforms quaternion parametrization of orientation to
% Euler angles parametrization

% See the following link for more details:
% https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.html

% Normalize the quaternions
% Q = normr(Q);

q0 = Q(:,1);
q1 = Q(:,2);
q2 = Q(:,3);
q3 = Q(:,4);

qx = Q(:,1);
qy = Q(:,2);
qz = Q(:,3);
qw = Q(:,4);

EA = [atan2d(-2*(q2.*q3 - q1.*q0), q0.^2 - q1.^2 - q2.^2 + q3.^2), ...
    asind(2*(q1.*q3 + q2.*q0)), ...
    atan2d(-2*(q1.*q2 - q3.*q0), q0.^2 + q1.^2 - q2.^2 - q3.^2)];

% EA = [atan2d(2*(qy.*qw - qx.*qz), 1-2*qy.^2-2*qz.^2), ...
%     asind(2*(qx.*qy + qz.*qw)), ...
%     atan2d(2*(qx.*qw - qy.*qz), 1-2*qx.^2-2*qz.^2)];

% Remove Complex Numbers
if ~isreal(EA)
    EA = real(EA);
end

end
