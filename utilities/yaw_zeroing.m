
% Extrahuj Yaw
yaw = atan2(2*(q(4)*q(3) + q(1)*q(2)), ...
            1 - 2*(q(2)^2 + q(3)^2));

% Sestav kvaternion pro Yaw = -yaw (kompenzace)
q_yaw_inv = [cos(yaw/2); 0; 0; -sin(yaw/2)];

q = [q(4), q(1:3)']';

% Vynásob - odstraní Yaw složku
q_zeroed = quatmultiply(q_yaw_inv', q');

q_zeroed = [q_zeroed(2:4), q_zeroed(1)]';

q_zeroed = q_zeroed / norm(q_zeroed);