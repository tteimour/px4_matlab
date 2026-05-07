function S = skew(v)
% skew — 3x3 skew-symmetric cross-product matrix of a 3-vector.
%   v x w = skew(v) * w
    S = [   0,   -v(3),  v(2);
          v(3),    0,   -v(1);
         -v(2),  v(1),    0  ];
end
