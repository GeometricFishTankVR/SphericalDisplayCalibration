function [ S ] = skew_sym( v )
%convert a skew symmetric vector to matrix
%   v: 3x1 vector
%   return 3x3 mat

S =  [0      -v(3) v(2);
          v(3)  0      -v(1);
          -v(2) v(1)  0;];
end

