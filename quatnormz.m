function   qn=quatnormz(q)
% quatnormz := normalaize a quaternion vector
%==========================================================================
% q    (4*1):= quaternion vector, un-normalized
% qn   (4*1):= quaternion vector, normalized
%==========================================================================
qn=q/sqrt(q(1)^2+q(2)^2+q(3)^2+q(4)^2);
%==========================================================================
end