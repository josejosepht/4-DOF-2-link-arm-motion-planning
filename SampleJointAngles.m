% Input: q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples to sample
% Output: qs -> num_samples x 4 matrix of joint angles,
%               all within joint limits

function qs = M1(q_min, q_max, num_samples)
%assign qs to be an initial empty list
qs=[];
for i=1:4
    %find uniform random samples between q_min and q_max
    qtemp = unifrnd(q_min(i),q_max(i),[num_samples 1]);
    qs=[qs ;qtemp'];
end
qs=qs';
end