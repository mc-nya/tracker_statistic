function  ret  = distance( x,y )
%DISTANCE 此处显示有关此函数的摘要
%   此处显示详细说明
    tmp=x-y;
    if size(tmp,1)~=1
        tmp=tmp';
    end
    ret=sqrt(tmp*tmp');

end


