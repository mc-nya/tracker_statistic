function  ret  = distance( x,y )
%DISTANCE �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
    tmp=x-y;
    if size(tmp,1)~=1
        tmp=tmp';
    end
    ret=sqrt(tmp*tmp');

end


