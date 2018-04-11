function limits = getLimits(goodxyz)
    limits=zeros(6,1);
            %LIMITS
            limits(1)=min(goodxyz(:,1));
            limits(2)=max(goodxyz(:,1));
            limits(3)=min(goodxyz(:,2));
            limits(4)=max(goodxyz(:,2));
            limits(5)=min(goodxyz(:,3));
            limits(6)=max(goodxyz(:,3));
end