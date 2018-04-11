function [bool] = IsIntercept(xyz1, xyz2)%,limits1, limits2)


   p=[];
   bool=0;
   margin=0.1; %0.02
   min_dist=10000;
   for i=1:length(xyz1(:,1))
       min_vec=sqrt((xyz2(:,1)-xyz1(i,1)).^2+(xyz2(:,2)-xyz1(i,2)).^2+(xyz2(:,3)-xyz1(i,3)).^2);
       min_aux=min(min_vec);
%        for j=1:length(xyz2(:,1))
%            min_aux=sqrt(sum(xyz1(i,:)-xyz2(j,:)).^2);
           if min_aux <min_dist
               min_dist=min_aux;
           end    
%        end
   end
%     
%    min_vec=min(dist);
%    min_dist=min(min_vec);
   if min_dist <margin
        bool=1; 
   end

%    if limits1(1) > limits2(2) + margin | limits1(2) < limits2(1) - margin %check x    
%     bool=0;
%    elseif limits1(3) > limits2(4)+ margin | limits1(4) < limits2(3) - margin%check y
%     bool=0;
%    elseif limits1(5) > limits2(6) + margin | limits1(6) < limits2(5) -margin %check z
%     bool=0;
%    else
%     bool=1;
%    end
%     
%    if bool==1
%        xminf=min(limits1(1),limits2(1));
%        xmaxf=max(limits1(2),limits2(2));     
%        yminf=min(limits1(3),limits2(3));
%        ymaxf=max(limits1(4),limits2(4));      
%        zminf=min(limits1(5),limits2(5));
%        zmaxf=max(limits1(6),limits2(6));
%        
%        p=[xminf xmaxf yminf ymaxf zminf zmaxf];
%    end

end