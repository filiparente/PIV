function [bool,p] = IsIntercept(limits1, limits2)

   p=[];
   bool=1;
   if limits1(1) > limits2(2) | limits1(2) < limits2(1) %check x    
    bool=0;
   elseif limits1(3) > limits2(4) | limits1(4) < limits2(3) %check y
    bool=0;
   elseif limits1(5) > limits2(6) | limits1(6) < limits2(5)  %check z
    bool=0;
   else
    bool=1;
   end
    
   if bool==1
       xminf=min(limits1(1),limits2(1));
       xmaxf=max(limits1(2),limits2(2));     
       yminf=min(limits1(3),limits2(3));
       ymaxf=max(limits1(4),limits2(4));      
       zminf=min(limits1(5),limits2(5));
       zmaxf=max(limits1(6),limits2(6));
       
       p=[xminf xmaxf yminf ymaxf zminf zmaxf];
   end

end