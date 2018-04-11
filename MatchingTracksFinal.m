function [final_limits, ntrack,new_frame,assignments, total_cost]= MatchingTracksFinal(img,hsv_descr_old,hsv_descr_new,keypt_descr_old,keypt_descr_new,points_counts,points_counts_old,final_limits,limits,match_thresh,alfa,Beta,gama, ntrack,costOfNonAssignment,old_frame)


total_cost=zeros(length(hsv_descr_old),length(hsv_descr_new));
assignments=[];
unassignedTracks=[];
unassignedDetections=[];
new_frame=[];

if img>1 & length(hsv_descr_old)~=0
    for k=1:length(hsv_descr_new)  
                for old_obj=1:length(hsv_descr_old) %for each obj in the previous frame
                    if length(hsv_descr_old{old_obj})>0 %in case it's empty we dont compare

                        %COMPARING HSV HISTOGRAM
                        color_cost=abs(dot(hsv_descr_new{k}/norm(hsv_descr_new{k}),hsv_descr_old{old_obj}/norm(hsv_descr_old{old_obj})));
                        %COMPARING KEYPTS
                        if length(keypt_descr_new{k})~=0 & length(keypt_descr_old{old_obj})~=0 
                            [matches1, ~]= vl_ubcmatch(keypt_descr_new{k},keypt_descr_old{old_obj},match_thresh);
                            feature_cost= length(matches1(1,:))/length(keypt_descr_new{k}(1,:));
                            matches1=[];
                        else
                            feature_cost=0;
                        end
%                         comparing nr of points
                        if  abs(points_counts(k)-points_counts_old(old_obj))/max([points_counts(k) points_counts_old(old_obj)]) >0.8
                            points_cost=0;
                        else
                            points_cost=1;
                        end
                        
                        total_cost(old_obj,k)=alfa*color_cost+Beta*feature_cost+gama*points_cost;
                    end
                    
                end
    end
    
    if length(total_cost)>0
    
        [assignments,unassignedTracks,unassignedDetections] = Tracking(total_cost,costOfNonAssignment);  

        for i=1:length(assignments(:,1))          
            myobj=assignments(i,2);
            new_frame(myobj)=old_frame(assignments(i,1));
            final_limits{new_frame(myobj)}(:,img)=limits{myobj};
        end

        for i=1:length(unassignedDetections)   
            ntrack=ntrack+1;
            myobj=unassignedDetections(i);
            new_frame(myobj)=ntrack;
            final_limits{ntrack}(:,img)=limits{myobj};
        end
    
    end
elseif img>1 & length(hsv_descr_old)==0
    for i=1:length(hsv_descr_new)
        ntrack=ntrack+1;
        final_limits{ntrack}(:,img)=limits{i};
        new_frame(i)=ntrack;
    end
elseif img==1
    ntrack=length(limits);
    for i=1:length(limits)
        final_limits{i}(:,img)=limits{i};
        new_frame(i)=i;
    end


end



end