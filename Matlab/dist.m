function cost = dist(x1,y1,x2,y2)
   
     %t = trend(x1,y1,x2,y2);
        
    
    %if ((x1~=x2) && (y1~=y2))
    %    cost = 2;  %--  diagonal movement
   % else    
   %     cost = 1.5; % -- straight path movement
    %    if (t == 1)
     %       cost = cost + 1;
      %  end
   % end
   cost = sqrt((abs(x1-x2)*abs(x1-x2)) + (abs(y1-y2)*abs(y1-y2)));
    
end