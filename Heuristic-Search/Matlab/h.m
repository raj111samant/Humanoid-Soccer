%
%function hue= h(xh,yh)
%global xg yg

 %   if ( (xh- xg)*(xh-xg) > (yh-yg)*(yh-yg))
  %      hue = abs(xh- xg);
       % disp('in if')
       % disp(hue)
 %   else
   %     hue = abs(yh- yg);
       % disp('in else')
       % disp(hue)
%    end
%end


function hue= h(xh,yh)
global xg yg

  %  if (abs(xh-xg) < abs(yh-yg))
 %       temp = abs(xh-xg)*2;
 %   elseif (abs(xh-xg) > abs(yh-yg))
 %       temp = abs(yh-yg)*2;
 %   else 
%        temp = abs(yh-yg)*2;
 %   end
%    
  %  hue = temp +  abs(abs(xh-xg) - abs(yh-yg))*1.5;
  
  
  hue =  sqrt((abs(xh-xg)*abs(xh-xg)) + (abs(yh-yg)*abs(yh-yg)));
  

  
end