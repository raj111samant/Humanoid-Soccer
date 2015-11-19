



function quant = quantize(y,x,m)

global slope arrey output node

 
               if(isgoal(x,y))
        node = [node;y x]
        disp('exit')
        quant = 0;
        return
    end

for i=1:20

disp(y+arrey(i,2))
disp(x+arrey(i,1))

    if output(y+arrey(i,2),x+arrey(i,1))==1
disp('---------------------------')
        slope = arrey(i,2)/arrey(i,1)
        disp('---------------------------')
   
        
        output(y,x) = 0;

        
        if m~=slope
            %nodes = nodes+1;

disp('----------- node----------------')

            node = [node;y x] 
            disp('---------------------------')

        end
     
        quant =  quantize(y+arrey(i,2),x+arrey(i,1),slope);
        return
    
    end

end