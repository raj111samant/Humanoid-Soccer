function star = ida(xs,ys)
global output xr yr counter


bound = h(xs,ys);

    while 1 
        output = output.*0;  %clear our estimation and start all over
        t = search(xr,yr,0,bound);
       % disp(counter);
       % disp(t)
 
         if t == 10000
            star = 10000;
            return
        elseif t == 20000
            star =  20000;
            return;
         else
            bound = t;
            %disp(bound)
            
         end
    end
end