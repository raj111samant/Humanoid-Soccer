function find = search(xnode,ynode,g,bound)
global output
global test
global arrey counter

counter = counter + 1;
pause(0.01);
                            
                            %ans = [xnode,ynode];
                           % disp(ans)
     
                            
    f = g + h(xnode, ynode);
   % disp('g:')
  %  disp(g);
     %  disp('b:')
   % disp(bound);
    %    disp('f')
    %disp(f)
    
    disp('co: ')
    disp(xnode)
    disp(ynode)

    if f > bound
       find = f;
       %disp('f>b true')
       return; 
    end
    disp('counter:')
    disp(counter)
    a1 = input('hi:');
    
    output(ynode,xnode) = 1;

    if isgoal(xnode,ynode) == 1
        output = output*0;
        output(ynode,xnode) = 1;
        find = 10000;
        return;
    end
   
    
    min = 20000;
    
    for k = 1:20
        i = arrey(k,1);
        j = arrey(k,2);
        
            if  gt(xnode + i, 0) && lt(xnode + i, 640) && gt(ynode + j, 0) && lt(ynode + j, 480) 
                if eq(test((ynode + j),(xnode+i )) , 0)  && ne(output(ynode + j,xnode + i) , 1)
                    %if (~isequal([ynode + i , xnode + j],[0 , 0]))
                    
                        t = search(xnode+i,ynode+j,g+dist(xnode,ynode,xnode+i,ynode+j),bound); % dist = 1 prev version
                       
                        if t == 10000
                            find = 10000;
                            output(ynode,xnode) = 1;
                             disp('[')
                            disp(test((ynode),(xnode)))
                            disp(']')
                            disp('---------------')
                            ans = [xnode,ynode];
                            disp(ans)
                            disp('---------------')
                            imshow(output);
                            return
                        end
            
                        if t < min
                            min = t;
                        end 
                    %end
                end
            end
    end
    
    find=min;

end