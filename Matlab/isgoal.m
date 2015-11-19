function is = isgoal(xn,yn)
global xg yg
    if (lt(abs(xn-xg) , 5) && lt(abs(yn-yg), 5))
        is = 1; 
    else
        is = 0;
    end;
end