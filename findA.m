function A = findA(Ia,Ib,p0,p1,p2)
    A = [Ia(1) - Ib(1)  p1(1) - p0(1)  p2(1) - p0(1);
         Ia(2) - Ib(2)  p1(2) - p0(2)  p2(2) - p0(2);
         Ia(3) - Ib(3)  p1(3) - p0(3)  p2(3) - p0(3)];
end

