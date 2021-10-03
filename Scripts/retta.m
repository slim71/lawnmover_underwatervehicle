function distPR = retta(position,p1,p2)
%Funzione che ritorna la distanza tra un punto e una retta costruita a
% partire da due suoi punti 
    m = p2-p1;
    coeff=m(2)/m(1);
    distPR = abs(position(2)-(coeff*(position(1)-p1(1))+p1(2)))/sqrt(1+coeff^2);
    %y-y1 = m*(x-x1) -> y = m*(x-x1)+y1 -> |y-[m(x-x1)+q]|
end