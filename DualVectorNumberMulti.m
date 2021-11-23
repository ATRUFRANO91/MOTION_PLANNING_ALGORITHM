function DualVectorNumberMulti = DualVectorNumberMulti(DualNumb,DualVector)

[row,col] = size(DualVector);
i = 1;
a1 = DualNumb(1,1);
b1 = DualNumb(1,2);



while i <=row
    a2 = DualVector(i,1);
    b2 = DualVector(i,2);
    
    real = a1*a2;
    dual = a1*b2+a2*b1;
    
    
    DualVectorNumberMulti(i,1:2) = [real, dual];
    i = i+1;
end


DualVectorNumberMulti = DualVectorNumberMulti;



end