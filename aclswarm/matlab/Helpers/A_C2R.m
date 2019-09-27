function AnR = A_C2R(An)

numA = size(An,3);
n = size(An,1);

AnR = zeros(2*n, 2*n, numA);

for k = 1 : numA
    
    for i = 1 : n
        for j = 1 : n
            
            lij = An(i,j);
            re = real(lij);
            im = imag(lij);
            
            AnR(2*i-1:2*i, 2*j-1:2*j, k) = [re, -im;  im, re];
              
        end 
    end
    
end

end

