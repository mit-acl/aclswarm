% Convert complex gains to real
function LnR = C2R(Ln)

numL = size(Ln,3);
n = size(Ln,1);

LnR = zeros(2*n, 2*n, numL);

for k = 1 : numL
    
    for i = 1 : n
        for j = 1 : n
            
            lij = Ln(i,j);
            re = real(lij);
            im = imag(lij);
            
            LnR(2*i-1:2*i, 2*j-1:2*j, k) = [re, -im;  im, re];
              
        end 
    end
    
end

end

