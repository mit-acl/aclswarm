function [n, q, adjmat, sigma1, p, aligned, sigma2] = read_alignment(filename)
    file = fopen(filename);
    
    % n.b. both MATLAB and Eigen are colmajor
    
    n = fread(file, 1, 'uint8');
    q = fread(file, [n, 3], 'double');
    adjmat = fread(file, [n, n], 'uint8');
    sigma1 = fread(file, [1, n], 'uint8');
    p = fread(file, [n, 3], 'double');
    aligned = fread(file, [n, 3], 'double');
    sigma2 = fread(file, [1, n], 'uint8');
    
    % 0-based index to 1-based
    sigma1 = sigma1 + 1;
    sigma2 = sigma2 + 1;
    
    fclose(file);
end