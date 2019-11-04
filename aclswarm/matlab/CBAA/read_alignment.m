function [n, q, adjmat, lastP, p, aligned, P] = read_alignment(filename)
    file = fopen(filename);
    
    % n.b. both MATLAB and Eigen are colmajor
    
    n = fread(file, 1, 'uint8');
    q = fread(file, [n, 3], 'double');
    adjmat = fread(file, [n, n], 'uint8');
    lastP = fread(file, [1, n], 'uint8');
    p = fread(file, [n, 3], 'double');
    aligned = fread(file, [n, 3], 'double');
    P = fread(file, [1, n], 'uint8');
    
    % 0-based index to 1-based
    lastP = lastP + 1;
    P = P + 1;
    
    fclose(file);
end