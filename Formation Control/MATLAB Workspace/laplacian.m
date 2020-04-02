function [ L ] = laplacian( A )
    L = indegree(A) - A;
end

