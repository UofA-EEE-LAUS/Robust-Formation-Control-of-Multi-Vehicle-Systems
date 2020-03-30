function [ delta ] = indegree( A )
    delta=diag(sum(A,2));
end

