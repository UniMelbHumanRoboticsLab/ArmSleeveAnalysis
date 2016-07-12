%WHIST  Weighted histogram.
%   N = WHIST(Y,X,W), where X is a vector, returns the distribution of Y
%   among bins with centers specified by X. Each element of Y is weighted
%   by the corresponding element of W. The first bin includes data between
%   -inf and the first center and the last bin includes data between the 
%   last bin and inf. 
%
%   Class support for inputs Y, X, W: 
%      float: double, single
%
%   See also HIST, HISTC.
%
%   Vincent Crocher - The University of Melbourne - 2016

function N = whist(Y, X, W)

    if(length(Y)~=length(W))
       error('whist: Y and W should have same length.');
    end

    if(isempty(W))
        W=ones(size(Y));
    end
    
    %Compute edges from bin centers
    edges=zeros(length(X)-1,1);
    for i=1:length(X)-1
       edges(i)=X(i)+(X(i+1)-X(i))/2.;
    end
    
    N=zeros(1,length(X));
    
    %Count number of elements in each bin
    for i=1:length(Y)
       bin_idx=select_bin(Y(i));
       N(bin_idx)=N(bin_idx)+W(i);
    end

    
    %Find corresponding bin for value v
    function idx=select_bin(v)
        %First bin starts at -inf
        if(v<edges(1))
            idx=1;
            return
        end
        
        for k=1:length(edges)-1
            if(v>edges(k) && v<edges(k+1))
                idx=k+1;
                return
            end
        end
        
        %Last bin ends at inf
        if(v>edges(end))
            idx=length(edges)+1;
            return
        end
    end
end