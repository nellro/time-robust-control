function fh = fill_between(x,ybot,ytop)
    if ~isvector(x) || ~isvector(ybot) || ~isvector(ytop)
        error('plot_between takes 3 vectors')
    end
    
    if length(x) ~= length(ybot) || length(x) ~= length(ytop)
        error('all inputs must be the same length')
    end
    
    ax = gca;
    was_hold = ishold(ax);
    hold(ax,'on')

    output = [];
    for i=1:length(x)
        % If y2 is below y1, then we don't need to add this point.
        if ytop(i) <= ybot(i)
            % But first, if that wasn't true for the previous point, then add the
            % crossing.
            if i>1 && ytop(i-1) > ybot(i-1)
                neighborhood = [x(i-1), x(i); ybot(i-1), ybot(i); ytop(i-1), ytop(i)];
                t = calct(neighborhood);
                output(:,end+1) = interp(t,neighborhood);
            end
            
            % Don't add this point.
        else
        % Otherwise y2 is above y1, and we do need to add this point. But first
        % ...
            % ... if that wasn't true for the previous point, then add the 
            % crossing.
            if i>1 && ytop(i-1) <= ybot(i-1)
                neighborhood = [x(i-1), x(i); ybot(i-1), ybot(i); ytop(i-1), ytop(i)];
                t = calct(neighborhood);
                output(:,end+1) = interp(t,neighborhood);
            end

            % Add this point.
            output(:,end+1) = [x(i); ytop(i); ybot(i)];
        end
    end
    
    xout = output(1,:);
    topout = output(2,:);
    botout = output(3,:);
    fh = fill([xout fliplr(xout)],[botout fliplr(topout)],[.929 .694 .125]);
    fh.EdgeColor = 'none';
    uistack(fh,'bottom')
    if ~was_hold
        hold(ax,'off')
    end
end

function t = calct(n)
    t = (n(3,1)-n(2,1))/(n(3,1)-n(2,1)-n(3,2)+n(2,2));
end

function out = interp(t,in)
    out = in(:,1) + t*(in(:,2) - in(:,1));
end
