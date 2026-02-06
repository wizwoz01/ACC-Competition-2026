function WP = load_waypoints(fname)
% Returns struct WP.pickup, WP.dropoff, WP.hub as Nx2 double arrays

txt = fileread(fname);

WP.pickup  = parse_block(txt, "path_to_pickup:");
WP.dropoff = parse_block(txt, "path_to_dropoff:");
WP.hub     = parse_block(txt, "path_to_hub:");
end

function A = parse_block(txt, header)
i0 = strfind(txt, header);
if isempty(i0)
    error("Header not found: %s", header);
end
txt2 = txt(i0(1):end);

iL = strfind(txt2, '[');
iR = strfind(txt2, ']');
if isempty(iL) || isempty(iR)
    error("Brackets not found for: %s", header);
end

block = txt2(iL(1)+1 : iR(1)-1);

tok = regexp(block, '([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+)', 'tokens');
if isempty(tok)
    error("No waypoint pairs parsed for: %s", header);
end

A = zeros(numel(tok),2);
for k = 1:numel(tok)
    A(k,1) = str2double(tok{k}{1});
    A(k,2) = str2double(tok{k}{2});
end
end