function rgb = led_color_rgb(color)
%LED_COLOR_RGB Map a color name to an RGB triplet.
%
%   rgb = led_color_rgb(color)
%
% Returns:
%   rgb - uint8(3x1) [R;G;B] in 0..255

    if nargin < 1 || isempty(color)
        rgb = uint8([0; 0; 0]);
        return;
    end

    switch lower(string(color))
        case "red"
            rgb = uint8([255; 0; 0]);
        case "green"
            rgb = uint8([0; 255; 0]);
        case "blue"
            rgb = uint8([0; 0; 255]);
        case {"orange", "amber"}
            rgb = uint8([255; 165; 0]);
        case {"off", "black"}
            rgb = uint8([0; 0; 0]);
        otherwise
            rgb = uint8([0; 0; 0]);
    end
end
