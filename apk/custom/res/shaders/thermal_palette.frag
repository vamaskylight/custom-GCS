// VAMA thermal colour modes, the same as VGCS (vgcs/video/thermal_palette.py,
// client request 2026-10-07). Each pixel's grey level picks its colour from
// one row of thermal_palettes.png. FlightDisplayViewVideoOutput.qml uses it as
// the video's layer effect.
//
// After a change, run python apk/make_thermal_palettes.py from the repo root:
// it compiles this file into thermal_palette.frag.qsb (Qt's shader tool).

#version 440

layout(location = 0) in vec2 qt_TexCoord0;
layout(location = 0) out vec4 fragColor;

layout(std140, binding = 0) uniform buf {
    mat4 qt_Matrix;
    float qt_Opacity;
    // The palette's row in the table, at the row's centre (0 to 1).
    float row;
    // Where the picture is inside the item: x, y, width, height (0 to 1).
    // Outside it (black bars) the pixels are left as they are.
    vec4 picture;
};

layout(binding = 1) uniform sampler2D source;
layout(binding = 2) uniform sampler2D table;

void main()
{
    vec4 c = texture(source, qt_TexCoord0);
    vec2 p = (qt_TexCoord0 - picture.xy) / picture.zw;
    if (any(lessThan(p, vec2(0.0))) || any(greaterThan(p, vec2(1.0)))) {
        fragColor = c * qt_Opacity;
        return;
    }
    // The grey level as VGCS takes it (Qt's qGray weights), 0 to 255.
    vec3 rgb = c.a > 0.0 ? c.rgb / c.a : vec3(0.0);
    float level = floor(dot(rgb, vec3(11.0, 16.0, 5.0)) / 32.0 * 255.0 + 0.5);
    // The centre of that level's pixel in the table, so its colour comes out exactly.
    vec3 mapped = texture(table, vec2((level + 0.5) / 256.0, row)).rgb;
    fragColor = vec4(mapped * c.a, c.a) * qt_Opacity;
}
