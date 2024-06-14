#pragma once

static const char* vertex_shader_text =
    "#version 330 \n"
    "uniform mat4 image_from_world;\n"
    "out vec4 colorV;\n"
    "layout(location = 0) in vec3 vertex_pos_world_space;\n"
    "layout(location = 1) in vec3 color_in;\n"
    "layout(location = 2) in float scale_in;\n"
    "void main()\n"
    "{\n"
    "    gl_Position = image_from_world * vec4(vertex_pos_world_space, 1.0);	\n"
    "    gl_PointSize = scale_in;	\n"
    "    colorV = vec4(color_in, 1.0);\n"
    "}\n\0";

static const char* fragment_shader_text =
    "#version 330 \n"
    "in vec4 colorV;\n"
    "out vec4 color;\n"
    "void main()\n"
    "{\n"
    "    color = colorV;\n"
    "}\n\0";